/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_gnrc_mac
 * @file
 * @brief       Internal functions of GNRC_WMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>
#include <net/gnrc.h>
#include <net/gnrc/mac/internal.h>
#include <net/gnrc/mac/types.h>
#include <net/gnrc/mac/mac.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

int gnrc_mac_get_dstaddr(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[])
{
    int res;
    gnrc_netif_hdr_t* netif_hdr;

    if(!pkt)
        return -ENODEV;

    netif_hdr = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

    if(netif_hdr) {
        if((res = netif_hdr->dst_l2addr_len) <= 0)
            return -ENOENT;

        *pointer_to_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
        return res;

    } else {
        return -ENOENT;
    }
}

void* gnrc_mac_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type)
{
    while(pkt != NULL)
    {
        if(pkt->type == type) {
            return pkt->data;
        }
        pkt = pkt->next;
    }
    return NULL;
}

/* TX queue handling */
int _find_neighbour(gnrc_mac_tx_neighbour_t* neighbours, uint8_t* dst_addr, int addr_len)
{
	//gnrc_mac_tx_neighbour_t* neighbours = gnrc_mac->tx.neighbours;

	for(int i = 0; i <= GNRC_MAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr_len == addr_len) {
            if(memcmp(&(neighbours[i].l2_addr), dst_addr, addr_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/* Free first empty queue that is not active */
int _free_neighbour(gnrc_mac_tx_t* tx)
{
	gnrc_mac_tx_neighbour_t* neighbours = tx.neighbours;

	/* Don't attempt to free broadcast neighbour, so start at index 1 */
	for(int i = 1; i <= GNRC_MAC_NEIGHBOUR_COUNT; i++) {
        if( (gnrc_priority_pktqueue_length(&(neighbours[i].queue)) == 0) &&
            (&neighbours[i] != tx.current_neighbour) ) {
            /* Mark as free */
            neighbours[i].l2_addr_len = 0;
            return i;
        }
	}
    return -1;
}

int _alloc_neighbour(gnrc_mac_tx_neighbour_t* neighbours)
{
    for(int i = 0; i <= GNRC_MAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr_len == 0) {
            gnrc_priority_pktqueue_init(&(neighbours[i].queue));
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

void _init_neighbour(gnrc_mac_tx_neighbour_t* neighbour, uint8_t* addr, int len)
{
    assert(neighbour != NULL);
    assert(addr  != NULL);
    assert(len > 0);

    neighbour->l2_addr_len = len;
    neighbour->phase = GNRC_MAC_PHASE_UNINITIALIZED;
    memcpy(&(neighbour->l2_addr), addr, len);
}

gnrc_priority_pktqueue_node_t* _alloc_pktqueue_node(gnrc_mac_tx_t* tx)
{
    /* search for free packet_queue_node */
    for (size_t i = 0; i < GNRC_MAC_TX_QUEUE_SIZE; i++) {
        if((tx->_queue_nodes[i].pkt == NULL) &&
           (tx->_queue_nodes[i].next == NULL))
        {
            return &tx->_queue_nodes[i];
        }
    }
    return NULL;
}

bool _queue_tx_packet(gnrc_mac_tx_t* tx, uint32_t priority, gnrc_pktsnip_t* pkt)
{
	gnrc_mac_tx_neighbour_t* neighbour;
    int neighbour_id;

    if(_packet_is_broadcast(pkt)) {
        /* Broadcast queue is neighbour 0 by definition */
        neighbour_id = 0;
        neighbour = _get_neighbour(tx, neighbour_id);

    } else {
        uint8_t* addr;
        int addr_len;
        bool neighbour_known = true;

        /* Get destination address of packet */
        addr_len = _get_dest_address(pkt, &addr);
        if(addr_len <= 0) {
            DEBUG("[gnrc_mac-int] Packet has no destination address\n");
            gnrc_pktbuf_release(pkt);
            return false;
        }

        /* Search for existing queue for destination */
        neighbour_id = _find_neighbour(tx->neighbours, addr, addr_len);

        /* Neighbour node doesn't have a queue yet */
        if(neighbour_id < 0) {
            neighbour_known = false;

            /* Try to allocate neighbour entry */
            neighbour_id = _alloc_neighbour(tx->neighbours);

            /* No neighbour entries left */
            if(neighbour_id < 0) {
                DEBUG("[gnrc_mac-int] No neighbour entries left, maybe increase "
                      "GNRC_MAC_NEIGHBOUR_COUNT for better performance\n");

                /* Try to free an unused queue */
                neighbour_id = _free_neighbour(tx);

                /* All queues are in use, so reject */
                if(neighbour_id < 0) {
                    DEBUG("[gnrc_mac-int] Couldn't allocate tx queue for packet\n");
                    gnrc_pktbuf_release(pkt);
                    return false;
                }
            }
        }

        neighbour = _get_neighbour(tx, neighbour_id);

        if(!neighbour_known) {
            _init_neighbour(neighbour, addr, addr_len);
        }

    }

    gnrc_priority_pktqueue_node_t* node = _alloc_pktqueue_node(tx);
    if(node) {
    	gnrc_priority_pktqueue_node_init(node, priority, pkt);
    	gnrc_priority_pktqueue_push(&neighbour->queue, node);
    }else {
        DEBUG("[gnrc_mac-int] Can't push to neighbour #%d's queue, no entries left\n",
                neighbour_id);
        gnrc_pktbuf_release(pkt);
        return false;
    }

    DEBUG("[gnrc_mac-int] Queuing pkt to neighbour #%d\n", neighbour_id);

    return true;
}


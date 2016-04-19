/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iQueueMAC
 * @file
 * @brief       Internal functions of iQueueMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>
#include <periph/rtt.h>
#include <net/gnrc.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>

#include "include/iqueuemac_internal.h"
#include "include/iqueue_types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[])
{
    int res;
    gnrc_netif_hdr_t* netif_hdr;

    if(!pkt)
        return -ENODEV;

    netif_hdr = (gnrc_netif_hdr_t*) pkt->data;
    if( (res = netif_hdr->dst_l2addr_len) <= 0)
        return -ENOENT;

    *pointer_to_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
    return res;
}

/******************************************************************************/

/* Find a payload based on it's protocol type */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type)
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

/******************************************************************************/

int _find_neighbour(iqueuemac_t* iqueuemac, uint8_t* dst_addr, int addr_len)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->neighbours;

    for(int i = 0; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr.len == addr_len) {
            if(memcmp(&(neighbours[i].l2_addr.addr), dst_addr, addr_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/******************************************************************************/

/* Free first empty queue that is not active */
int _free_neighbour(iqueuemac_t* iqueuemac)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->neighbours;

    for(int i = 0; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if( packet_queue_length(&(neighbours[i].queue)) == 0) {
            /* Mark as free */
            neighbours[i].l2_addr.len = 0;
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

int _alloc_neighbour(iqueuemac_t* iqueuemac)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->neighbours;

    for(int i = 0; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr.len == 0) {
            packet_queue_init(&(neighbours[i].queue),
                              iqueuemac->_queue_nodes,
                              (sizeof(iqueuemac->_queue_nodes) / sizeof(packet_queue_node_t)));
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

void _init_neighbour(iqueuemac_tx_neighbour_t* neighbour, uint8_t* addr, int len)
{
    assert(neighbour != NULL);
    assert(addr  != NULL);
    assert(len > 0);

    neighbour->l2_addr.len = len;
    neighbour->cp_phase = IQUEUEMAC_PHASE_UNINITIALIZED;
    memcpy(&(neighbour->l2_addr.addr), addr, len);
}

/******************************************************************************/

/******************************************************************************/

bool _queue_tx_packet(iqueuemac_t* iqueuemac,  gnrc_pktsnip_t* pkt)
{

    iqueuemac_tx_neighbour_t* neighbour;
    int neighbour_id;

    if(_packet_is_broadcast(pkt)) {
        /* Broadcast queue is neighbour 0 by definition */
        neighbour_id = 0;
        neighbour = _get_neighbour(iqueuemac, neighbour_id);

    } else {
        uint8_t* addr;
        int addr_len;
        bool neighbour_known = true;

        /* Get destination address of packet */
        addr_len = _get_dest_address(pkt, &addr);
        if(addr_len <= 0) {
            DEBUG("[iqueuemac-int] Packet has no destination address\n");
            gnrc_pktbuf_release(pkt);
            return false;
        }

        /* Search for existing queue for destination */
        neighbour_id = _find_neighbour(iqueuemac, addr, addr_len);

        /* Neighbour node doesn't have a queue yet */
        if(neighbour_id < 0) {
            neighbour_known = false;

            /* Try to allocate neighbour entry */
            neighbour_id = _alloc_neighbour(iqueuemac);

            /* No neighbour entries left */
            if(neighbour_id < 0) {
                DEBUG("[iqueuemac-int] No neighbour entries left, maybe increase "
                      "iqueuemac_NEIGHBOUR_COUNT for better performance\n");

                /* Try to free an unused queue */
                neighbour_id = _free_neighbour(iqueuemac);

                /* All queues are in use, so reject */
                if(neighbour_id < 0) {
                    DEBUG("[iqueuemac-int] Couldn't allocate tx queue for packet\n");
                    gnrc_pktbuf_release(pkt);
                    return false;
                }
            }
        }

        neighbour = _get_neighbour(iqueuemac, neighbour_id);

        if(!neighbour_known) {
            _init_neighbour(neighbour, addr, addr_len);
        }

    }

    if(packet_queue_push(&(neighbour->queue), pkt, 0) == NULL) {
        DEBUG("[iqueuemac-int] Can't push to neighbour #%d's queue, no entries left\n",
                neighbour_id);
        gnrc_pktbuf_release(pkt);
        return false;
    }

    DEBUG("[iqueuemac-int] Queuing pkt to neighbour #%d\n", neighbour_id);

    return true;
}

/******************************************************************************/



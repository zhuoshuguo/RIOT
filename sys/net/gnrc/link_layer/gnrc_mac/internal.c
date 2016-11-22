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

#if ((GNRC_MAC_TX_QUEUE_SIZE != 0)||(GNRC_MAC_RX_QUEUE_SIZE != 0))
gnrc_priority_pktqueue_node_t* _alloc_pktqueue_node(gnrc_priority_pktqueue_node_t* nodes, uint32_t size)
{
    /* search for free packet_queue_node */
    for (size_t i = 0; i < size; i++) {
        if((nodes[i].pkt == NULL) &&
           (nodes[i].next == NULL)) {
            return &nodes[i];
        }
    }

    return NULL;
}
#endif
/* endif for `#if((GNRC_MAC_TX_QUEUE_SIZE != 0)||(GNRC_MAC_RX_QUEUE_SIZE != 0))` */

#if GNRC_MAC_TX_QUEUE_SIZE != 0
#if GNRC_MAC_NEIGHBOR_COUNT != 0
int _gnrc_mac_find_neighbor(gnrc_mac_tx_t* tx, const uint8_t* dst_addr, int addr_len)
{
    gnrc_mac_tx_neighbor_t* neighbors;
    neighbors = tx->neighbors;

    for(int i = 0; i <= (signed)GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if(neighbors[i].l2_addr_len == addr_len) {
            if(memcmp(&(neighbors[i].l2_addr), dst_addr, addr_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/* Free first empty queue that is not active */
int _gnrc_mac_free_neighbor(gnrc_mac_tx_t* tx)
{
    gnrc_mac_tx_neighbor_t* neighbors;
    neighbors = tx->neighbors;

    /* Don't attempt to free broadcast neighbor, so start at index 1 */
    for(int i = 1; i <= (signed)GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if((gnrc_priority_pktqueue_length(&(neighbors[i].queue)) == 0) &&
           (&neighbors[i] != tx->current_neighbor)) {
            /* Mark as free */
            neighbors[i].l2_addr_len = 0;
            return i;
        }
    }
    return -1;
}

int _gnrc_mac_alloc_neighbor(gnrc_mac_tx_t* tx)
{
    gnrc_mac_tx_neighbor_t* neighbors;
    neighbors = tx->neighbors;

    for(int i = 0; i <= (signed)GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if(neighbors[i].l2_addr_len == 0) {
            gnrc_priority_pktqueue_init(&(neighbors[i].queue));
            return i;
        }
    }
    return -1;
}

void _gnrc_mac_init_neighbor(gnrc_mac_tx_neighbor_t* neighbor, const uint8_t* addr, int len)
{
    assert(neighbor != NULL);
    assert(addr  != NULL);
    assert(len > 0);

    neighbor->l2_addr_len = len;
    neighbor->phase = GNRC_MAC_PHASE_UNINITIALIZED;
    memcpy(&(neighbor->l2_addr), addr, len);
}
#endif
/* endif for #if GNRC_MAC_NEIGHBOR_COUNT != 0 */

bool gnrc_mac_queue_tx_packet(gnrc_mac_tx_t* tx, uint32_t priority, gnrc_pktsnip_t* pkt)
{
#if GNRC_MAC_NEIGHBOR_COUNT == 0

    gnrc_priority_pktqueue_node_t* node;
    node = _alloc_pktqueue_node(tx->_queue_nodes, GNRC_MAC_TX_QUEUE_SIZE);

    if(node) {
        gnrc_priority_pktqueue_node_init(node, priority, pkt);
        gnrc_priority_pktqueue_push(tx->queue, node);
        return true;
    }

    DEBUG("[gnrc_mac-int] Can't push to TX queue, no entries left\n");
    return false;

#else
/* else for `#if GNRC_MAC_NEIGHBOR_COUNT == 0` */

    gnrc_mac_tx_neighbor_t* neighbor;
    int neighbor_id;

    if(gnrc_mac_chk_pkt_bcast(pkt)) {
        /* Broadcast queue is neighbor 0 by definition */
        neighbor_id = 0;
        neighbor = gnrc_mac_get_neighbor(tx, neighbor_id);

    } else {
        uint8_t* addr;
        int addr_len;
        bool neighbor_known = true;

        /* Get destination address of packet */
        addr_len = gnrc_mac_get_dstaddr(pkt, &addr);
        if(addr_len <= 0) {
            DEBUG("[gnrc_mac-int] Packet has no destination address\n");
            return false;
        }

        /* Search for existing queue for destination */
        neighbor_id = _gnrc_mac_find_neighbor(tx, addr, addr_len);

        /* neighbor node doesn't have a queue yet */
        if(neighbor_id < 0) {
            neighbor_known = false;

            /* Try to allocate neighbor entry */
            neighbor_id = _gnrc_mac_alloc_neighbor(tx);

            /* No neighbor entries left */
            if(neighbor_id < 0) {
                DEBUG("[gnrc_mac-int] No neighbor entries left, maybe increase "
                      "GNRC_MAC_NEIGHBOR_COUNT for better performance\n");

                /* Try to free an unused queue */
                neighbor_id = _gnrc_mac_free_neighbor(tx);

                /* All queues are in use, so reject */
                if(neighbor_id < 0) {
                    DEBUG("[gnrc_mac-int] Couldn't allocate tx queue for packet\n");
                    return false;
                }
            }
        }

        neighbor = gnrc_mac_get_neighbor(tx, neighbor_id);

        if(!neighbor_known) {
            _gnrc_mac_init_neighbor(neighbor, addr, addr_len);
        }

    }

    gnrc_priority_pktqueue_node_t* node;
    node = _alloc_pktqueue_node(tx->_queue_nodes, GNRC_MAC_TX_QUEUE_SIZE);
    if(node) {
        gnrc_priority_pktqueue_node_init(node, priority, pkt);
        gnrc_priority_pktqueue_push(&neighbor->queue, node);
        DEBUG("[gnrc_mac-int] Queuing pkt to neighbor #%d\n", neighbor_id);
        return true;
    }

    DEBUG("[gnrc_mac-int] Can't push to neighbor #%d's queue, no entries left\n",
            neighbor_id);
    return false;

#endif
/* endif for `#if GNRC_MAC_NEIGHBOR_COUNT == 0` */
}
#endif
/* endif for `#if GNRC_MAC_TX_QUEUE_SIZE != 0` */

#if GNRC_MAC_RX_QUEUE_SIZE != 0
bool gnrc_mac_queue_rx_packet(gnrc_mac_rx_t* rx, uint32_t priority, gnrc_pktsnip_t* pkt)
{
    gnrc_priority_pktqueue_node_t* node;
    node = _alloc_pktqueue_node(rx->_queue_nodes, GNRC_MAC_RX_QUEUE_SIZE);

    if(node) {
        gnrc_priority_pktqueue_node_init(node, priority, pkt);
        gnrc_priority_pktqueue_push(&rx->queue, node);
        return true;
    }

    DEBUG("[gnrc_mac] Can't push RX packet @ %p, no entries left\n", pkt);
    return false;
}
#endif
/* endif for `#if GNRC_MAC_RX_QUEUE_SIZE != 0` */

#if GNRC_MAC_DISPATCH_BUFFER_SIZE != 0
void gnrc_mac_dispatch(gnrc_pktsnip_t* buffer[])
{
    assert(buffer != NULL);

    for(unsigned i = 0; i < GNRC_MAC_DISPATCH_BUFFER_SIZE; i++) {
        if(buffer[i]) {
            if (!gnrc_netapi_dispatch_receive(buffer[i]->type, GNRC_NETREG_DEMUX_CTX_ALL, buffer[i])) {
                DEBUG("Unable to forward packet of type %i\n", buffer[i]->type);
                gnrc_pktbuf_release(buffer[i]);
            }
            buffer[i] = NULL;
        }
    }
}
#endif
/* endif for `#if GNRC_MAC_DISPATCH_BUFFER_SIZE != 0` */


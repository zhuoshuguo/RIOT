/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_lwmac
 * @file
 * @brief       Internal functions of LWMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include <stdbool.h>
#include <periph/rtt.h>
#include <net/gnrc.h>

#include <net/gnrc/gnrc_mac_type/packet_queue.h>
#include <net/gnrc/gnrc_mac_type/gnrc_mac_internal.h>

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

// TODO: Don't use global variables
void _set_netdev_state(gnrc_mac_t* gnrc_mac, netopt_state_t devstate)
{
	gnrc_mac->netdev2_driver->set(gnrc_mac->netdev->dev,
                               NETOPT_STATE,
                               &devstate,
                               sizeof(devstate));
}

/******************************************************************************/

netopt_state_t _get_netdev_state(gnrc_mac_t* gnrc_mac)
{
    netopt_state_t state;
	if (0 < gnrc_mac->netdev2_driver->get(gnrc_mac->netdev->dev,
                                       NETOPT_STATE,
                                       &state,
                                       sizeof(state))) {
        return state;
    }
    return -1;
}


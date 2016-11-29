/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @author  Martine Lenders <mlenders@inf.fu-berlin.de>
 */

#include "net/gnrc/netif/hdr.h"

gnrc_pktsnip_t *gnrc_netif_hdr_build(uint8_t *src, uint8_t src_len, uint8_t *dst, uint8_t dst_len)
{
    gnrc_pktsnip_t *pkt = gnrc_pktbuf_add(NULL, NULL,
                                          sizeof(gnrc_netif_hdr_t) + src_len + dst_len,
                                          GNRC_NETTYPE_NETIF);

    if (pkt == NULL) {
        return NULL;
    }

    gnrc_netif_hdr_init(pkt->data, src_len, dst_len);

    if (src != NULL && src_len > 0) {
        gnrc_netif_hdr_set_src_addr(pkt->data, src, src_len);
    }

    if (dst != NULL && dst_len > 0) {
        gnrc_netif_hdr_set_dst_addr(pkt->data, dst, dst_len);
    }

    return pkt;
}

int gnrc_netif_get_dstaddr(gnrc_pktsnip_t* pkt, uint8_t** pointer_to_addr)
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

/** @} */

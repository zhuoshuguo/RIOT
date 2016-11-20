/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_mac  A common MAC type for providing key MAC parameters and helper functions
 * @ingroup     net
 * @brief       A common MAC type for providing key MAC parameters and helper functions.
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of MAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_MAC_INTERNAL_H_
#define GNRC_MAC_INTERNAL_H_

#include <stdint.h>
#include <net/gnrc/mac/types.h>
#include <net/ieee802154.h>

#ifdef __cplusplus
extern "C" {
#endif

/* @brief Extract the destination address out of an GNRC_NETTYPE_NETIF pktsnip
 *
 * @param[in]   pkt                 pktsnip from whom to extract
 * @param[out]  pointer_to_addr     pointer to address will be stored here
 *
 * @return                          length of destination address
 */
int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[]);

/* @brief Find the first pktsnip of @p type
 *
 * Will search linearly through the packet buffer @p pkt and yield
 * gnrc_pktsnip_t::data of the first pktsnip match the type @p type.
 *
 * @param[in]   pkt     pktsnip that will be searched
 * @param[in]   type    type to search for
 *
 * @return              pointer to data, NULL is not found
 */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type);

/* @brief Check if packet is for broadcast
 *
 * @param[in]   pkt     packet to check
 *
 * @return              true if the packet is for broadcast, otherwise false
 */
static inline bool _packet_is_broadcast(gnrc_pktsnip_t* pkt)
{
    gnrc_netif_hdr_t* netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    return ((netif_hdr == NULL) ? false :
            (netif_hdr->flags & (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)));
}

/* @brief Check whether the two given addresses match each other (are the same).
 *
 * @param[in]   addr1    the first address given for checking
 * @param[in]   addr2    the second address given for checking
 * @param[in]   add_len  the length of the address
 *
 * @return               true if the two address match each other, otherwise return false.
 */
static inline bool _addr_match(uint8_t * addr1, uint8_t* addr2, uint8_t add_len)
{
    assert(addr1);
    assert(addr2);
    assert(add_len);

    return (memcmp(addr1, addr2, add_len) == 0);
}

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_INTERNAL_H_ */
/** @} */

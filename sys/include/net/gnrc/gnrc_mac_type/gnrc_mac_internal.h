/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Internal functions if LWMAC
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_MAC_INTERNAL_H_
#define GNRC_MAC_INTERNAL_H_

#include <stdint.h>
//#include "periph/rtt.h"
#include "gnrc_mac_types.h"

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

/* @brief Shortcut to get the state of netdev
 *
 * @param[in]   gnrc_mac           gnrc_mac state that stores netdev pointer
 *
 * @return                      state of netdev
 */
netopt_state_t _get_netdev_state(gnrc_mac_t* gnrc_mac);

/* @brief Shortcut to set the state of netdev
 *
 * @param[in]   gnrc_mac           gnrc_mac state that stores netdev pointer
 * @param[in]   devstate        new state for netdev
 */
void _set_netdev_state(gnrc_mac_t* gnrc_mac, netopt_state_t devstate);

/* @brief Check if packet is broadcast
 *
 * @param[in]   pkt             packet to check
 */
static inline bool _packet_is_broadcast(gnrc_pktsnip_t* pkt)
{
    gnrc_netif_hdr_t* netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    return ( (netif_hdr == NULL) ? false :
                              (netif_hdr->flags & (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) );
}


static inline bool _addr_match(l2_addr_t* addr1, l2_addr_t* addr2)
{
    assert(addr1);
    assert(addr2);

    if(addr1->len != addr2->len)
        return false;

    return (memcmp(addr1->addr, addr2->addr, addr1->len) == 0);
}

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_INTERNAL_H_ */
/** @} */

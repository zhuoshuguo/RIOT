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
#include <net/ieee802154.h>
#include <net/gnrc/mac/types.h>

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
int gnrc_mac_get_dstaddr(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[]);

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
void* gnrc_mac_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type);

/* @brief Check if packet is for broadcast
 *
 * @param[in]   pkt     packet to check
 *
 * @return              true if the packet is for broadcast, otherwise false
 */
static inline bool gnrc_mac_chk_pkt_bcast(gnrc_pktsnip_t* pkt)
{
    gnrc_netif_hdr_t* netif_hdr = gnrc_mac_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
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
static inline bool gnrc_mac_addr_match(uint8_t * addr1, uint8_t* addr2, uint8_t add_len)
{
    assert(addr1);
    assert(addr2);
    assert(add_len);

    return (memcmp(addr1, addr2, add_len) == 0);
}

#if GNRC_MAC_TX_QUEUE_SIZE != 0
#if GNRC_MAC_NEIGHBOR_COUNT != 0
/* @brief fetch the address of the tx neighbor according to the neighbor's ID
 *
 * @param[in]     tx        internal state of transmission state machine
 * @param[in]     id        the ID of the neighbor
 *
 * @return                  the address of the neighbor
 */
static inline gnrc_mac_tx_neighbor_t* gnrc_mac_get_neighbor(gnrc_mac_tx_t* tx, unsigned int id)
{
    return &(tx->neighbors[id]);
}
#endif
/* GNRC_MAC_NEIGHBOR_COUNT */

/* @brief queue the packet into the related transmission packet queue.
 *        Note that, in case the neighbor structure is used and the neighbor has Tx-queue,
 *        this function queues the packet to the queue associated with the
 *        pkt's destination neighbor. On the other hand, if neighbor structure is not used,
 *        this function queues the packet to the single priority TX queue stored in
 *        the 'gnrc_mac_tx_t' structure.
 *
 * @param[in,out] tx        gnrc_mac transmission management object
 * @param[in]     priority  the priority of @p pkt
 * @param[in]     pkt       pktsnip that will be queued
 *
 * @return                  return true if queued successfully, otherwise false.
 */
bool gnrc_mac_queue_tx_packet(gnrc_mac_tx_t* tx, uint32_t priority, gnrc_pktsnip_t* pkt);
#endif
/* GNRC_MAC_TX_QUEUE_SIZE */

#if GNRC_MAC_RX_QUEUE_SIZE != 0
/* @brief queue the packet into the reception packet queue.
 *
 * @param[in,out] rx        gnrc_mac reception management object
 * @param[in]     priority  the priority of @p pkt
 * @param[in]     pkt       pktsnip that will be queued
 *
 * @return                  return true if queued successfully, otherwise false.
 */
bool gnrc_mac_queue_rx_packet(gnrc_mac_rx_t* rx, uint32_t priority, gnrc_pktsnip_t* pkt);
#endif
/* GNRC_MAC_RX_QUEUE_SIZE */

#if GNRC_MAC_DISPATCH_BUFFER_SIZE != 0
/* @brief dispatch all the packets in the dispatching buffer to upper layer.
 *
 * @param[in,out]  buffer   the buffer storing dispatching packets
 */
void gnrc_mac_dispatch(gnrc_pktsnip_t* buffer[]);
#endif
/* endif for `#if GNRC_MAC_DISPATCH_BUFFER_SIZE != 0` */

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_INTERNAL_H_ */
/** @} */

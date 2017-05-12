/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_lwmac
 * @brief       Internal functions of LWMAC
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_LWMAC_INTERNAL_H_
#define GNRC_LWMAC_INTERNAL_H_

#include <stdint.h>

#include "periph/rtt.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/mac/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Flag to track if the sender can continue to transmit packet to
 *          the receiver in its TX procedure.
 */
#define GNRC_NETDEV_LWMAC_TX_CONTINUE          (0x0008U)

/**
 * @brief   Flag to track if the sender should quit Tx in current cycle.
 */
#define GNRC_NETDEV_LWMAC_QUIT_TX              (0x0010U)

/**
 * @brief   Flag to track if the device need to reselect a new phase.
 */
#define GNRC_NETDEV_LWMAC_PHASE_BACKOFF        (0x0020U)

/**
 * @brief   Flag to track if the device need to quit listening procedure.
 */
#define GNRC_NETDEV_LWMAC_QUIT_RX              (0x0040U)

/**
 * @brief Type to pass information about parsing.
 */
typedef struct {
    lwmac_hdr_t *header;    /**< Lwmac header of packet */
    l2_addr_t src_addr;     /**< copied source address of packet  */
    l2_addr_t dst_addr;     /**< copied destination address of packet */
} lwmac_packet_info_t;

/**
 * @brief Next RTT event must be at least this far in the future.
 *
 *        When setting an RTT alarm to short in the future it could be possible that
 *        the counter already passed the calculated alarm before it could be set. This
 *        margin will be applied when using `_next_inphase_event()`.
 */
#define LWMAC_RTT_EVENT_MARGIN_TICKS    (RTT_MS_TO_TICKS(2))

/**
 * @brief set the TX-continue flag of the device
 *
 * @param[in] dev          ptr to netdev device
 * @param[in] tx_continue  value for Lwmac tx-continue flag
 *
 */
static inline void gnrc_netdev_lwmac_set_tx_continue(gnrc_netdev_t *dev, bool tx_continue)
{
    if (tx_continue) {
        dev->mac_info |= GNRC_NETDEV_LWMAC_TX_CONTINUE;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_LWMAC_TX_CONTINUE;
    }
}

/**
 * @brief get the TX-continue flag of the device
 *
 * @param[in] dev          ptr to netdev device
 *
 */
static inline bool gnrc_netdev_lwmac_get_tx_continue(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_LWMAC_TX_CONTINUE);
}

/**
 * @brief set the quit-TX flag of the device
 *
 * @param[in] dev          ptr to netdev device
 * @param[in] quit_tx      value for Lwmac quit-TX flag
 *
 */
static inline void gnrc_netdev_lwmac_set_quit_tx(gnrc_netdev_t *dev, bool quit_tx)
{
    if (quit_tx) {
        dev->mac_info |= GNRC_NETDEV_LWMAC_QUIT_TX;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_LWMAC_QUIT_TX;
    }
}

/**
 * @brief get the quit-TX flag of the device
 *
 * @param[in] dev          ptr to netdev device
 *
 */
static inline bool gnrc_netdev_lwmac_get_quit_tx(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_LWMAC_QUIT_TX);
}

/**
 * @brief set the phase-backoff flag of the device
 *
 * @param[in] dev          ptr to netdev device
 * @param[in] backoff      value for Lwmac phase-backoff flag
 *
 */
static inline void gnrc_netdev_lwmac_set_phase_backoff(gnrc_netdev_t *dev, bool backoff)
{
    if (backoff) {
        dev->mac_info |= GNRC_NETDEV_LWMAC_PHASE_BACKOFF;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_LWMAC_PHASE_BACKOFF;
    }
}

/**
 * @brief get the phase-backoff of the device
 *
 * @param[in] dev          ptr to netdev device
 *
 */
static inline bool gnrc_netdev_lwmac_get_phase_backoff(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_LWMAC_PHASE_BACKOFF);
}

/**
 * @brief set the quit-RX flag of the device
 *
 * @param[in] dev          ptr to netdev device
 * @param[in] quit_rx      value for Lwmac quit-Rx flag
 *
 */
static inline void gnrc_netdev_lwmac_set_quit_rx(gnrc_netdev_t *dev, bool quit_rx)
{
    if (quit_rx) {
        dev->mac_info |= GNRC_NETDEV_LWMAC_QUIT_RX;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_LWMAC_QUIT_RX;
    }
}

/**
 * @brief get the quit-RX flag of the device
 *
 * @param[in] dev          ptr to netdev device
 *
 */
static inline bool gnrc_netdev_lwmac_get_quit_rx(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_LWMAC_QUIT_RX);
}

/**
 * @brief Parse an incoming packet and extract important information.
 *
 *        Copies addresses into @p info, but header points inside @p pkt.
 *
 * @param[in]   pkt             packet that will be parsed
 * @param[out]  info            structure that will hold parsed information
 *
 * @return                      0 if correctly parsed
 * @return                      <0 on error
 */
int _parse_packet(gnrc_pktsnip_t *pkt, lwmac_packet_info_t *info);

/**
 * @brief Shortcut to get the state of netdev.
 *
 * @param[in]   gnrc_netdev    gnrc_netdev structure
 *
 * @return                     state of netdev
 */
netopt_state_t _get_netdev_state(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Shortcut to set the state of netdev
 *
 * @param[in]   gnrc_netdev    gnrc_netdev structure
 * @param[in]   devstate       new state for netdev
 */
void _set_netdev_state(gnrc_netdev_t *gnrc_netdev, netopt_state_t devstate);

/**
 * @brief Convert RTT ticks to device phase
 *
 * @param[in]   ticks    RTT ticks
 *
 * @return               device phase
 */
static inline uint32_t _ticks_to_phase(uint32_t ticks)
{
    return (ticks % RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
}

/**
 * @brief Get device's current phase
 *
 * @return               device phase
 */
static inline uint32_t _phase_now(void)
{
    return _ticks_to_phase(rtt_get_counter());
}

/**
 * @brief Calculate how many ticks remaining to the targeted phase in the future
 *
 * @param[in]   phase    device phase
 *
 * @return               RTT ticks
 */
static inline uint32_t _ticks_until_phase(uint32_t phase)
{
    long int tmp = phase - _phase_now();

    if (tmp < 0) {
        /* Phase in next interval */
        tmp += RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US);
    }

    return (uint32_t)tmp;
}

/**
 * @brief Convert device phase to RTT ticks
 *
 * @param[in]   phase    device phase
 *
 * @return               RTT ticks
 */
uint32_t _phase_to_ticks(uint32_t phase);

/**
 * @brief Find the Tx neighbor that has a packet queued and is next for sending
 *
 * @param[in]   gnrc_netdev    gnrc_netdev structure
 *
 * @return                     tx neighbor
 * @return                     NULL, if there is no neighbor for transmission.
 */
gnrc_mac_tx_neighbor_t *_next_tx_neighbor(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Calculate the next event's timing in rtt timer ticks
 *
 * @param[in]   last        gnrc_netdev structure
 * @param[in]   interval    device's wake up interval
 *
 * @return                  RTT ticks
 */
uint32_t _next_inphase_event(uint32_t last, uint32_t interval);

/**
 * @brief Store the received packet to the dispatch buffer and remove possible
 *        duplicate packets.
 *
 * @param[in,out]   buffer      RX dispatch packet buffer
 * @param[in]       pkt         received packet
 *
 * @return                      0 if correctly stored
 * @return                      <0 on error
 */
int _dispatch_defer(gnrc_pktsnip_t * buffer[], gnrc_pktsnip_t * pkt);

/**
 * @brief Dispatch received packet to the upper layer
 *
 * @param[in,out]   buffer      RX dispatch packet buffer
 */
void _dispatch(gnrc_pktsnip_t * buffer[]);

/**
 * @brief Print out the Lwmac header information.
 *
 * @param[in] hdr  lwmac header
 */
void lwmac_print_hdr(lwmac_hdr_t *hdr);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_INTERNAL_H_ */
/** @} */

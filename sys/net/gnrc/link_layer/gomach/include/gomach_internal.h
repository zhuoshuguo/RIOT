/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_gomach
 * @{
 *
 * @file
 * @brief       GoMacH's internal functions.
 * @internal
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GOMACH_INTERNAL_H
#define GOMACH_INTERNAL_H

#include <stdint.h>

#include "periph/rtt.h"
#include "net/gnrc/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Flag to track if the transmission has finished.
 */
#define GNRC_NETDEV_GOMACH_INFO_TX_FINISHED         (0x0008U)

/**
 * @brief Flag to track if a packet has been successfully received.
 */
#define GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED        (0x0010U)

/**
 * @brief Flag to track if need to update GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_ND_UPDATE        (0x0001U)

/**
 * @brief Flag to track if need to quit the current cycle in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_QUIT_CYCLE        (0x0002U)

/**
 * @brief Flag to track if CP period has ended in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_CP_END        (0x0004U)

/**
 * @brief Flag to track if vTDMA has ended in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_VTDMA_END        (0x0008U)

/**
 * @brief Flag to track if the node has received unintended preamble.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_UNINTD_PREAMBLE        (0x0010U)

/**
 * @brief Flag to track if need to quit the current cycle in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLE        (0x0020U)

/**
 * @brief Flag to track if node's duty-cycle has started in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_DUTY_CYCLE_START        (0x0040U)

/**
 * @brief Flag to track if node need to backoff its phase in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_BACKOFF        (0x0080U)

/**
 * @brief Flag to track if node's phase has changed in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_CHANGED        (0x0100U)

/**
 * @brief Flag to track if beacon transmission fail in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_BEACON_FAIL        (0x0200U)

/**
 * @brief Flag to track if node's packet buffer is full in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_BUFFER_FULL        (0x0400U)

/**
 * @brief Flag to track if node has entered a new cycle in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_ENTER_NEW_CYCLE        (0x0800U)

/**
 * @brief Flag to track if node has got preamble-ACK in GoMacH.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLEACK        (0x1000U)

/**
 * @brief Flag to track if node's radio is on public-channel-1.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_ON_PUBCHAN_1        (0x2000U)

/**
 * @brief Flag to track if node has reached maximum preamble interval.
 */
#define GNRC_NETDEV_GOMACH_INTERNAL_INFO_MAX_PREAM_INTERV        (0x4000U)

/**
 * @brief Flag to track if node has turned on its radio.
 */
#define GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON        (0x8000U)

/**
 * @brief Set the TX-finish flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] tx_finish    value for GoMacH TX-finish flag.
 *
 */
static inline void gnrc_gomach_set_tx_finish(gnrc_netdev_t *gnrc_netdev, bool tx_finish)
{
    if (tx_finish) {
        gnrc_netdev->mac_info |= GNRC_NETDEV_GOMACH_INFO_TX_FINISHED;
    }
    else {
        gnrc_netdev->mac_info &= ~GNRC_NETDEV_GOMACH_INFO_TX_FINISHED;
    }
}

/**
 * @brief Get the TX-finish flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if TX has finished.
 * @return                 false if TX hasn't finished yet.
 */
static inline bool gnrc_gomach_get_tx_finish(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->mac_info & GNRC_NETDEV_GOMACH_INFO_TX_FINISHED);
}

/**
 * @brief Set the packet-received flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] received     value for GoMacH packet-received flag.
 *
 */
static inline void gnrc_gomach_set_pkt_received(gnrc_netdev_t *gnrc_netdev, bool received)
{
    if (received) {
        gnrc_netdev->mac_info |= GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED;
    }
    else {
        gnrc_netdev->mac_info &= ~GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED;
    }
}

/**
 * @brief Get the packet-received flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 *
 * @return                 true if radio has successfully received a packet.
 * @return                 false if radio hasn't received a packet yet.
 */
static inline bool gnrc_gomach_get_pkt_received(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->mac_info & GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED);
}

/**
 * @brief Set the quit-cycle flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] quit             value for GoMacH's quit-cycle flag.
 *
 */
static inline void gnrc_gomach_set_quit_cycle(gnrc_netdev_t *gnrc_netdev, bool quit)
{
    if (quit) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_QUIT_CYCLE;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_QUIT_CYCLE;
    }
}

/**
 * @brief Get the quit-cycle flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if need to quit cycle.
 * @return                 false if no need to quit cycle.
 */
static inline bool gnrc_gomach_get_quit_cycle(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_QUIT_CYCLE);
}

/**
 * @brief Set the got-preamble flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] got_preamble     value for GoMacH's got-preamble flag.
 *
 */
static inline void gnrc_gomach_set_got_preamble(gnrc_netdev_t *gnrc_netdev, bool got_preamble)
{
    if (got_preamble) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLE;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLE;
    }
}

/**
 * @brief Get the got-preamble flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if get preamble packet.
 * @return                 false if not get preamble packet yet.
 */
static inline bool gnrc_gomach_get_got_preamble(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLE);
}

/**
 * @brief Set the cp-end flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] cp_end     value for GoMacH's cp-end flag.
 *
 */
static inline void gnrc_gomach_set_cp_end(gnrc_netdev_t *gnrc_netdev, bool cp_end)
{
    if (cp_end) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_CP_END;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_CP_END;
    }
}

/**
 * @brief Get the cp-end flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if cp has ended.
 * @return                 false if cp hasn't ended yet.
 */
static inline bool gnrc_gomach_get_cp_end(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_CP_END);
}

/**
 * @brief Set the vTDMA-end flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] vtdma_end     value for GoMacH's vTDMA-end flag.
 *
 */
static inline void gnrc_gomach_set_vTDMA_end(gnrc_netdev_t *gnrc_netdev, bool vtdma_end)
{
    if (vtdma_end) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_VTDMA_END;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_VTDMA_END;
    }
}

/**
 * @brief Get the vTDMA-end flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if vTDMA has ended.
 * @return                 false if vTDMA hasn't ended yet.
 */
static inline bool gnrc_gomach_get_vTDMA_end(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_VTDMA_END);
}

/**
 * @brief Set the unintended-preamble flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] uintd_preamble   value for GoMacH's unintended-preamble flag.
 *
 */
static inline void gnrc_gomach_set_unintd_preamble(gnrc_netdev_t *gnrc_netdev, bool uintd_preamble)
{
    if (uintd_preamble) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_UNINTD_PREAMBLE;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_UNINTD_PREAMBLE;
    }
}

/**
 * @brief Get the unintended-preamble flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if has received unintended-preamble.
 * @return                 false if hasn't received unintended-preamble yet.
 */
static inline bool gnrc_gomach_get_unintd_preamble(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_UNINTD_PREAMBLE);
}

/**
 * @brief Set the need-update flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] update   value for GoMacH's need-update flag.
 *
 */
static inline void gnrc_gomach_set_update(gnrc_netdev_t *gnrc_netdev, bool update)
{
    if (update) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_ND_UPDATE;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_ND_UPDATE;
    }
}

/**
 * @brief Get the need-update flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if need update GoMacH.
 * @return                 false if no need to update GoMacH.
 */
static inline bool gnrc_gomach_get_update(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_ND_UPDATE);
}

/**
 * @brief Set the duty-cycle-start flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] start   value for GoMacH's duty-cycle-start flag.
 *
 */
static inline void gnrc_gomach_set_duty_cycle_start(gnrc_netdev_t *gnrc_netdev, bool start)
{
    if (start) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_DUTY_CYCLE_START;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_DUTY_CYCLE_START;
    }
}

/**
 * @brief Get the duty-cycle-start flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if duty-cycle has started.
 * @return                 false if duty-cycle hasn't started yet.
 */
static inline bool gnrc_gomach_get_duty_cycle_start(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_DUTY_CYCLE_START);
}

/**
 * @brief Set the phase-backoff flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] backoff   value for GoMacH's phase-backoff flag.
 *
 */
static inline void gnrc_gomach_set_phase_backoff(gnrc_netdev_t *gnrc_netdev, bool backoff)
{
    if (backoff) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_BACKOFF;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_BACKOFF;
    }
}

/**
 * @brief Get the phase-backoff flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if need to run phase backoff.
 * @return                 false if no need to run phase backoff.
 */
static inline bool gnrc_gomach_get_phase_backoff(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_BACKOFF);
}

/**
 * @brief Set the phase-changed flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] change   value for GoMacH's phase-changed flag.
 *
 */
static inline void gnrc_gomach_set_phase_changed(gnrc_netdev_t *gnrc_netdev, bool change)
{
    if (change) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_CHANGED;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_CHANGED;
    }
}

/**
 * @brief Get the phase-changed flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node's phase has changed.
 * @return                 false if node's phase hasn't changed yet.
 */
static inline bool gnrc_gomach_get_phase_changed(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_PHASE_CHANGED);
}

/**
 * @brief Set the beacon-fail flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] fail   value for GoMacH's beacon-fail flag.
 *
 */
static inline void gnrc_gomach_set_beacon_fail(gnrc_netdev_t *gnrc_netdev, bool fail)
{
    if (fail) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_BEACON_FAIL;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_BEACON_FAIL;
    }
}

/**
 * @brief Get the beacon-fail flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if send beacon fail.
 * @return                 false upon beacon transmission success.
 */
static inline bool gnrc_gomach_get_beacon_fail(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_BEACON_FAIL);
}

/**
 * @brief Set the buffer-full flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] full   value for GoMacH's buffer-full flag.
 *
 */
static inline void gnrc_gomach_set_buffer_full(gnrc_netdev_t *gnrc_netdev, bool full)
{
    if (full) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_BUFFER_FULL;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_BUFFER_FULL;
    }
}

/**
 * @brief Get the buffer-full flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node's packet buffer is full.
 * @return                 false if node's packet buffer is not full.
 */
static inline bool gnrc_gomach_get_buffer_full(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_BUFFER_FULL);
}

/**
 * @brief Set the enter-new-cycle flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] enter   value for GoMacH's enter-new-cycle flag.
 *
 */
static inline void gnrc_gomach_set_enter_new_cycle(gnrc_netdev_t *gnrc_netdev, bool enter)
{
    if (enter) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_ENTER_NEW_CYCLE;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_ENTER_NEW_CYCLE;
    }
}

/**
 * @brief Get the enter-new-cycle flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node has entered a new cycle.
 * @return                 false if node hasn't entered a new cycle yet.
 */
static inline bool gnrc_gomach_get_enter_new_cycle(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_ENTER_NEW_CYCLE);
}

/**
 * @brief Set the got-preamble-ACK flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] got   value for GoMacH's got-preamble-ACK flag.
 *
 */
static inline void gnrc_gomach_set_got_preamble_ack(gnrc_netdev_t *gnrc_netdev, bool got)
{
    if (got) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLEACK;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLEACK;
    }
}

/**
 * @brief Get the got-preamble-ACK flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node has got preamble-ACK.
 * @return                 false if node hasn't got preamble-ACK yet.
 */
static inline bool gnrc_gomach_get_got_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_GOT_PREAMBLEACK);
}

/**
 * @brief Set the on-public-channel-1 flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] on_pubchan_1   value for GoMacH's on-public-channel-1 flag.
 *
 */
static inline void gnrc_gomach_set_on_pubchan_1(gnrc_netdev_t *gnrc_netdev, bool on_pubchan_1)
{
    if (on_pubchan_1) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_ON_PUBCHAN_1;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_ON_PUBCHAN_1;
    }
}

/**
 * @brief Get the on-public-channel-1 flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node is on public channel 1.
 * @return                 false if node is not on public channel 1.
 */
static inline bool gnrc_gomach_get_on_pubchan_1(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_ON_PUBCHAN_1);
}

/**
 * @brief Set the max-preamble-interval flag of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] max   value for GoMacH's max-preamble-interval flag.
 *
 */
static inline void gnrc_gomach_set_max_pream_interv(gnrc_netdev_t *gnrc_netdev, bool max)
{
    if (max) {
        gnrc_netdev->gomach.gomach_info |= GNRC_NETDEV_GOMACH_INTERNAL_INFO_MAX_PREAM_INTERV;
    }
    else {
        gnrc_netdev->gomach.gomach_info &= ~GNRC_NETDEV_GOMACH_INTERNAL_INFO_MAX_PREAM_INTERV;
    }
}

/**
 * @brief Get the max-preamble-interval flag of the device.
 *
 * @param[in] gnrc_netdev  ptr to netdev device
 *
 * @return                 true if node has reached maximum preamble interval.
 * @return                 false if node hasn't reached maximum preamble interval yet.
 */
static inline bool gnrc_gomach_get_max_pream_interv(gnrc_netdev_t *gnrc_netdev)
{
    return (gnrc_netdev->gomach.gomach_info & GNRC_NETDEV_GOMACH_INTERNAL_INFO_MAX_PREAM_INTERV);
}

/**
 * @brief Get device's current phase.
 *
 * @param[in] gnrc_netdev   ptr to netdev device.
 *
 * @return                  device's current phase.
 */
uint32_t gnrc_gomach_phase_now(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Calculate how many ticks remaining to the targeted phase in the future.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 * @param[in] phase        device phase.
 *
 * @return                 RTT ticks remaining to the targeted phase.
 */
static inline uint32_t gnrc_gomach_ticks_until_phase(gnrc_netdev_t *gnrc_netdev, uint32_t phase)
{
    assert(gnrc_netdev != NULL);

    long int tmp = phase - gnrc_gomach_phase_now(gnrc_netdev);

    if (tmp < 0) {
        tmp += RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);
    }

    return (uint32_t)tmp;
}

/**
 * @brief Turn on (wake up) the radio of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
static inline void gnrc_gomach_set_netdev_state(gnrc_netdev_t *gnrc_netdev, netopt_state_t devstate)
{
    assert(gnrc_netdev != NULL);

    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_STATE,
                                  &devstate,
                                  sizeof(devstate));
    if (devstate == NETOPT_STATE_IDLE) {
    	xtimer_usleep(1000);
    }

#if (GNRC_GOMACH_ENABLE_DUTYCYLE_RECORD == 1)
    if (devstate == NETOPT_STATE_IDLE) {
        if (!(gnrc_netdev->gomach.gomach_info & GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON)) {
        	gnrc_netdev->gomach.last_radio_on_time_ticks = xtimer_now_usec64();
        	gnrc_netdev->gomach.gomach_info |= GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON;
        }
        return;
    }
    else if ((devstate == NETOPT_STATE_SLEEP) &&
             (gnrc_netdev->gomach.gomach_info & GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON)) {
    	gnrc_netdev->gomach.radio_off_time_ticks = xtimer_now_usec64();
    	gnrc_netdev->gomach.awake_duration_sum_ticks +=
                    (gnrc_netdev->gomach.radio_off_time_ticks -
                     gnrc_netdev->gomach.last_radio_on_time_ticks);

    	gnrc_netdev->gomach.gomach_info &= ~GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON;
    }
#endif
}

/**
 * @brief Set the auto-ACK parameter of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] autoack      value for the auto-ACK parameter.
 *
 */
static inline void gnrc_gomach_set_autoack(gnrc_netdev_t *gnrc_netdev, netopt_enable_t autoack)
{
    assert(gnrc_netdev != NULL);

    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_AUTOACK,
                                  &autoack,
                                  sizeof(autoack));
}

/**
 * @brief Set the ACK-require parameter of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to gnrc_netdev device.
 * @param[in] ack_req      value for the ACK-require parameter.
 *
 */
static inline void gnrc_gomach_set_ack_req(gnrc_netdev_t *gnrc_netdev, netopt_enable_t ack_req)
{
    assert(gnrc_netdev != NULL);

    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_ACK_REQ,
                                  &ack_req,
                                  sizeof(ack_req));
}

/**
 * @brief Shortcut to get the state of netdev.
 *
 * @param[in] gnrc_netdev  gnrc_netdev structure.
 *
 * @return                 state of netdev upon success.
 * @return                 -ENOSYS, upon failure.
 */
static inline netopt_state_t gnrc_gomach_get_netdev_state(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    netopt_state_t state;

    if (0 < gnrc_netdev->dev->driver->get(gnrc_netdev->dev,
                                          NETOPT_STATE,
                                          &state,
                                          sizeof(state))) {
        return state;
    }
    return -ENOSYS;
}

/**
 * @brief Turn the radio to a specific channel.
 *
 * @param[in,out] gnrc_netdev  ptr to gnrc_netdev device.
 * @param[in] channel_num  targeted channel number to turn to.
 *
 */
static inline void gnrc_gomach_turn_channel(gnrc_netdev_t *gnrc_netdev, uint16_t channel_num)
{
    assert(gnrc_netdev != NULL);

    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_CHANNEL,
                                  &channel_num,
                                  sizeof(channel_num));
    xtimer_usleep(1000);
}

/**
 * @brief Check if the received packet is a duplicate packet.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 * @param[in] pa_info      ptr to received packet's parsed information.
 *
 * @return                 true if the received packet is a duplicate packet.
 * @return                 false if the received packet is not a duplicate packet.
 */
bool gnrc_gomach_check_duplicate(gnrc_netdev_t *gnrc_netdev, gnrc_gomach_packet_info_t *pa_info);

/**
 * @brief Send a pktsnip using GoMacH.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 * @param[in] pkt          ptr to the packet for sending.
 * @param[in] csma_enable  value of csma-enable parameter.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_send(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable);

/**
 * @brief Reply a preamble-ACK packet using GoMacH.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 * @param[in] info         ptr to the info of the preamble packet.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_gomach_packet_info_t *info);

/**
 * @brief Broadcast a beacon packet in GoMacH.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_send_beacon(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Store the received packet to the dispatch buffer.
 *
 * @param[in,out]   buffer      RX dispatch packet buffer
 * @param[in]       pkt         received packet
 *
 * @return                      0 if correctly stored
 * @return                      <0 on error
 */
int gnrc_gomach_dispatch_defer(gnrc_pktsnip_t * buffer[], gnrc_pktsnip_t * pkt);

/**
 * @brief Update the queue-length indicator of the packet sender.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] pkt          received packet
 * @param[in] info         ptr to the info of the received packet.
 *
 */
void gnrc_gomach_indicator_update(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt,
                                  gnrc_gomach_packet_info_t *pa_info);

/**
 * @brief Process packets received during the CP (wake-up) period of GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev);


int gnrc_gomach_bcast_rcv_packet_process(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Choose a sub-channel for a device running GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_init_choose_subchannel(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Broadcast the chosen sub-channel sequence to the device's neighbors.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_bcast_subchann_seq(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma);

/**
 * @brief Send a preamble packet to the targeted neighbor.
 *
 * @param[in] gnrc_netdev  ptr to netdev device.
 * @param[in] csma_enable  value of csma-enable parameter.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_send_preamble(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable);

int gnrc_gomach_send_RI_beacon(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable);

/**
 * @brief Process the received preamble-ACK packet to get phase-locked with the sender.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] pkt          ptr to the received preamble-ACK.
 *
 */
void gnrc_gomach_process_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt);

/**
 * @brief Process the received packets to when waiting for the preamble-ACK packet.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_process_pkt_in_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Send a data packet to the targeted neighbor.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in] csma_enable  value of csma-enable parameter.
 *
 * @return                 >0 upon sending success.
 * @return                 0< upon sending failure.
 */
int gnrc_gomach_send_data(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable);

/**
 * @brief Find a neighbor that is next to send packet to.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 * @return                 true, if found next TX neighbor.
 * @return                 false, if not found next TX neighbor.
 *
 */
bool gnrc_gomach_find_next_tx_neighbor(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Process the received beacon packet.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 * @param[in]     pkt          ptr to the received beacon.
 *
 */
void gnrc_gomach_beacon_process(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt);

/**
 * @brief Process the received packets when waiting for the beacon during t2k procedure in
 *        GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_packet_process_in_wait_beacon(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Process the received packets in the vTDMA period in GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_packet_process_in_vtdma(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Update the TX neighbors' phases in GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_update_neighbor_phase(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief Update the TX neighbors' public channel phase in GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_update_neighbor_pubchan(gnrc_netdev_t *gnrc_netdev);

#ifdef __cplusplus
}
#endif

#endif /* GOMACH_INTERNAL_H */
/** @} */

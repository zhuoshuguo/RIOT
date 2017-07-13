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
#include "net/gnrc/gomach/gomach_types.h"

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
        tmp += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
    }

    return (uint32_t)tmp;
}

/**
 * @brief Turn on (wake up) the radio of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
static inline void gnrc_gomach_turn_on_radio(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    netopt_state_t devstate = NETOPT_STATE_IDLE;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_STATE,
                                  &devstate,
                                  sizeof(devstate));
}

/**
 * @brief Turn off the radio of the device.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
static inline void gnrc_gomach_turn_off_radio(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    netopt_state_t devstate = NETOPT_STATE_SLEEP;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                  NETOPT_STATE,
                                  &devstate,
                                  sizeof(devstate));
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
 * @return                 -1, upon failure.
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
    return -1;
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
}

/**
 * @brief Turn the radio to the listen state.
 *
 * @param[in,out] gnrc_netdev  ptr to gnrc_netdev device.
 *
 */
static inline void gnrc_gomach_turn_to_listen_mode(gnrc_netdev_t *gnrc_netdev){
    gnrc_gomach_turn_on_radio(gnrc_netdev);
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
bool gnrc_gomach_check_duplicate(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *pa_info);

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
int gnrc_gomach_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *info);

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
                                  iqueuemac_packet_info_t *pa_info);

/**
 * @brief Process packets received during the CP (wake-up) period of GoMacH.
 *
 * @param[in,out] gnrc_netdev  ptr to netdev device.
 *
 */
void gnrc_gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev);

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

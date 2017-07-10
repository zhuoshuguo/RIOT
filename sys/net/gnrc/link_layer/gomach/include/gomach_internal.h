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
 * @brief   Flag to track if transmission has finished.
 */
#define GNRC_NETDEV_GOMACH_INFO_TX_FINISHED         (0x0008U)

/**
 * @brief   Flag to track if a packet has been successfully received.
 */
#define GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED        (0x0010U)

static inline void gnrc_netdev_gomach_set_tx_finish(gnrc_netdev_t *dev, bool tx_finish)
{
    if (tx_finish) {
        dev->mac_info |= GNRC_NETDEV_GOMACH_INFO_TX_FINISHED;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_GOMACH_INFO_TX_FINISHED;
    }
}

static inline bool gnrc_netdev_gomach_get_tx_finish(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_GOMACH_INFO_TX_FINISHED);
}

static inline void gnrc_netdev_gomach_set_pkt_received(gnrc_netdev_t *dev, bool received)
{
    if (received) {
        dev->mac_info |= GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED;
    }
    else {
        dev->mac_info &= ~GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED;
    }
}

static inline bool gnrc_netdev_gomach_get_pkt_received(gnrc_netdev_t *dev)
{
    return (dev->mac_info & GNRC_NETDEV_GOMACH_INFO_PKT_RECEIVED);
}

/* @brief Parse an incoming packet and extract important information
 *
 * Copies addresses into @p info, but header points inside @p pkt.
 *
 * @param[in]   pkt             packet that will be parsed
 * @param[out]  info            structure that will hold parsed information
 *
 * @return                      0 if correctly parsed
 * @return                      <0 on error
 */
int _parse_packet(gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *info);

/* RTT phase calculation */
static inline uint32_t _ticks_to_phase(uint32_t ticks)
{
	return (ticks % RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US));
}

uint32_t _phase_now(gnrc_netdev_t *gnrc_netdev);

static inline uint32_t _ticks_until_phase(gnrc_netdev_t *gnrc_netdev, uint32_t phase)
{
    long int tmp = phase - _phase_now(gnrc_netdev);

    if (tmp < 0) {
        tmp += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
    }

    return (uint32_t)tmp;
}

uint32_t _next_inphase_event(uint32_t last, uint32_t interval);

void gomach_turn_on_radio(gnrc_netdev_t *gnrc_netdev);
void gomach_turn_off_radio(gnrc_netdev_t *gnrc_netdev);
void gomach_set_autoack(gnrc_netdev_t *gnrc_netdev, netopt_enable_t autoack);
void gomach_set_ack_req(gnrc_netdev_t *gnrc_netdev, netopt_enable_t ack_req);
netopt_state_t _get_netdev_state(gnrc_netdev_t *gnrc_netdev);
//void iqueuemac_set_promiscuousmode(iqueuemac_t* iqueuemac, netopt_enable_t enable);
void gomach_turn_channel(gnrc_netdev_t *gnrc_netdev, uint16_t channel_num);

static inline void gomach_set_raddio_to_listen_mode(gnrc_netdev_t *gnrc_netdev){
    gomach_turn_on_radio(gnrc_netdev);
}

bool iqueuemac_check_duplicate(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *pa_info);
int gomach_send(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable);
int iqueue_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *info);
int gomach_send_beacon(gnrc_netdev_t *gnrc_netdev);
int iqueue_push_packet_to_dispatch_queue(gnrc_pktsnip_t * buffer[], gnrc_pktsnip_t * pkt, iqueuemac_packet_info_t * pa_info);
void iqueuemac_router_queue_indicator_update(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *pa_info);
void gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev);
void iqueuemac_packet_process_in_init(gnrc_netdev_t *gnrc_netdev);
void iqueuemac_init_choose_subchannel(gnrc_netdev_t *gnrc_netdev);
void gomach_bcast_subchann_seq(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma);
int gomach_send_preamble(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma);
void iqueuemac_device_process_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *pa_info);
void iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev);
int gomach_send_data_packet(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable);
bool gomach_find_next_tx_neighbor(gnrc_netdev_t *gnrc_netdev);
//bool iqueuemac_check_has_pending_packet(packet_queue_t* q);
void iqueuemac_beacon_process(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt);
void gomach_wait_beacon_packet_process(gnrc_netdev_t *gnrc_netdev);
void iqueuemac_router_vtdma_receive_packet_process(gnrc_netdev_t *gnrc_netdev);
void gomach_figure_neighbors_new_phase(gnrc_netdev_t *gnrc_netdev);
void _dispatch(gnrc_pktsnip_t **buffer);
void update_neighbor_pubchan(gnrc_netdev_t *gnrc_netdev);
void iqueuemac_broadcast_receive_packet_process(gnrc_netdev_t *gnrc_netdev);

#ifdef __cplusplus
}
#endif

#endif /* GOMACH_INTERNAL_H */
/** @} */

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


#define GNRC_NETDEV_GOMACH_INFO_TX_FINISHED         (0x0008U)

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

/* @brief   Type to pass information about parsing */
typedef struct {
    iqueuemac_hdr_t *header;    /**< iqueuemac header of packet */
    l2_addr_t src_addr;         /**< copied source address of packet  */
    l2_addr_t dst_addr;         /**< copied destination address of packet */
    uint8_t seq;                /**< seq of the received packet */
} iqueuemac_packet_info_t;

/* @brief   Next RTT event must be at least this far in the future
 *
 * When setting an RTT alarm to short in the future it could be possible that
 * the counter already passed the calculated alarm before it could be set. This
 * margin will be applied when using `_next_inphase_event()`.
 */
//#define iqueuemac_RTT_EVENT_MARGIN_TICKS    ( RTT_MS_TO_TICKS(2) )

/* @brief Extract the destination address out of an GNRC_NETTYPE_NETIF pktsnip
 *
 * @param[in]   pkt                 pktsnip from whom to extract
 * @param[out]  pointer_to_addr     pointer to address will be stored here
 *
 * @return                          length of destination address
 */
int _get_dest_address(gnrc_pktsnip_t * pkt, uint8_t * pointer_to_addr[]);

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
void *_gnrc_pktbuf_find(gnrc_pktsnip_t *pkt, gnrc_nettype_t type);

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
//int _parse_packet(gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* info);

/* RTT phase calculation */
uint32_t _ticks_to_phase(uint32_t ticks);
//uint32_t _phase_to_ticks(uint32_t phase);
uint32_t _phase_now(gnrc_netdev_t *gnrc_netdev);
uint32_t _ticks_until_phase(gnrc_netdev_t *gnrc_netdev, uint32_t phase); //uint32_t _ticks_until_phase(uint32_t phase);


static inline gnrc_mac_tx_neighbor_t *_get_neighbour(gnrc_netdev_t *gnrc_netdev, unsigned int id)
{
    return &(gnrc_netdev->tx.neighbors[id]);
}

uint32_t _next_inphase_event(uint32_t last, uint32_t interval);

//int _dispatch_defer(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt);

//void _dispatch(gnrc_pktsnip_t* buffer[]);

static inline bool _addr_match(l2_addr_t *addr1, l2_addr_t *addr2)
{
    assert(addr1);
    assert(addr2);

    if (addr1->len != addr2->len) {
        return false;
    }

    return (memcmp(addr1->addr, addr2->addr, addr1->len) == 0);
}

void gomach_turn_on_radio(gnrc_netdev_t *gnrc_netdev);
void gomach_turn_off_radio(gnrc_netdev_t *gnrc_netdev);
void gomach_set_autoack(gnrc_netdev_t *gnrc_netdev, netopt_enable_t autoack);
void gomach_set_ack_req(gnrc_netdev_t *gnrc_netdev, netopt_enable_t ack_req);
netopt_state_t _get_netdev_state(gnrc_netdev_t *gnrc_netdev);
//void iqueuemac_set_promiscuousmode(iqueuemac_t* iqueuemac, netopt_enable_t enable);
void gomach_turn_channel(gnrc_netdev_t *gnrc_netdev, uint16_t channel_num);
void iqueuemac_set_raddio_to_listen_mode(gnrc_netdev_t *gnrc_netdev);

bool iqueuemac_check_duplicate(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *pa_info);
int gomach_send(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable);
int iqueue_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *info);
int gomach_send_beacon(gnrc_netdev_t *gnrc_netdev);
int _parse_packet(gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *info);
int iqueue_push_packet_to_dispatch_queue(gnrc_pktsnip_t * buffer[], gnrc_pktsnip_t * pkt, iqueuemac_packet_info_t * pa_info);
void iqueuemac_router_queue_indicator_update(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *pa_info);
void gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev);
void iqueuemac_update_subchannel_occu_flags(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *pa_info);
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

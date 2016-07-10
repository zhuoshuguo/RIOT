/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_iqueue_mac traffic adaptive MAC
 * @ingroup     net
 * @brief       Internal functions if iqueue_mac
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of iqueue_mac protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_IQUEUEMAC_INTERNAL_H_
#define GNRC_IQUEUEMAC_INTERNAL_H_

#include <stdint.h>
#include "periph/rtt.h"
#include "iqueuemac_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* @brief   Type to pass information about parsing */
typedef struct {
	iqueuemac_hdr_t* header;    /**< iqueuemac header of packet */
    l2_addr_t  src_addr;    /**< copied source address of packet  */
    l2_addr_t  dst_addr;    /**< copied destination address of packet */
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


/* @brief Check if packet is broadcast
 *
 * @param[in]   pkt             packet to check
*/

static inline bool _packet_is_broadcast(gnrc_pktsnip_t* pkt)
{
    gnrc_netif_hdr_t* netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    return ( (netif_hdr == NULL) ? false :
                              (netif_hdr->flags & GNRC_NETIF_HDR_FLAGS_BROADCAST) );
}

/* TX queue handling */
int _find_neighbour(iqueuemac_t* iqueuemac, uint8_t* dst_addr, int addr_len);
int _free_neighbour(iqueuemac_t* iqueuemac);
int _alloc_neighbour(iqueuemac_t* iqueuemac);
void _init_neighbour(iqueuemac_tx_neighbour_t* neighbour, uint8_t* addr, int len);

/* RTT phase calculation */
uint32_t _ticks_to_phase(uint32_t ticks);
//uint32_t _phase_to_ticks(uint32_t phase);
uint32_t _phase_now(iqueuemac_t* iqueuemac);
uint32_t _ticks_until_phase(iqueuemac_t* iqueuemac, uint32_t phase); //uint32_t _ticks_until_phase(uint32_t phase);


static inline iqueuemac_tx_neighbour_t* _get_neighbour(iqueuemac_t* iqueuemac, unsigned int id)
{
    return &(iqueuemac->tx.neighbours[id]);
}

bool _queue_tx_packet(iqueuemac_t* iqueuemac,  gnrc_pktsnip_t* pkt);
uint32_t _next_inphase_event(uint32_t last, uint32_t interval);

//int _dispatch_defer(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt);

//void _dispatch(gnrc_pktsnip_t* buffer[]);

static inline bool _addr_match(l2_addr_t* addr1, l2_addr_t* addr2)
{
    assert(addr1);
    assert(addr2);

    if(addr1->len != addr2->len)
        return false;

    return (memcmp(addr1->addr, addr2->addr, addr1->len) == 0);
}

void iqueuemac_trun_on_radio(iqueuemac_t* iqueuemac);
void iqueuemac_trun_off_radio(iqueuemac_t* iqueuemac);
void iqueuemac_turn_radio_channel(iqueuemac_t* iqueuemac, uint16_t channel_num);
void iqueuemac_set_raddio_to_listen_mode(iqueuemac_t* iqueuemac);

int iqueuemac_send(iqueuemac_t* iqueuemac, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable);
void iqueue_send_preamble_ack(iqueuemac_t* iqueuemac, iqueuemac_packet_info_t* info);
int iqueuemac_assemble_and_send_beacon(iqueuemac_t* iqueuemac);
int _parse_packet(gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* info);
int iqueue_push_packet_to_dispatch_queue(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info, iqueuemac_t* iqueuemac);
void iqueuemac_router_queue_indicator_update(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info);
void iqueue_router_cp_receive_packet_process(iqueuemac_t* iqueuemac);
void iqueuemac_update_subchannel_occu_flags(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info);
void iqueuemac_packet_process_in_init(iqueuemac_t* iqueuemac);
void iqueuemac_init_choose_subchannel(iqueuemac_t* iqueuemac);
void iqueuemac_send_announce(iqueuemac_t* iqueuemac, netopt_enable_t use_csma);
void iqueue_mac_send_preamble(iqueuemac_t* iqueuemac, netopt_enable_t use_csma);
void iqueuemac_add_in_cluster_neighbor(iqueuemac_t* iqueuemac, l2_addr_t* addr);
void iqueuemac_remove_in_cluster_neighbor(iqueuemac_t* iqueuemac, l2_addr_t* addr);
void iqueuemac_device_process_preamble_ack(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info);
void iqueuemac_packet_process_in_wait_preamble_ack(iqueuemac_t* iqueuemac);
void iqueuemac_send_data_packet(iqueuemac_t* iqueuemac, netopt_enable_t csma_enable);
bool iqueue_mac_find_next_tx_neighbor(iqueuemac_t* iqueuemac);
//bool iqueuemac_check_has_pending_packet(packet_queue_t* q);
void iqueuemac_beacon_process(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt);
void iqueuemac_wait_beacon_packet_process(iqueuemac_t* iqueuemac);
void iqueue_node_cp_receive_packet_process(iqueuemac_t* iqueuemac);
void iqueuemac_router_vtdma_receive_packet_process(iqueuemac_t* iqueuemac);
void _dispatch(gnrc_pktsnip_t* buffer[]);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_INTERNAL_H_ */
/** @} */

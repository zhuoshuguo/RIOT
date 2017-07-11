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
 * @brief       Implementation of GoMacH's internal functions.
 *
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>

#include "periph/rtt.h"
#include "random.h"
#include "net/gnrc.h"
#include "net/gnrc/mac/types.h"
#include "net/gnrc/mac/mac.h"
#include "net/gnrc/gomach/hdr.h"
#include "net/gnrc/gomach/gomach.h"
#include "net/gnrc/gomach/gomach_types.h"
#include "include/gomach_internal.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef LOG_LEVEL
/**
 * @brief Default log level define
 */
#define LOG_LEVEL LOG_WARNING
#endif

#include "log.h"

int _parse_packet(gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *info)
{
    assert(info != NULL);
    assert(pkt != NULL);

    gnrc_netif_hdr_t *netif_hdr;
    gnrc_pktsnip_t *iqueuemac_snip;
    iqueuemac_hdr_t *iqueuemac_hdr;

    netif_hdr = (gnrc_netif_hdr_t *) gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_hdr == NULL) {
        return -1;
    }
    netif_hdr = ((gnrc_pktsnip_t *)netif_hdr)->data;

    if (netif_hdr->dst_l2addr_len > sizeof(info->dst_addr)) {
        return -3;
    }

    if (netif_hdr->src_l2addr_len > sizeof(info->src_addr)) {
        return -4;
    }

    /* Dissect GoMacH header, Every frame has header as first member */
    iqueuemac_hdr = (iqueuemac_hdr_t *) pkt->data;

    switch (iqueuemac_hdr->type) {
        case FRAMETYPE_BEACON: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_beacon_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }
        case FRAMETYPE_PREAMBLE: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_preamble_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }
        case FRAMETYPE_PREAMBLE_ACK: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_preamble_ack_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }
        case FRAMETYPE_IQUEUE_DATA: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_data_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }
        case FRAMETYPE_ANNOUNCE: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_announce_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }
        case FRAMETYPE_BROADCAST: {
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_broadcast_t),
                                              GNRC_NETTYPE_GOMACH);
            break;
        }

        default:{
            return -2;
        }
    }

    /* Memory location may have changed while marking. */
    iqueuemac_hdr = iqueuemac_snip->data;

    /* Get the destination address. */
    switch (iqueuemac_hdr->type) {
        case FRAMETYPE_PREAMBLE: {
            info->dst_addr = ((iqueuemac_frame_preamble_t *)iqueuemac_hdr)->dst_addr;
            break;
        }
        case FRAMETYPE_PREAMBLE_ACK: {
            info->dst_addr = ((iqueuemac_frame_preamble_ack_t *)iqueuemac_hdr)->dst_addr;
            break;
        }
        case FRAMETYPE_IQUEUE_DATA: {
            if (netif_hdr->dst_l2addr_len) {
                info->dst_addr.len = netif_hdr->dst_l2addr_len;
                memcpy(info->dst_addr.addr,
                       gnrc_netif_hdr_get_dst_addr(netif_hdr),
                       netif_hdr->dst_l2addr_len);
            }
            break;
        }
        default: {
            break;
        }
    }

    /* Get the source address. */
    if (netif_hdr->src_l2addr_len) {
        info->src_addr.len = netif_hdr->src_l2addr_len;
        memcpy(info->src_addr.addr,
               gnrc_netif_hdr_get_src_addr(netif_hdr),
               netif_hdr->src_l2addr_len);
    }

    info->header = iqueuemac_hdr;
    info->seq = netif_hdr->seq;
    return 0;
}

uint32_t gnrc_gomach_phase_now(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    uint32_t phase_now;

    phase_now = rtt_get_counter();

    /* in case that rtt overflows */
    if (phase_now < gnrc_netdev->gomach.last_wakeup) {
        uint32_t gap_to_full;
        gap_to_full = GNRC_GOMACH_PHASE_MAX - gnrc_netdev->gomach.last_wakeup;
        phase_now += gap_to_full;
    }
    else {
        phase_now = phase_now - gnrc_netdev->gomach.last_wakeup;
    }

    return phase_now;
}

int gnrc_gomach_send(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable)
{
    assert(gnrc_netdev != NULL);
    assert(pkt != NULL);

    /* Enable/disable CSMA according to the input. */
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_CSMA, &csma_enable,
                                  sizeof(netopt_enable_t));

    gnrc_gomach_set_tx_finish(gnrc_netdev, false);
    gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_UNDEF);
    return gnrc_netdev->send(gnrc_netdev, pkt);
}


int gnrc_gomach_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *info)
{
    assert(gnrc_netdev != NULL);
    assert(info != NULL);

    gnrc_pktsnip_t *pkt_iqmac;
    gnrc_pktsnip_t *pkt;
    gnrc_netif_hdr_t *nethdr_preamble_ack;

    /* Start assemble the preamble-ACK packet according to preamble packet info. */
    iqueuemac_frame_preamble_ack_t iqueuemac_preamble_ack_hdr;

    iqueuemac_preamble_ack_hdr.header.type = FRAMETYPE_PREAMBLE_ACK;
    iqueuemac_preamble_ack_hdr.dst_addr = info->src_addr;
    /* Tell the preamble sender the device's (preamble-ACK sender) current phase.
     * This is to allow the preamble sender to deduce the exact phase of the receiver. */
    iqueuemac_preamble_ack_hdr.phase_in_ticks = gnrc_gomach_phase_now(gnrc_netdev);

    pkt = gnrc_pktbuf_add(NULL, &iqueuemac_preamble_ack_hdr, sizeof(iqueuemac_preamble_ack_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_preamble_ack().\n");
        return -ENOBUFS;
    }
    pkt_iqmac = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: netif_hdr add failed in gnrc_gomach_send_preamble_ack().\n");
        gnrc_pktbuf_release(pkt_iqmac);
        return -ENOBUFS;
    }
    pkt_iqmac = pkt;

    /* We wouldn't get here if add the NETIF header had failed, so no sanity checks needed */
    nethdr_preamble_ack = (gnrc_netif_hdr_t *)
                          (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF))->data;

    /* Construct NETIF header and insert address for preamble-ACK packet */
    gnrc_netif_hdr_init(nethdr_preamble_ack, 0, 0);

    /* Send preamble-ACK as broadcast*/
    nethdr_preamble_ack->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    int res = gnrc_gomach_send(gnrc_netdev, pkt, NETOPT_DISABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH]: send preamble-ack failed in gnrc_gomach_send_preamble_ack().\n");
        gnrc_pktbuf_release(pkt_iqmac);
    }
    return res;
}

int gnrc_gomach_send_beacon(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    int i;
    int j = 0;
    uint8_t total_tdma_node_num = 0;
    uint8_t total_tdma_slot_num = 0;

    /* First check how many slots needed to be allocated. */
    gnrc_netdev->rx.router_vtdma_mana.total_slots_num = 0;

    for (i = 0; i < IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT; i++) {
        if (gnrc_netdev->rx.rx_register_list[i].queue_indicator > 0) {
            total_tdma_slot_num = gnrc_netdev->rx.rx_register_list[i].queue_indicator;
            break;
        }
    }

    if (total_tdma_slot_num == 0) {
        return 0;
    }

    total_tdma_slot_num = 0;

    gnrc_pktsnip_t *pkt;
    gnrc_pktsnip_t *pkt_iqmac;
    gnrc_netif_hdr_t *nethdr_beacon;

    /* Start assemble the beacon packet */
    iqueuemac_frame_beacon_t iqueuemac_hdr;
    iqueuemac_hdr.header.type = FRAMETYPE_BEACON;
    iqueuemac_hdr.sub_channel_seq = gnrc_netdev->gomach.sub_channel_num;

    /* Start generating the slots list and the related ID list for guiding
     * the following vTMDA procedure (slotted transmission). */
    gnrc_netdev->rx.router_vtdma_mana.total_slots_num = 0;

    l2_id_t id_list[IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT];
    uint8_t slots_list[IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT];

    for (i = 0; i < IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT; i++) {
        if (gnrc_netdev->rx.rx_register_list[i].queue_indicator > 0) {
            /* Record the device's (that will be allocated slots) address to the ID list. */
            memcpy(id_list[j].addr,
                   gnrc_netdev->rx.rx_register_list[i].node_addr.addr,
                   gnrc_netdev->rx.rx_register_list[i].node_addr.len);

            /* Record the number of allocated slots to the slots list. */
            slots_list[j] = gnrc_netdev->rx.rx_register_list[i].queue_indicator;

            total_tdma_node_num++;
            total_tdma_slot_num += slots_list[j];

            /* If there is no room for allocating more slots, stop. */
            if (total_tdma_slot_num >= IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM) {
                uint8_t redueced_slots_num;
                redueced_slots_num = total_tdma_slot_num - IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM;
                slots_list[j] -= redueced_slots_num;
                total_tdma_slot_num -= redueced_slots_num;
                break;
            }
            j++;
        }
    }

    iqueuemac_hdr.schedulelist_size = total_tdma_node_num;

    if (total_tdma_node_num > 0) {
    	/* If there are slots to allocate, add the slots list and the ID list to
    	 * the beacon! */
        gnrc_netdev->rx.router_vtdma_mana.total_slots_num = total_tdma_slot_num;

        /* Add the slots list to the beacon. */
        pkt = gnrc_pktbuf_add(NULL, slots_list, total_tdma_node_num * sizeof(uint8_t),
                              GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            return -ENOBUFS;
        }
        pkt_iqmac = pkt;

        /* Add the ID list to the beacon. */
        pkt = gnrc_pktbuf_add(pkt, id_list, total_tdma_node_num * sizeof(l2_id_t),
                              GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            gnrc_pktbuf_release(pkt_iqmac);
            return -ENOBUFS;
        }
        pkt_iqmac = pkt;

        /* Add the GoMacH header to the beacon. */
        pkt = gnrc_pktbuf_add(pkt, &iqueuemac_hdr, sizeof(iqueuemac_hdr), GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            gnrc_pktbuf_release(pkt_iqmac);
            return -ENOBUFS;
        }
        pkt_iqmac = pkt;
    }
    else {
        /* If there is no slots to allocate, quit sending beacon! */
        return 0;
    }

    /* Add the Netif header. */
    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
        gnrc_pktbuf_release(pkt_iqmac);
        return -ENOBUFS;
    }
    pkt_iqmac = pkt;

    /* We wouldn't get here if add the NETIF header had failed,
     * so no sanity checks needed. */
    nethdr_beacon = (gnrc_netif_hdr_t *) (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF))->data;

    /* Construct NETIF header. */
    gnrc_netif_hdr_init(nethdr_beacon, 0, 0);

    /* Send beacon as broadcast*/
    nethdr_beacon->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    int res;
    if (gnrc_netdev->gomach.get_other_preamble == true) {
        /* Use csma for collision avoidance if we found ongoing preamble transmission. */
        res = gnrc_gomach_send(gnrc_netdev, pkt, NETOPT_ENABLE);
    }
    else {
        /* Send the beacon without CSMA if there is no ongoing preamble transmission. */
        res = gnrc_gomach_send(gnrc_netdev, pkt, NETOPT_DISABLE);
    }

    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH]: send beacon failed, release it.\n");
        gnrc_pktbuf_release(pkt);
    }
    return res;
}

int gnrc_gomach_dispatch_defer(gnrc_pktsnip_t *buffer[], gnrc_pktsnip_t *pkt)
{
    assert(buffer != NULL);
    assert(pkt != NULL);

    for (unsigned i = 0; i < GNRC_MAC_DISPATCH_BUFFER_SIZE; i++) {
        /* Buffer will be filled bottom-up and emptied completely so no holes */
        if (buffer[i] == NULL) {
            buffer[i] = pkt;
            return 0;
        }
    }

    gnrc_pktbuf_release(pkt);
    LOG_ERROR("ERROR: [GOMACH]: dispatch buffer full, drop pkt.\n");

    return -1;
}

void gnrc_gomach_indicator_update(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt,
                                  iqueuemac_packet_info_t *pa_info)
{
    assert(gnrc_netdev != NULL);
    assert(pkt != NULL);
    assert(pa_info != NULL);

    iqueuemac_frame_data_t *iqueuemac_data_hdr;
    iqueuemac_data_hdr = (iqueuemac_frame_data_t *)
                         gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);

    if (iqueuemac_data_hdr == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: GoMacH's data header is null.\n");
        return;
    }
    iqueuemac_data_hdr = ((gnrc_pktsnip_t *) iqueuemac_data_hdr)->data;

    int i;
    /* Check whether the device has been registered or not. */
    for (i = 0; i < IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT; i++) {
        if (memcmp(&gnrc_netdev->rx.rx_register_list[i].node_addr.addr,
                   &pa_info->src_addr.addr,
                   pa_info->src_addr.len) == 0) {
            /* Update the sender's queue-length indicator. */
            gnrc_netdev->rx.rx_register_list[i].queue_indicator = iqueuemac_data_hdr->queue_indicator;
            return;
        }
    }

    /* The sender has not registered yet. */
    for (i = 0; i < IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT; i++) {
        if ((gnrc_netdev->rx.rx_register_list[i].node_addr.len == 0) ||
            (gnrc_netdev->rx.rx_register_list[i].queue_indicator == 0)) {
            gnrc_netdev->rx.rx_register_list[i].node_addr.len = pa_info->src_addr.len;
            memcpy(gnrc_netdev->rx.rx_register_list[i].node_addr.addr,
                   pa_info->src_addr.addr,
                   pa_info->src_addr.len);

            /* Update the sender's queue-length indicator. */
            gnrc_netdev->rx.rx_register_list[i].queue_indicator = iqueuemac_data_hdr->queue_indicator;
            return;
        }
    }
}

bool gnrc_gomach_check_duplicate(gnrc_netdev_t *gnrc_netdev, iqueuemac_packet_info_t *pa_info)
{
    assert(gnrc_netdev != NULL);
    assert(pa_info != NULL);

    int i;
    /* First check if we can found the same source sender ID in the recorded info units. */
    for (i = 0; i < IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE; i++) {
        if (memcmp(&gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr,
                   &pa_info->src_addr.addr,
                   pa_info->src_addr.len) == 0) {
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
            if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].seq == pa_info->seq) {
                /* Found same MAC sequence, this is duplicate packet . */
                return true;
            }
            else {
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].seq = pa_info->seq;
                return false;
            }
        }
    }

    /* Look for a free info unit */
    for (i = 0; i < IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE; i++) {
        if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len == 0) {
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len = pa_info->src_addr.len;
            memcpy(gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr,
                   pa_info->src_addr.addr,
                   pa_info->src_addr.len);
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].seq = pa_info->seq;
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
            return false;
        }
    }

    return false;
}

void gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_pktsnip_t *pkt;

    iqueuemac_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {

        /* parse the packet */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case FRAMETYPE_BEACON: {

                gnrc_pktbuf_release(pkt);
#if 0
                uint32_t own_phase;
                uint32_t sender_phase;

                own_phase = gnrc_gomach_phase_now(gomach);

                iqueuemac_frame_beacon_t *iqueuemac_beacon_hdr;
                iqueuemac_beacon_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_GOMACH);

                /* this means that the beacon sender will change its phase, so meaningless to deal with this received beacon */
                if (iqueuemac_beacon_hdr->current_phase == 0) {
                    gnrc_pktbuf_release(pkt);
                    continue;
                }

                sender_phase = iqueuemac_beacon_hdr->current_phase + RTT_US_TO_TICKS(IQUEUEMAC_RECEPTION_MAGIN_US);

                /* in case the sender's phase is larger */
                if (sender_phase >= own_phase) {

                    uint32_t phase_gap_ticks;

                    /* calculate the gap in ticks */
                    phase_gap_ticks = sender_phase - own_phase;

                    if (phase_gap_ticks < RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US)) {
                        uint32_t backoff_ticks;
                        uint32_t random_backoff;

                        gomach->phase_backoff = true;
                        backoff_ticks = RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US + IQUEUEMAC_RECEPTION_MAGIN_US) - phase_gap_ticks;

                        random_backoff = RTT_US_TO_TICKS((IQUEUEMAC_SUPERFRAME_DURATION_US / 2));
                        random_backoff = random_uint32_range(0, random_backoff);
                        gomach->backoff_phase_ticks =  backoff_ticks + random_backoff;
                    }
                }
                else {
                    ; /* currently, we don't deal with the case the sender's phase is smaller */
                }

                gnrc_pktbuf_release(pkt);
                /* in the future, take CP overlape collision measurements after receive ohter's beacon!! */
#endif

            } break;

            case FRAMETYPE_PREAMBLE: {
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) {
                    gnrc_netdev->gomach.got_preamble = true;
                    /** if reception is not going on, reply preamble-ack,
                     * also, don't send preamble-ACK if CP ends. **/
                    if (gnrc_gomach_get_netdev_state(gnrc_netdev) == NETOPT_STATE_IDLE) {
                        /***  disable auto-ack ***/
                        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

                        int res;
                        res = gnrc_gomach_send_preamble_ack(gnrc_netdev, &receive_packet_info);
                        if (res < 0) {
                            printf("preamble-ack: res %d\n", res);
                        }

                        /* Enable Auto ACK again for data reception */
                        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);
                    }
                }
                else {
                    //gomach->quit_current_cycle = true;
                    /* if receives unintended preamble, don't send beacon and quit the following vTDMA period. */
                    gnrc_netdev->gomach.get_other_preamble = true;
                }
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE_ACK: {
                gnrc_pktbuf_release(pkt);

            } break;


            // gomach.rx.last_seq_info.seq = netif_hdr->seq;
            case FRAMETYPE_IQUEUE_DATA: {

                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) {
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        puts("dup pkt.");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    _dispatch(gnrc_netdev->rx.dispatch_buffer);
                }
                else { /* if the data is not for the node, release it.  */
                      /* it is very unlikely that we will receive not-intended data here, since CP will not overlape! */
                    gnrc_pktbuf_release(pkt);
                }
                //gnrc_pktbuf_release(pkt);
                //printf("%lu. \n", RTT_TICKS_TO_US(gnrc_gomach_phase_now(gomach)));
                //puts("gomach: router receives a data !!");
            } break;

            case FRAMETYPE_BROADCAST: {
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                _dispatch(gnrc_netdev->rx.dispatch_buffer);
                //puts("gomach: router receives a broadcast data !!");
            } break;

            default: {
                gnrc_pktbuf_release(pkt); break;
            }
        }

    } /* end of while loop */
}

void iqueuemac_packet_process_in_init(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_pktsnip_t *pkt;

    iqueuemac_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {

        /* parse the packet */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case FRAMETYPE_BEACON: {
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE: {
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE_ACK: {
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_IQUEUE_DATA: {
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_BROADCAST: {
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                _dispatch(gnrc_netdev->rx.dispatch_buffer);
                //puts("gomach: router receives a broadcast data !!");
            } break;

            case FRAMETYPE_ANNOUNCE: {
                /*** it seems that this "init_retry" procedure is unnecessary here!! maybe delete it in the future ***/
                //gomach->router_states.init_retry = true;
                gnrc_pktbuf_release(pkt);
            } break;


            default: gnrc_pktbuf_release(pkt); break;
        }

    } /* end of while loop */
}

void iqueuemac_init_choose_subchannel(gnrc_netdev_t *gnrc_netdev)
{

    uint16_t subchannel_seq, check_seq, own_id;

/*
    memcpy(&own_id,
           gnrc_netdev->l2_addr,
           2);
*/
	own_id = 0;
	own_id = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len-2];
	own_id = own_id << 8;
	own_id |= gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len-1];

    /* range from 12 to 25 */
    //own_id = 12;
    subchannel_seq = 12 + (own_id % 14);
    //printf("gomach: the random selected subchannel is %d .\n", subchannel_seq);
    //printf("gomach: subchannel flag is %d .\n", gomach->subchannel_occu_flags);

    int i = 0;
    for (i = 0; i < 14; i++) {
        /* range from 1 to 14, lead to 2nd to 15th bit */
        check_seq = subchannel_seq - 11;
        check_seq = (1 << check_seq);

        if (check_seq & gnrc_netdev->gomach.subchannel_occu_flags) {
            //puts("gomach: subchannel exist, find next subchannel.");
            own_id += 1;
            subchannel_seq = 12 + (own_id % 14);
        }
        else {
            break;
        }
    }

    gnrc_netdev->gomach.sub_channel_num = subchannel_seq;
    //printf("gomach: the final selected subchannel is %d .\n", subchannel_seq);
}

int gomach_send_preamble(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma)
{
    /****** assemble and send the beacon ******/
    gnrc_pktsnip_t *pkt;
    gnrc_netif_hdr_t *nethdr_preamble;
    gnrc_pktsnip_t *pkt_iqmac;

    /* Assemble preamble packet */
    iqueuemac_frame_preamble_t iqueuemac_preamble_hdr;

    iqueuemac_preamble_hdr.header.type = FRAMETYPE_PREAMBLE;
    memcpy(iqueuemac_preamble_hdr.dst_addr.addr,
           gnrc_netdev->tx.current_neighbor->l2_addr,
           gnrc_netdev->tx.current_neighbor->l2_addr_len);
    iqueuemac_preamble_hdr.dst_addr.len = gnrc_netdev->tx.current_neighbor->l2_addr_len;

    pkt = gnrc_pktbuf_add(NULL, &iqueuemac_preamble_hdr, sizeof(iqueuemac_preamble_hdr), GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        puts("gomach: preamble_hdr buf add failed in iqueue_mac_send_preamble().");
        return -ENOBUFS;
    }
    pkt_iqmac = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        puts("gomach: preamble_hdr netif add failed in iqueue_mac_send_preamble().");
        gnrc_pktbuf_release(pkt_iqmac);
        return -ENOBUFS;
    }
    pkt_iqmac = pkt;

    /* We wouldn't get here if add the NETIF header had failed, so no
        sanity checks needed */
    nethdr_preamble = (gnrc_netif_hdr_t *) (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF))->data;

    /* Construct NETIF header and initiate address fields */
    gnrc_netif_hdr_init(nethdr_preamble, 0, 0);
    //gnrc_netif_hdr_set_dst_addr(nethdr_wa, lwmac->rx.l2_addr.addr, lwmac->rx.l2_addr.len);

    /* Send WA as broadcast*/
    nethdr_preamble->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    netopt_enable_t csma_enable;
    csma_enable = use_csma;
    int res;

    res = gnrc_gomach_send(gnrc_netdev, pkt, csma_enable);
    if (res < 0) {
        puts("gomach: send preamble failed in iqueue_mac_send_preamble().");
        gnrc_pktbuf_release(pkt_iqmac);
    }
    return res;
}


void gomach_bcast_subchann_seq(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma)
{
    /****** assemble and send the beacon ******/
    gnrc_pktsnip_t *pkt;
    gnrc_netif_hdr_t *nethdr_announce;

    /* Assemble announce packet */
    iqueuemac_frame_announce_t iqueuemac_announce_hdr;

    iqueuemac_announce_hdr.header.type = FRAMETYPE_ANNOUNCE;
    iqueuemac_announce_hdr.subchannel_seq = gnrc_netdev->gomach.sub_channel_num;

    pkt = gnrc_pktbuf_add(NULL, &iqueuemac_announce_hdr, sizeof(iqueuemac_announce_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        puts("gomach: pktbuf add failed in iqueuemac_send_announce().");
    }

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        puts("gomach: pktbuf add failed in iqueuemac_send_announce().");
    }
    /* We wouldn't get here if add the NETIF header had failed, so no
        sanity checks needed */
    nethdr_announce = (gnrc_netif_hdr_t *)
                      (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF))->data;

    /* Construct NETIF header and initiate address fields */
    gnrc_netif_hdr_init(nethdr_announce, 0, 0);

    /* Send WA as broadcast*/
    nethdr_announce->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    netopt_enable_t csma_enable;
    csma_enable = use_csma;
    gnrc_gomach_send(gnrc_netdev, pkt, csma_enable);
}

void iqueuemac_device_process_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt, iqueuemac_packet_info_t *pa_info)
{

    iqueuemac_frame_preamble_ack_t *iqueuemac_preamble_ack_hdr;

    iqueuemac_preamble_ack_hdr = (iqueuemac_frame_preamble_ack_t *) gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);

    if (iqueuemac_preamble_ack_hdr == NULL) {
        puts("iqueuemac_preamble_ack_hdr is null");
        return;
    }

    iqueuemac_preamble_ack_hdr = ((gnrc_pktsnip_t *)iqueuemac_preamble_ack_hdr)->data;

    /***** update all the necessary information to marked as a known neighbor ****/
    gnrc_netdev->tx.current_neighbor->mac_type = KNOWN;

    /*** remember to reduce a bit the phase for locking, since there is a hand-shake procedure before ***/
    //uint32_t  phase_ticks;
    /*** adjust the phase of the receiver ***/

    long int phase_ticks;

    if ((gnrc_netdev->gomach.phase_changed == true) && (gnrc_netdev->rx.enter_new_cycle == true)) {
        /* this means that the node is already in a new cycle when doing phase changed.
         * So, give some compensation for later phase adjust */
        phase_ticks = gnrc_gomach_phase_now(gnrc_netdev) + gnrc_netdev->gomach.backoff_phase_ticks - iqueuemac_preamble_ack_hdr->phase_in_ticks;
        //phase_ticks += (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US) / 4);
    }
    else {
        phase_ticks = gnrc_gomach_phase_now(gnrc_netdev) - iqueuemac_preamble_ack_hdr->phase_in_ticks;
    }

    if (phase_ticks < 0) {
        phase_ticks += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
    }

    /* check if phase is too close */
    long int future_neighbor_phase;
    if (gnrc_netdev->gomach.phase_changed == true) {
        future_neighbor_phase = phase_ticks - gnrc_netdev->gomach.backoff_phase_ticks;

        if (future_neighbor_phase < 0) {
            future_neighbor_phase += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
        }
    }
    else {
        future_neighbor_phase = phase_ticks;
    }

    uint32_t neighbor_phase;
    neighbor_phase = (uint32_t)future_neighbor_phase;

    /* if the sender's phase is too close to the receiver */
    if ((RTT_TICKS_TO_US(neighbor_phase) > (IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_MIN_GAP_US)) ||
        (RTT_TICKS_TO_US(neighbor_phase) < IQUEUEMAC_CP_MIN_GAP_US)) {
        puts("p close");
        gnrc_netdev->gomach.phase_backoff = true;
        gnrc_netdev->gomach.backoff_phase_ticks =
            random_uint32_range(RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US), RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_MIN_GAP_US));
    }

    /*** move 1/3 CP duration to give some time redundancy for sender the has forward timer-drift!!! ***/
    //phase_ticks = (phase_ticks + (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)/3)) % RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);

    gnrc_netdev->tx.current_neighbor->cp_phase = (uint32_t)phase_ticks; //gnrc_gomach_phase_now(gomach); //- RTT_US_TO_TICKS(IQUEUEMAC_WAIT_CP_SECUR_GAP_US); rtt_get_counter();


#if 0
    if ((gomach->father_router_addr.len != 0) && (_addr_match(&gomach->father_router_addr, &iqueuemac_preamble_ack_hdr->father_router))) {
        //gomach->tx.current_neighbour->in_same_cluster = true;
        //gomach->tx.current_neighbour->cp_phase = 0;

        gomach->tx.current_neighbour->in_same_cluster = false;
        gomach->tx.current_neighbour->cp_phase = rtt_get_counter();

        puts("gomach: node got phase-locked with father.");

        /***  add the node type into the in-cluster list if the receiver and the sender share the same father ***/
        if (iqueuemac_preamble_ack_hdr->device_type == NODE) {
            iqueuemac_add_in_cluster_neighbor(gomach, &pa_info->src_addr);
        }
        //puts("gomach: get phased-locked, in the same cluster.");
    }
    else { //for router type, it will automatically enter here, since father-router are different
        gomach->tx.current_neighbour->in_same_cluster = false;
        gomach->tx.current_neighbour->cp_phase = rtt_get_counter();

        if (iqueuemac_preamble_ack_hdr->device_type == NODE) {
            iqueuemac_remove_in_cluster_neighbor(gomach, &pa_info->src_addr);
        }
        //puts("gomach: get phased-locked, not in the same cluster.");
    }


    /* if this is the father router, get phase-locked!!!!  */
    if ((_addr_match(&gomach->father_router_addr, &pa_info->src_addr)) && (gomach->mac_type == NODE)) {
        rtt_clear_alarm();

        uint32_t phase_ticks;
        uint32_t alarm;
        //uint32_t  current_timing_us;

        /**set the rtt counter to be the same as father router*/
        phase_ticks = iqueuemac_preamble_ack_hdr->phase_in_ticks;
        rtt_set_counter(phase_ticks);

        /** set set next rtt alarm*/
        if (phase_ticks >= RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)) {
            alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
            iqueuemac_set_rtt_alarm(alarm, (void *) IQUEUEMAC_EVENT_RTT_N_ENTER_CP);
            gomach->node_states.in_cp_period = false;
            //puts("gomach: node got phase-locked with father in sleep.");
        }
        else {
            alarm = RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
            iqueuemac_set_rtt_alarm(alarm, (void *) IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP);
            gomach->node_states.in_cp_period = true;
            puts("gomach: node got phase-locked with father in CP.");
        }

    }
#endif

}

void iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_pktsnip_t *pkt;

    iqueuemac_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* parse the packet */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case FRAMETYPE_BEACON: {
                /* it is very unlikely that we will receive beacon here.  */
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE: {
                /* Due to non-overlap CP rule, it is very unlikely that we will receive preamble here.
                * But, in case it happens, quit this t-2-u for collision avoidance.
                * Release all received preamle here to reduce complexity. Only reply preamble in CP.*/
                gnrc_pktbuf_release(pkt);
                puts("q-u-1");
                gnrc_netdev->gomach.quit_current_cycle = true;
            } break;

            case FRAMETYPE_PREAMBLE_ACK: {
                //puts("gomach: nodes receives a preamble_ack");
                if ((memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) &&
                    (memcmp(&gnrc_netdev->tx.current_neighbor->l2_addr,
                            &receive_packet_info.src_addr.addr,
                            gnrc_netdev->tx.current_neighbor->l2_addr_len) == 0)) {
                    gnrc_netdev->tx.got_preamble_ack = true;

                    iqueuemac_device_process_preamble_ack(gnrc_netdev, pkt, &receive_packet_info);

                    /**got preamble-ack, flush the rx queue***/
                    gnrc_pktbuf_release(pkt);
                    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
                    return;
                }
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_IQUEUE_DATA: {
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) {
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        puts("dup pkt.");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    _dispatch(gnrc_netdev->rx.dispatch_buffer);
                }
                else { /* if the data is not for the node, release it.  */

                    gnrc_pktbuf_release(pkt);
                }
            } break;

            case FRAMETYPE_BROADCAST: {
                /* Due to non-overlap CP rule, it is very unlikely that we will receive broadcast here.
                 * But, in case it happens, quit this t-2-u for collision avoidance.
                 * Release the broadcast pkt, and receive it in CP, thus to reduce complexity.*/
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_pktbuf_release(pkt);
                puts("q-u-2");
            } break;

            default: gnrc_pktbuf_release(pkt); break;
        }

    } /* end of while loop */

}

int gomach_send_data_packet(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable)
{
    gnrc_pktsnip_t *pkt;

    pkt = gnrc_netdev->tx.packet;

    if (pkt == NULL) {
        puts("iqsend: pkt was NULL\n");
        return -EINVAL;
    }

    /*** enable auto-ACK ??? ***/

    /* Insert iqueue-mac header above NETIF header */
    iqueuemac_frame_data_t *iqueuemac_data_hdr_pointer;
    iqueuemac_data_hdr_pointer = (iqueuemac_frame_data_t *) gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);

    if (iqueuemac_data_hdr_pointer == NULL) {

        iqueuemac_frame_data_t iqueuemac_data_hdr;
        iqueuemac_data_hdr.header.type = FRAMETYPE_IQUEUE_DATA;
        iqueuemac_data_hdr.queue_indicator = gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue);

        /* save payload pointer */
        gnrc_pktsnip_t *payload = gnrc_netdev->tx.packet->next;

        pkt->next = gnrc_pktbuf_add(pkt->next, &iqueuemac_data_hdr, sizeof(iqueuemac_data_hdr), GNRC_NETTYPE_GOMACH);
        if (pkt->next == NULL) {
            puts("gomach: pktbuf add failed in iqueuemac_send_data_packet().");
            /* make append payload after netif header again */
            gnrc_netdev->tx.packet->next = payload;
            return -ENOBUFS;
        }

    }
    else {
        /*** update queue-indicator ***/
        iqueuemac_data_hdr_pointer->queue_indicator = gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue);
    }

    gnrc_pktbuf_hold(gnrc_netdev->tx.packet, 1);

    int res;
    res = gnrc_gomach_send(gnrc_netdev, gnrc_netdev->tx.packet, csma_enable);
    if (res < 0) {
        /* If res is < 0, then, the old pkt will not be released in send(). so need to release old data once */
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
        puts("gomach: tx-res < 0 in iqueuemac_send_data_packet().");
    }
    return res;

}


bool gomach_find_next_tx_neighbor(gnrc_netdev_t *gnrc_netdev)
{
    int next = -1;

    //uint32_t phase_check;
    //uint32_t phase_nearest = GNRC_GOMACH_PHASE_MAX;

    /*** If current_neighbour is not NULL, means last t-2-r or t-2-u failed, will continue try t-2-r/t-2-u
     * again for the same neighbor, which has not been released in last t-2-r/t-2-u. ***/
    if (gnrc_netdev->tx.current_neighbor != NULL) {
        return true;
    }

    if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.neighbors[0].queue) > 0) {
        next = 0;
    }
    else {
        /*** find the next neighbor ***/
        uint32_t j;
        j = gnrc_netdev->tx.last_tx_neighbor_id + 1;

        if (j >= GNRC_MAC_NEIGHBOR_COUNT) {
            j = 1;
        }

        for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {

            if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.neighbors[j].queue) > 0) {
                gnrc_netdev->tx.last_tx_neighbor_id = j;
                next = (int)j;
                break;
            }
            else {
                j++;
                if (j >= GNRC_MAC_NEIGHBOR_COUNT) {
                    j = 1;
                }
            }

#if 0
            if (gomach->tx.neighbours[i].queue.length > 0) {
                /* Unknown destinations are initialized with their phase at the end
                 * of the local interval, so known destinations that still wakeup
                 * in this interval will be preferred. */
                phase_check = gnrc_gomach_ticks_until_phase(gomach, gomach->tx.neighbours[i].cp_phase);

                if (phase_check <= phase_nearest) {
                    next = i;
                    phase_nearest = phase_check;
                }
            }
#endif
        }
    }
    ////////
    if (next >= 0) {
        gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.neighbors[next].queue);
        if (pkt != NULL) {
            gnrc_netdev->tx.packet = pkt;
            gnrc_netdev->tx.current_neighbor = &gnrc_netdev->tx.neighbors[next];
            gnrc_netdev->tx.tx_seq = 0;
            gnrc_netdev->tx.t2u_retry_counter = 0;

            //printf("gomach: the find nearest neighbor is %d. \n", next);
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}

/*
   bool iqueuemac_check_has_pending_packet(packet_queue_t* q)
   {
    gnrc_pktsnip_t* pkt;
    pkt = packet_queue_head(q);

    if(pkt != NULL){
        return true;
    }

    return false;
   }*/


void iqueuemac_beacon_process(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt)
{
    iqueuemac_frame_beacon_t *iqueuemac_beacon_hdr;
    gnrc_pktsnip_t *iqueuemac_snip;

    l2_id_t *id_list;
    uint8_t *slots_list;
    uint8_t schedulelist_size = 0;
    bool got_allocated_slots;
    uint8_t id_position;
    uint8_t slots_position;

    iqueuemac_beacon_hdr = (iqueuemac_frame_beacon_t *) gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);

    if (iqueuemac_beacon_hdr == NULL) {
        puts("iqueuemac_beacon_hdr is null");
        return;
    }

    iqueuemac_beacon_hdr = ((gnrc_pktsnip_t *) iqueuemac_beacon_hdr)->data;

    schedulelist_size = iqueuemac_beacon_hdr->schedulelist_size;
    gnrc_netdev->tx.vtdma_para.sub_channel_seq = iqueuemac_beacon_hdr->sub_channel_seq;

    if (schedulelist_size == 0) {
        gnrc_netdev->tx.vtdma_para.slots_num = 0;
        gnrc_netdev->tx.vtdma_para.slots_position = 0;
        return;
    }

    /**** take the ID-list out ****/
    iqueuemac_snip = gnrc_pktbuf_mark(pkt, schedulelist_size * sizeof(l2_id_t), GNRC_NETTYPE_GOMACH);
    id_list = iqueuemac_snip->data;
    /**** take the slots-list out ****/
    slots_list = pkt->data;

    /**** check whether has been allocated slots ****/
    int i = 0;
    got_allocated_slots = false;
    id_position = 0;

    for (i = 0; i < schedulelist_size; i++) {
        if (memcmp(gnrc_netdev->l2_addr, id_list[i].addr, gnrc_netdev->l2_addr_len) == 0) {
            got_allocated_slots = true;
            id_position = i;
        }
    }

    /**** find the slots number and position ****/
    if (got_allocated_slots == true) {
        gnrc_netdev->tx.vtdma_para.slots_num = slots_list[id_position];

        slots_position = 0;
        for (i = 0; i < id_position; i++) {
            slots_position += slots_list[i];
        }
        gnrc_netdev->tx.vtdma_para.slots_position = slots_position;

        //printf("gomach: the allocated slots-num is %d, id-position is %d .\n", gomach->tx.vtdma_para.slots_num, id_position);
    }
    else {
        gnrc_netdev->tx.vtdma_para.slots_num = 0;
        gnrc_netdev->tx.vtdma_para.slots_position = 0;
    }
}

/****** check whether this function can be merged with router-wait-beacon-packet-process!!! ******/
void gomach_wait_beacon_packet_process(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_pktsnip_t *pkt;

    iqueuemac_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {

        /* parse the packet */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case FRAMETYPE_BEACON: {
                if (memcmp(&gnrc_netdev->tx.current_neighbor->l2_addr,
                           &receive_packet_info.src_addr.addr,
                           gnrc_netdev->tx.current_neighbor->l2_addr_len) == 0) {
                    gnrc_netdev->tx.vtdma_para.get_beacon = true;
                    iqueuemac_beacon_process(gnrc_netdev, pkt);
                }
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE: {
                /* Release preamble pkt no matter the preamble is for it or not, and quit the t-2-r. */
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_PREAMBLE_ACK: {
                // should we quit this period also??!
                gnrc_pktbuf_release(pkt);
            } break;

            case FRAMETYPE_IQUEUE_DATA: {
                /* It is unlikely that we will received a data for us here. This means the nodes' CP is close with its
                 * destination's. */
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) {
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        puts("dup pkt.");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    _dispatch(gnrc_netdev->rx.dispatch_buffer);
                }
                else { /* if the data is not for the node, release it.  */
                      /* it is very unlikely that we will receive not-intended data here, since CP will not overlape! */
                    gnrc_pktbuf_release(pkt);
                }
            } break;

            case FRAMETYPE_BROADCAST: {
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_pktbuf_release(pkt);
            } break;

            default: gnrc_pktbuf_release(pkt); break;
        }

    } /* end of while loop */
}

void iqueuemac_router_vtdma_receive_packet_process(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_pktsnip_t *pkt;

    iqueuemac_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {

        /* parse the packet */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case FRAMETYPE_IQUEUE_DATA: {
                gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                    gnrc_pktbuf_release(pkt);
                    puts("dup pkt.");
                    return;
                }
                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);

                _dispatch(gnrc_netdev->rx.dispatch_buffer);
                //puts("gomach: router receives a data in vtdma!!");
            } break;

            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }

    } /* end of while loop */
}

void gomach_figure_neighbors_new_phase(gnrc_netdev_t *gnrc_netdev)
{

    if (gnrc_netdev->gomach.phase_changed == true) {

        gnrc_netdev->gomach.phase_changed = false;

        for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {
            if (gnrc_netdev->tx.neighbors[i].mac_type == KNOWN) {
                long int tmp = gnrc_netdev->tx.neighbors[i].cp_phase - gnrc_netdev->gomach.backoff_phase_ticks;
                if (tmp < 0) {
                    tmp += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);

                    /* update the neighbor's cur_pub_channel if tmp < 0 */
                    if (gnrc_netdev->tx.neighbors[i].pub_chanseq == gnrc_netdev->gomach.pub_channel_1) {
                        gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_2;
                    }
                    else {
                        gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_1;
                    }
                }
                gnrc_netdev->tx.neighbors[i].cp_phase = (uint32_t)tmp;
            }
        }
    }
}

void _dispatch(gnrc_pktsnip_t **buffer)
{
    assert(buffer != NULL);

    for (unsigned i = 0; i < GNRC_MAC_DISPATCH_BUFFER_SIZE; i++) {
        if (buffer[i]) {

            /* save pointer to netif header */
            gnrc_pktsnip_t *netif = buffer[i]->next->next;

            /* remove gomach header */
            buffer[i]->next->next = NULL;
            gnrc_pktbuf_release(buffer[i]->next);

            /* make append netif header after payload again */
            buffer[i]->next = netif;

            if (!gnrc_netapi_dispatch_receive(buffer[i]->type, GNRC_NETREG_DEMUX_CTX_ALL, buffer[i])) {
                DEBUG("Unable to forward packet of type %i\n", buffer[i]->type);
                gnrc_pktbuf_release(buffer[i]);
                puts("dispatch pkt fail, drop it");
            }
            buffer[i] = NULL;
        }
    }
}

void update_neighbor_pubchan(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev->gomach.cur_pub_channel == gnrc_netdev->gomach.pub_channel_1) {
        gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_2;
    }
    else {
        gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_1;
    }

    /* update tx-nighbors' current channel */
    for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if (gnrc_netdev->tx.neighbors[i].mac_type == KNOWN) {
            /* switch public channel */
            if (gnrc_netdev->tx.neighbors[i].pub_chanseq == gnrc_netdev->gomach.pub_channel_1) {
                gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_2;
            }
            else {
                gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_1;
            }
        }
    }
}


void iqueuemac_broadcast_receive_packet_process(gnrc_netdev_t *gnrc_netdev){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL ) {
    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	/* it is very unlikely that we will receive beacon here.  */
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
            	/* Due to non-overlap CP rule, it is very unlikely that we will receive preamble here.
            	 * But, in case it happens, quit this t-2-u for collision avoidance.
            	 * Release all received preamle here to reduce complexity. Only reply preamble in CP.*/
            	gnrc_pktbuf_release(pkt);
            	gnrc_netdev->gomach.quit_current_cycle = true;
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr, gnrc_netdev->l2_addr_len) == 0) {
            		gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                	if((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))){
                		gnrc_pktbuf_release(pkt);
                		puts("dup pkt.");
                		return;
                	}

            		gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
            		_dispatch(gnrc_netdev->rx.dispatch_buffer);
            	}else {/* if the data is not for the node, release it.  */

            		gnrc_pktbuf_release(pkt);
            	}
            }break;

            case FRAMETYPE_BROADCAST:{
            	/* Due to non-overlap CP rule, it is very unlikely that we will receive broadcast here.
            	 * But, in case it happens, quit this t-2-u for collision avoidance.
            	 * Release the broadcast pkt, and receive it in CP, thus to reduce complexity.*/
        		gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
        		_dispatch(gnrc_netdev->rx.dispatch_buffer);
        		//puts("get bcast when send bcast");
        		gnrc_netdev->gomach.quit_current_cycle = true;
            }break;

            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */

}

/******************************************************************************/



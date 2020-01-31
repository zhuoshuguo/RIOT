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
#include "net/gnrc/mac/internal.h"
#include "net/gnrc/gomach/hdr.h"
#include "net/gnrc/gomach/gomach.h"
#include "net/gnrc/gomach/timeout.h"
#include "net/gnrc/gomach/types.h"
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

static int _parse_packet(gnrc_pktsnip_t *pkt, gnrc_gomach_packet_info_t *info)
{
    assert(info != NULL);
    assert(pkt != NULL);

    gnrc_netif_hdr_t *netif_hdr = NULL;
    gnrc_pktsnip_t *gomach_snip = NULL;
    gnrc_gomach_hdr_t *gomach_hdr = NULL;
    gnrc_pktsnip_t *netif_snip = NULL;

    netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_snip == NULL) {
        return -ENODATA;
    }
    else {
        netif_hdr = netif_snip->data;
    }

    if (netif_hdr->dst_l2addr_len > sizeof(info->dst_addr)) {
        return -ENODATA;
    }

    if (netif_hdr->src_l2addr_len > sizeof(info->src_addr)) {
        return -ENODATA;
    }

    /* Dissect GoMacH header, Every frame has header as first member */
    gomach_hdr = (gnrc_gomach_hdr_t *) pkt->data;

    switch (gomach_hdr->type) {
        case GNRC_GOMACH_FRAME_BEACON: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_beacon_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_PREAMBLE: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_preamble_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_RIBEACON: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_preamble_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_PREAMBLE_ACK: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_preamble_ack_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_DATA: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_data_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_ANNOUNCE: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_announce_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }
        case GNRC_GOMACH_FRAME_BROADCAST: {
            gomach_snip = gnrc_pktbuf_mark(pkt, sizeof(gnrc_gomach_frame_broadcast_t),
                                           GNRC_NETTYPE_GOMACH);
            break;
        }

        default: {
            return -ENODATA;
        }
    }

    /* Memory location may have changed while marking. */
    gomach_hdr = gomach_snip->data;

    /* Get the destination address. */
    switch (gomach_hdr->type) {
        case GNRC_GOMACH_FRAME_PREAMBLE: {
            info->dst_addr = ((gnrc_gomach_frame_preamble_t *)gomach_hdr)->dst_addr;
            break;
        }
        case GNRC_GOMACH_FRAME_PREAMBLE_ACK: {
            info->dst_addr = ((gnrc_gomach_frame_preamble_ack_t *)gomach_hdr)->dst_addr;
            break;
        }
        case GNRC_GOMACH_FRAME_DATA: {
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

    info->header = gomach_hdr;
    info->seq = netif_hdr->seq;
    return 0;
}

uint32_t gnrc_gomach_phase_now(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    uint32_t phase_now = rtt_get_counter();

    /* in case that rtt overflows */
    if (phase_now < gnrc_netdev->gomach.last_wakeup) {
        uint32_t gap_to_full = GNRC_GOMACH_PHASE_MAX - gnrc_netdev->gomach.last_wakeup;
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

int gnrc_gomach_send_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_gomach_packet_info_t *info)
{
    assert(gnrc_netdev != NULL);
    assert(info != NULL);

    gnrc_pktsnip_t *gomach_pkt = NULL;
    gnrc_pktsnip_t *pkt = NULL;
    gnrc_netif_hdr_t *nethdr_preamble_ack = NULL;

    /* Start assemble the preamble-ACK packet according to preamble packet info. */
    gnrc_gomach_frame_preamble_ack_t gomach_preamble_ack_hdr;

    gomach_preamble_ack_hdr.header.type = GNRC_GOMACH_FRAME_PREAMBLE_ACK;
    gomach_preamble_ack_hdr.dst_addr = info->src_addr;
    /* Tell the preamble sender the device's (preamble-ACK sender) current phase.
     * This is to allow the preamble sender to deduce the exact phase of the receiver. */
    gomach_preamble_ack_hdr.phase_in_ticks = gnrc_gomach_phase_now(gnrc_netdev);

    pkt = gnrc_pktbuf_add(NULL, &gomach_preamble_ack_hdr, sizeof(gomach_preamble_ack_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_preamble_ack().\n");
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: netif_hdr add failed in gnrc_gomach_send_preamble_ack().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    gnrc_pktsnip_t *netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_snip == NULL) {
        LOG_ERROR("[GOMACH]: NO netif_hdr found in gnrc_gomach_send_preamble_ack().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    else {
        nethdr_preamble_ack = netif_snip->data;
    }

    /* Construct NETIF header and insert address for preamble-ACK packet. */
    gnrc_netif_hdr_init(nethdr_preamble_ack, 0, 0);

    /* Send preamble-ACK as broadcast. */
    nethdr_preamble_ack->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    int res = gnrc_gomach_send(gnrc_netdev, pkt, NETOPT_DISABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH]: send preamble-ack failed in gnrc_gomach_send_preamble_ack().\n");
        gnrc_pktbuf_release(gomach_pkt);
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
    gnrc_netdev->rx.vtdma_manag.total_slots_num = 0;

    for (i = 0; i < GNRC_GOMACH_SLOSCH_UNIT_COUNT; i++) {
        if (gnrc_netdev->rx.slosch_list[i].queue_indicator > 0) {
            total_tdma_slot_num = gnrc_netdev->rx.slosch_list[i].queue_indicator;
            break;
        }
    }

    if (total_tdma_slot_num == 0) {
        return 0;
    }

    total_tdma_slot_num = 0;

    gnrc_pktsnip_t *pkt = NULL;
    gnrc_pktsnip_t *gomach_pkt = NULL;
    gnrc_netif_hdr_t *nethdr_beacon = NULL;

    /* Start assemble the beacon packet */
    gnrc_gomach_frame_beacon_t gomach_beaocn_hdr;
    gomach_beaocn_hdr.header.type = GNRC_GOMACH_FRAME_BEACON;
    gomach_beaocn_hdr.sub_channel_seq = gnrc_netdev->gomach.sub_channel_seq;

    /* Start generating the slots list and the related ID list for guiding
     * the following vTMDA procedure (slotted transmission). */
    gnrc_netdev->rx.vtdma_manag.total_slots_num = 0;

    gnrc_gomach_l2_id_t id_list[GNRC_GOMACH_SLOSCH_UNIT_COUNT];
    uint8_t slots_list[GNRC_GOMACH_SLOSCH_UNIT_COUNT];

    for (i = 0; i < GNRC_GOMACH_SLOSCH_UNIT_COUNT; i++) {
        if (gnrc_netdev->rx.slosch_list[i].queue_indicator > 0) {
            /* Record the device's (that will be allocated slots) address to the ID list. */
            memcpy(id_list[j].addr,
                   gnrc_netdev->rx.slosch_list[i].node_addr.addr,
                   gnrc_netdev->rx.slosch_list[i].node_addr.len);

            /* Record the number of allocated slots to the slots list. */
            slots_list[j] = gnrc_netdev->rx.slosch_list[i].queue_indicator;

            total_tdma_node_num++;
            total_tdma_slot_num += slots_list[j];

            /* If there is no room for allocating more slots, stop. */
            if (total_tdma_slot_num >= GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM) {
                uint8_t redueced_slots_num;
                redueced_slots_num = total_tdma_slot_num - GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM;
                slots_list[j] -= redueced_slots_num;
                total_tdma_slot_num -= redueced_slots_num;
                break;
            }
            j++;
        }
    }

    gomach_beaocn_hdr.schedulelist_size = total_tdma_node_num;

    if (total_tdma_node_num > 0) {
        /* If there are slots to allocate, add the slots list and the ID list to
         * the beacon! */
        gnrc_netdev->rx.vtdma_manag.total_slots_num = total_tdma_slot_num;

        /* Add the slots list to the beacon. */
        pkt = gnrc_pktbuf_add(NULL, slots_list, total_tdma_node_num * sizeof(uint8_t),
                              GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            return -ENOBUFS;
        }
        gomach_pkt = pkt;

        /* Add the ID list to the beacon. */
        pkt = gnrc_pktbuf_add(pkt, id_list, total_tdma_node_num * sizeof(gnrc_gomach_l2_id_t),
                              GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            gnrc_pktbuf_release(gomach_pkt);
            return -ENOBUFS;
        }
        gomach_pkt = pkt;

        /* Add the GoMacH header to the beacon. */
        pkt = gnrc_pktbuf_add(pkt, &gomach_beaocn_hdr, sizeof(gomach_beaocn_hdr), GNRC_NETTYPE_GOMACH);
        if (pkt == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
            gnrc_pktbuf_release(gomach_pkt);
            return -ENOBUFS;
        }
        gomach_pkt = pkt;
    }
    else {
        /* If there is no slots to allocate, quit sending beacon! */
        return 0;
    }

    /* Add the Netif header. */
    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_beacon().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    gnrc_pktsnip_t *beacon_netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (beacon_netif_snip == NULL) {
        LOG_ERROR("[GOMACH]: NO netif_hdr found in send_beacon().\n");
        gnrc_pktbuf_release(pkt);
        return -ENOBUFS;
    }
    else {
        nethdr_beacon = beacon_netif_snip->data;
    }

    /* Construct NETIF header. */
    gnrc_netif_hdr_init(nethdr_beacon, 0, 0);

    /* Send beacon as broadcast*/
    nethdr_beacon->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    int res;
    if (gnrc_gomach_get_unintd_preamble(gnrc_netdev)) {
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
    else {
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR,
                                GNRC_GOMACH_NO_TX_ISR_US);
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

    return -ENOBUFS;
}

void gnrc_gomach_indicator_update(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt,
                                  gnrc_gomach_packet_info_t *pa_info)
{
    assert(gnrc_netdev != NULL);
    assert(pkt != NULL);
    assert(pa_info != NULL);

    gnrc_gomach_frame_data_t *gomach_data_hdr = NULL;

    gnrc_pktsnip_t *gomach_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);
    if (gomach_snip == NULL) {
        LOG_ERROR("[GOMACH]: No gomach header found in gnrc_gomach_indicator_update().\n");
        return;
    }
    else {
        gomach_data_hdr = gomach_snip->data;
    }

    if (gomach_data_hdr == NULL) {
        LOG_ERROR("[GOMACH]: GoMacH's data header is null.\n");
        return;
    }

    int i;
    /* Check whether the device has been registered or not. */
    for (i = 0; i < GNRC_GOMACH_SLOSCH_UNIT_COUNT; i++) {
        if (memcmp(&gnrc_netdev->rx.slosch_list[i].node_addr.addr,
                   &pa_info->src_addr.addr,
                   pa_info->src_addr.len) == 0) {
            /* Update the sender's queue-length indicator. */
            gnrc_netdev->rx.slosch_list[i].queue_indicator = gomach_data_hdr->queue_indicator;
            return;
        }
    }

    /* The sender has not registered yet. */
    for (i = 0; i < GNRC_GOMACH_SLOSCH_UNIT_COUNT; i++) {
        if ((gnrc_netdev->rx.slosch_list[i].node_addr.len == 0) ||
            (gnrc_netdev->rx.slosch_list[i].queue_indicator == 0)) {
            gnrc_netdev->rx.slosch_list[i].node_addr.len = pa_info->src_addr.len;
            memcpy(gnrc_netdev->rx.slosch_list[i].node_addr.addr,
                   pa_info->src_addr.addr,
                   pa_info->src_addr.len);

            /* Update the sender's queue-length indicator. */
            gnrc_netdev->rx.slosch_list[i].queue_indicator = gomach_data_hdr->queue_indicator;
            return;
        }
    }
}

bool gnrc_gomach_check_duplicate(gnrc_netdev_t *gnrc_netdev, gnrc_gomach_packet_info_t *pa_info)
{
    assert(gnrc_netdev != NULL);
    assert(pa_info != NULL);

    int i;
    /* First check if we can found the same source sender ID in the recorded info units. */
    for (i = 0; i < GNRC_GOMACH_DUPCHK_BUFFER_SIZE; i++) {
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
    for (i = 0; i < GNRC_GOMACH_DUPCHK_BUFFER_SIZE; i++) {
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

void gnrc_gomach_cp_packet_process(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_gomach_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* Parse the received packet, fetch key MAC informations. */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            LOG_DEBUG("[GOMACH] CP: Packet could not be parsed: %i\n", res);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case GNRC_GOMACH_FRAME_PREAMBLE: {
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
                           gnrc_netdev->l2_addr_len) == 0) {
                    /* Get a preamble packet that is for the device itself. */
                    gnrc_gomach_set_got_preamble(gnrc_netdev, true);

                    /* If reception is not going on, reply preamble-ack. */
                    if (gnrc_gomach_get_netdev_state(gnrc_netdev) == NETOPT_STATE_IDLE) {
                        /* Disable auto-ack. */
                        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

                        int res = gnrc_gomach_send_preamble_ack(gnrc_netdev, &receive_packet_info);
                        if (res < 0) {
                            LOG_ERROR("ERROR: [GOMACH]: send preamble-ACK failed: %d.\n", res);
                        }

                        /* Enable Auto ACK again for data reception. */
                        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);
                    }
                }
                else {
                    /* Receives unintended preamble that is not for the device. */
                    gnrc_gomach_set_unintd_preamble(gnrc_netdev, true);
                }
                gnrc_pktbuf_release(pkt);
                break;
            }

            case GNRC_GOMACH_FRAME_DATA: {
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
                           gnrc_netdev->l2_addr_len) == 0) {
                    /* The data is for itself, now update the sender's queue-length indicator. */
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    /* Check that whether this is a duplicate packet. */
                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        LOG_DEBUG("[GOMACH]: received a duplicate packet.\n");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    gnrc_mac_dispatch(&gnrc_netdev->rx);
                }
                else {
                    /* If the data is not for the device, release it. */
                    gnrc_pktbuf_release(pkt);
                }
                break;
            }
            case GNRC_GOMACH_FRAME_BROADCAST: {
                /* Receive a broadcast packet, quit the listening period to avoid receive duplicate
                 * broadcast packet. */
                //gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                gnrc_mac_dispatch(&gnrc_netdev->rx);
                break;
            }
            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }
    }
}

int gnrc_gomach_bcast_rcv_packet_process(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_gomach_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* Parse the received packet, fetch key MAC informations. */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            LOG_DEBUG("[GOMACH] CP: Packet could not be parsed: %i\n", res);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case GNRC_GOMACH_FRAME_PREAMBLE: {

                gnrc_pktbuf_release(pkt);
                return -1;
            }

            case GNRC_GOMACH_FRAME_DATA: {
//                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
//                           gnrc_netdev->l2_addr_len) == 0) {
//                    /* The data is for itself, now update the sender's queue-length indicator. */
//                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);
//
//                    /* Check that whether this is a duplicate packet. */
//                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
//                        gnrc_pktbuf_release(pkt);
//                        LOG_DEBUG("[GOMACH]: received a duplicate packet.\n");
//                        return;
//                    }
//
//                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
//                    gnrc_mac_dispatch(&gnrc_netdev->rx);
//                }
//                else {
//                    /* If the data is not for the device, release it. */
//                    gnrc_pktbuf_release(pkt);
//                }
                gnrc_pktbuf_release(pkt);
                break;
            }
            case GNRC_GOMACH_FRAME_BROADCAST: {
                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                gnrc_mac_dispatch(&gnrc_netdev->rx);
                break;
            }
            case GNRC_GOMACH_FRAME_RIBEACON: {
            	gnrc_pktbuf_release(pkt);
                return 1;
            }
            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }
    }

    return 0;
}

void gnrc_gomach_init_choose_subchannel(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    uint16_t subchannel_seq, check_seq, own_id;

    own_id = 0;
    own_id = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 2];
    own_id = own_id << 8;
    own_id |= gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 1];

    /* First randomly set a sub-channel sequence, which ranges from 12 to 25. */
    subchannel_seq = 12 + (own_id % 14);

    /* Find a free sub-channel sequence. */
    int i = 0;
    for (i = 0; i < 14; i++) {
        check_seq = subchannel_seq - 11;
        check_seq = (1 << check_seq);

        if (check_seq & gnrc_netdev->gomach.subchannel_occu_flags) {
            LOG_INFO("INFO: [GOMACH]: sub-channel already occupied, find a new one.\n");
            own_id += 1;
            subchannel_seq = 12 + (own_id % 14);
        }
        else {
            break;
        }
    }

    gnrc_netdev->gomach.sub_channel_seq = subchannel_seq;
}

int gnrc_gomach_send_preamble(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_netif_hdr_t *nethdr_preamble;
    gnrc_pktsnip_t *gomach_pkt;

    /* Assemble the preamble packet. */
    gnrc_gomach_frame_preamble_t gomach_preamble_hdr;

    gomach_preamble_hdr.header.type = GNRC_GOMACH_FRAME_PREAMBLE;
    memcpy(gomach_preamble_hdr.dst_addr.addr,
           gnrc_netdev->tx.current_neighbor->l2_addr,
           gnrc_netdev->tx.current_neighbor->l2_addr_len);
    gomach_preamble_hdr.dst_addr.len = gnrc_netdev->tx.current_neighbor->l2_addr_len;

    pkt = gnrc_pktbuf_add(NULL, &gomach_preamble_hdr, sizeof(gomach_preamble_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_preamble().\n");
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: netif add failed in gnrc_gomach_send_preamble().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    gnrc_pktsnip_t *netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_snip == NULL) {
        LOG_ERROR("[GOMACH]: No netif_hdr found in gnrc_gomach_send_preamble().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    else {
        nethdr_preamble = netif_snip->data;
    }

    /* Construct NETIF header and initiate address fields. */
    gnrc_netif_hdr_init(nethdr_preamble, 0, 0);

    /* Send preamble packet as broadcast. */
    nethdr_preamble->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    return gnrc_gomach_send(gnrc_netdev, pkt, csma_enable);
}


int gnrc_gomach_send_RI_beacon(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_netif_hdr_t *nethdr_preamble;
    gnrc_pktsnip_t *gomach_pkt;

    /* Assemble the RI-beacon using preamble packet style. */
    gnrc_gomach_frame_preamble_t gomach_preamble_hdr;

    gomach_preamble_hdr.header.type = GNRC_GOMACH_FRAME_RIBEACON;

    /* No need to use address */
//    memcpy(gomach_preamble_hdr.dst_addr.addr,
//           gnrc_netdev->tx.current_neighbor->l2_addr,
//           gnrc_netdev->tx.current_neighbor->l2_addr_len);
//    gomach_preamble_hdr.dst_addr.len = gnrc_netdev->tx.current_neighbor->l2_addr_len;

    pkt = gnrc_pktbuf_add(NULL, &gomach_preamble_hdr, sizeof(gomach_preamble_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_RI_beacon().\n");
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: netif add failed in gnrc_gomach_send_RI_beacon().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    gnrc_pktsnip_t *netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_snip == NULL) {
        LOG_ERROR("[GOMACH]: No netif_hdr found in gnrc_gomach_send_preamble().\n");
        gnrc_pktbuf_release(gomach_pkt);
        return -ENOBUFS;
    }
    else {
        nethdr_preamble = netif_snip->data;
    }

    /* Construct NETIF header and initiate address fields. */
    gnrc_netif_hdr_init(nethdr_preamble, 0, 0);

    /* Send preamble packet as broadcast. */
    nethdr_preamble->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    return gnrc_gomach_send(gnrc_netdev, pkt, csma_enable);
}

int gnrc_gomach_bcast_subchann_seq(gnrc_netdev_t *gnrc_netdev, netopt_enable_t use_csma)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_pktsnip_t *gomach_pkt;
    gnrc_netif_hdr_t *nethdr_announce;

    /* Assemble the sub-channel sequence announce packet. */
    gnrc_gomach_frame_announce_t gomach_announce_hdr;

    gomach_announce_hdr.header.type = GNRC_GOMACH_FRAME_ANNOUNCE;
    gomach_announce_hdr.subchannel_seq = gnrc_netdev->gomach.sub_channel_seq;

    pkt = gnrc_pktbuf_add(NULL, &gomach_announce_hdr, sizeof(gomach_announce_hdr),
                          GNRC_NETTYPE_GOMACH);
    if (pkt == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_bcast_subchann_seq().\n");
        return -ENOBUFS;
    }
    gomach_pkt = pkt;

    pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
    if (pkt == NULL) {
        gnrc_pktbuf_release(gomach_pkt);
        LOG_ERROR("ERROR: [GOMACH]: netif add failed in gnrc_gomach_bcast_subchann_seq().\n");
        return -ENOBUFS;
    }

    gnrc_pktsnip_t *netif_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF);
    if (netif_snip == NULL) {
        LOG_ERROR("[GOMACH]: No netif_hdr found in gnrc_gomach_bcast_subchann_seq().\n");
        gnrc_pktbuf_release(pkt);
        return -ENOBUFS;
    }
    else {
        nethdr_announce = netif_snip->data;
    }

    /* Construct NETIF header and initiate address fields. */
    gnrc_netif_hdr_init(nethdr_announce, 0, 0);

    /* Send the packet as broadcast. */
    nethdr_announce->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    return gnrc_gomach_send(gnrc_netdev, pkt, use_csma);
}

void gnrc_gomach_process_preamble_ack(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt)
{
    assert(gnrc_netdev != NULL);
    assert(pkt != NULL);

    gnrc_gomach_frame_preamble_ack_t *gomach_preamble_ack_hdr = NULL;

    gnrc_pktsnip_t *gomach_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);
    if (gomach_snip == NULL) {
        LOG_ERROR("[GOMACH]: No gomach_snip found in gnrc_gomach_process_preamble_ack().\n");
        return;
    }
    else {
        gomach_preamble_ack_hdr = gomach_snip->data;
    }

    if (gomach_preamble_ack_hdr == NULL) {
        LOG_ERROR("[GOMACH]: preamble_ack_hdr is null.\n");
        return;
    }

    /* Mark the neighbor as phase-known */
    gnrc_netdev->tx.current_neighbor->mac_type = GNRC_GOMACH_TYPE_KNOWN;

    /* Fetch and deduce the exact phase of the neighbor. */
    long int phase_ticks;

    if (gnrc_gomach_get_phase_changed(gnrc_netdev) && gnrc_gomach_get_enter_new_cycle(gnrc_netdev)) {
        /* This means that this device is already in a new cycle after reset a new phase
         * (phase-backoff). So, give some compensation for later phase adjust. */
        phase_ticks = gnrc_gomach_phase_now(gnrc_netdev) +
                      gnrc_netdev->gomach.backoff_phase_ticks -
                      gomach_preamble_ack_hdr->phase_in_ticks;
    }
    else {
        phase_ticks = gnrc_gomach_phase_now(gnrc_netdev) -
                      gomach_preamble_ack_hdr->phase_in_ticks;
    }

    if (phase_ticks < 0) {
        phase_ticks += RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);
    }

    /* Check if the sender's phase is too close to the receiver. */
    long int future_neighbor_phase;
    if (gnrc_gomach_get_phase_changed(gnrc_netdev)) {
        future_neighbor_phase = phase_ticks - gnrc_netdev->gomach.backoff_phase_ticks;

        if (future_neighbor_phase < 0) {
            future_neighbor_phase += RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);
        }
    }
    else {
        future_neighbor_phase = phase_ticks;
    }

    uint32_t neighbor_phase = (uint32_t)future_neighbor_phase;

    if ((RTT_TICKS_TO_US(neighbor_phase) > (GNRC_GOMACH_SUPERFRAME_DURATION_US - GNRC_GOMACH_CP_MIN_GAP_US)) ||
        (RTT_TICKS_TO_US(neighbor_phase) < GNRC_GOMACH_CP_MIN_GAP_US)) {
        LOG_DEBUG("[GOMACH] t2u: own phase is close to the neighbor's.\n");
        gnrc_gomach_set_phase_backoff(gnrc_netdev, true);
        /* Set a random phase-backoff value. */
        gnrc_netdev->gomach.backoff_phase_ticks =
            random_uint32_range(RTT_US_TO_TICKS(GNRC_GOMACH_CP_MIN_GAP_US),
                                RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US -
                                                GNRC_GOMACH_CP_MIN_GAP_US));
    }

    gnrc_netdev->tx.current_neighbor->cp_phase = (uint32_t) phase_ticks;
}

void gnrc_gomach_process_pkt_in_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_gomach_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* Parse the received packet. */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            LOG_DEBUG("[GOMACH] t2u: Packet could not be parsed: %i\n", res);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case GNRC_GOMACH_FRAME_PREAMBLE: {
                /* Found other ongoing preamble transmission, quit its own t2u for
                 * collision avoidance. */
                gnrc_pktbuf_release(pkt);
                LOG_DEBUG("[GOMACH] t2u: found other preamble, quit t2u.\n");
                gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                break;
            }
            case GNRC_GOMACH_FRAME_PREAMBLE_ACK: {
                if ((memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
                            gnrc_netdev->l2_addr_len) == 0) &&
                    (memcmp(&gnrc_netdev->tx.current_neighbor->l2_addr,
                            &receive_packet_info.src_addr.addr,
                            gnrc_netdev->tx.current_neighbor->l2_addr_len) == 0)) {
                    /* Got preamble-ACK from targeted device. */
                    gnrc_gomach_set_got_preamble_ack(gnrc_netdev, true);

                    /* Analyze the preamble-ACK to get phase-locked with the neighbor device. */
                    gnrc_gomach_process_preamble_ack(gnrc_netdev, pkt);

                    gnrc_pktbuf_release(pkt);
                    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
                    return;
                }

                /* Preamble-ACK is not from targeted device. release it. */
                gnrc_pktbuf_release(pkt);
                break;
            }
            case GNRC_GOMACH_FRAME_DATA: {
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
                           gnrc_netdev->l2_addr_len) == 0) {
                    /* The data is for itself, now update the sender's queue-length indicator. */
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    /* Check that whether this is a duplicate packet. */
                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        LOG_DEBUG("[GOMACH] t2u: received a duplicate packet.\n");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    gnrc_mac_dispatch(&gnrc_netdev->rx);
                }
                else {
                    /* If the data is not for the device, release it.  */
                    gnrc_pktbuf_release(pkt);
                }
                break;
            }
            case GNRC_GOMACH_FRAME_BROADCAST: {
                /* Release the received broadcast pkt. Only receive broadcast packets in CP,
                 * thus to reduce complexity. */
                gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                gnrc_pktbuf_release(pkt);
                LOG_DEBUG("WARNING: [GOMACH] t2u: receive a broadcast packet, quit t2u.\n");
                break;
            }
            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }
    }
}

int gnrc_gomach_send_data(gnrc_netdev_t *gnrc_netdev, netopt_enable_t csma_enable)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt = gnrc_netdev->tx.packet;

    assert(pkt != NULL);

    /* Insert GoMacH header above NETIF header. */
    gnrc_gomach_frame_data_t *gomach_data_hdr_pointer;

    gnrc_pktsnip_t *gomach_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);
    if (gomach_snip != NULL) {
        gomach_data_hdr_pointer = gomach_snip->data;
    }
    else {
        gomach_data_hdr_pointer = NULL;
    }

    if (gomach_data_hdr_pointer == NULL) {
        /* No GoMacH header yet, build one. */
        gnrc_gomach_frame_data_t gomach_data_hdr;
        gomach_data_hdr.header.type = GNRC_GOMACH_FRAME_DATA;

        /* Set the queue-length indicator according to its current queue situation. */
        gomach_data_hdr.queue_indicator = gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue);

        /* Save the payload pointer. */
        gnrc_pktsnip_t *payload = gnrc_netdev->tx.packet->next;

        pkt->next = gnrc_pktbuf_add(pkt->next, &gomach_data_hdr, sizeof(gomach_data_hdr),
                                    GNRC_NETTYPE_GOMACH);
        if (pkt->next == NULL) {
            LOG_ERROR("ERROR: [GOMACH]: pktbuf add failed in gnrc_gomach_send_data().\n");

            /* Make append payload after netif header again. */
            gnrc_netdev->tx.packet->next = payload;
            return -ENOBUFS;
        }
    }
    else {
        /* GoMacH header exists, update the queue-indicator. */
        gomach_data_hdr_pointer->queue_indicator = gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue);
    }

    gnrc_pktbuf_hold(gnrc_netdev->tx.packet, 1);

    /* Send the data packet here. */
    return gnrc_gomach_send(gnrc_netdev, gnrc_netdev->tx.packet, csma_enable);
}

bool gnrc_gomach_find_next_tx_neighbor(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    int next = -1;

    /* If current neighbor pointer is not NULL, it means we have pending packet from last
     * t2u or t2k or bcast to send. In this case, return immediately. */
    if (gnrc_netdev->tx.current_neighbor != NULL) {
        return true;
    }

    /* First check whether we have broadcast packet to send. */
    if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.neighbors[0].queue) > 0) {
        next = 0;
    }
    else {
        /* Find the next neighbor to send data packet to. */

        /* Don't always start checking with ID 0, take turns to check every neighbor's queue,
         * thus to be more fair. */
        uint32_t j = gnrc_netdev->tx.last_tx_neighbor_id + 1;

        if (j >= GNRC_MAC_NEIGHBOR_COUNT) {
            j = 1;
        }

        for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {
            if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.neighbors[j].queue) > 0) {
                gnrc_netdev->tx.last_tx_neighbor_id = j;
                next = (int) j;
                break;
            }
            else {
                j++;
                if (j >= GNRC_MAC_NEIGHBOR_COUNT) {
                    j = 1;
                }
            }
        }
    }

    if (next >= 0) {
        gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.neighbors[next].queue);
        if (pkt != NULL) {
            gnrc_netdev->tx.packet = pkt;
            gnrc_netdev->tx.current_neighbor = &gnrc_netdev->tx.neighbors[next];
            gnrc_netdev->tx.tx_seq = 0;
            gnrc_netdev->tx.t2u_retry_counter = 0;
            return true;
        }
        else {
            return false;
        }
    }

    return false;
}

void gnrc_gomach_beacon_process(gnrc_netdev_t *gnrc_netdev, gnrc_pktsnip_t *pkt)
{
    assert(gnrc_netdev != NULL);
    assert(pkt != NULL);

    gnrc_gomach_frame_beacon_t *gomach_beacon_hdr = NULL;
    gnrc_pktsnip_t *gomach_snip = NULL;

    gnrc_gomach_l2_id_t *id_list;
    uint8_t *slots_list;
    uint8_t schedulelist_size = 0;
    bool got_allocated_slots;
    uint8_t id_position;
    uint8_t slots_position;

    gnrc_pktsnip_t *beacon_snip = gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_GOMACH);
    if (beacon_snip == NULL) {
        LOG_ERROR("[GOMACH]: No beacon-snip found in gnrc_gomach_beacon_process().\n");
        return;
    }
    else {
        gomach_beacon_hdr = beacon_snip->data;
    }

    if (gomach_beacon_hdr == NULL) {
        LOG_ERROR("ERROR: [GOMACH]: GoMacH's beacon header is null.\n");
        return;
    }

    schedulelist_size = gomach_beacon_hdr->schedulelist_size;
    gnrc_netdev->tx.vtdma_para.sub_channel_seq = gomach_beacon_hdr->sub_channel_seq;

    if (schedulelist_size == 0) {
        /* No allocated slots. */
        gnrc_netdev->tx.vtdma_para.slots_num = 0;
        gnrc_netdev->tx.vtdma_para.slots_position = 0;
        return;
    }

    /* Take the ID-list out. */
    gomach_snip = gnrc_pktbuf_mark(pkt, schedulelist_size * sizeof(gnrc_gomach_l2_id_t),
                                   GNRC_NETTYPE_GOMACH);
    id_list = gomach_snip->data;

    /* Take the slots-list out. */
    slots_list = pkt->data;

    /* Check whether this device has been allocated slots. */
    int i = 0;
    got_allocated_slots = false;
    id_position = 0;

    for (i = 0; i < schedulelist_size; i++) {
        if (memcmp(gnrc_netdev->l2_addr, id_list[i].addr, gnrc_netdev->l2_addr_len) == 0) {
            got_allocated_slots = true;
            id_position = i;
            break;
        }
    }

    if (got_allocated_slots == true) {
        /* Find the slots number and the related slots position. */
        gnrc_netdev->tx.vtdma_para.slots_num = slots_list[id_position];

        slots_position = 0;
        for (i = 0; i < id_position; i++) {
            slots_position += slots_list[i];
        }
        gnrc_netdev->tx.vtdma_para.slots_position = slots_position;
    }
    else {
        /* No allocated slots. */
        gnrc_netdev->tx.vtdma_para.slots_num = 0;
        gnrc_netdev->tx.vtdma_para.slots_position = 0;
    }
}

void gnrc_gomach_packet_process_in_wait_beacon(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_gomach_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* Parse the received packet. */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            LOG_DEBUG("[GOMACH] t2k: Packet could not be parsed: %i\n", res);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case GNRC_GOMACH_FRAME_BEACON: {
                if (memcmp(&gnrc_netdev->tx.current_neighbor->l2_addr,
                           &receive_packet_info.src_addr.addr,
                           gnrc_netdev->tx.current_neighbor->l2_addr_len) == 0) {
                    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON);
                    gnrc_gomach_beacon_process(gnrc_netdev, pkt);
                }
                gnrc_pktbuf_release(pkt);
                break;
            }
            case GNRC_GOMACH_FRAME_PREAMBLE: {
                /* Release preamble packet no matter the preamble is for it or not,
                 * and quit the t2k procedure. */
                gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                gnrc_pktbuf_release(pkt);
                break;
            }
            case GNRC_GOMACH_FRAME_DATA: {
                /* It is unlikely that we will received a data for us here.
                 * This means the device' CP is close with its destination's. */
                if (memcmp(&gnrc_netdev->l2_addr, &receive_packet_info.dst_addr.addr,
                           gnrc_netdev->l2_addr_len) == 0) {
                    gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                    if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                        gnrc_pktbuf_release(pkt);
                        LOG_DEBUG("[GOMACH]: received a duplicate packet.\n");
                        return;
                    }

                    gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                    gnrc_mac_dispatch(&gnrc_netdev->rx);
                }
                else {
                    gnrc_pktbuf_release(pkt);
                }
                break;
            }
            case GNRC_GOMACH_FRAME_BROADCAST: {
                gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                gnrc_pktbuf_release(pkt);
                break;
            }
            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }
    }
}

void gnrc_gomach_packet_process_in_vtdma(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    gnrc_pktsnip_t *pkt;
    gnrc_gomach_packet_info_t receive_packet_info;

    while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->rx.queue)) != NULL) {
        /* Parse the received packet. */
        int res = _parse_packet(pkt, &receive_packet_info);
        if (res != 0) {
            LOG_DEBUG("[GOMACH] vtdma: Packet could not be parsed: %i\n", res);
            gnrc_pktbuf_release(pkt);
            continue;
        }

        switch (receive_packet_info.header->type) {
            case GNRC_GOMACH_FRAME_DATA: {
                gnrc_gomach_indicator_update(gnrc_netdev, pkt, &receive_packet_info);

                if ((gnrc_gomach_check_duplicate(gnrc_netdev, &receive_packet_info))) {
                    gnrc_pktbuf_release(pkt);
                    LOG_DEBUG("[GOMACH] vtdma: received a duplicate packet.\n");
                    return;
                }

                gnrc_gomach_dispatch_defer(gnrc_netdev->rx.dispatch_buffer, pkt);
                gnrc_mac_dispatch(&gnrc_netdev->rx);
                break;
            }
            default: {
                gnrc_pktbuf_release(pkt);
                break;
            }
        }
    }
}

void gnrc_gomach_update_neighbor_phase(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if (gnrc_netdev->tx.neighbors[i].mac_type == GNRC_GOMACH_TYPE_KNOWN) {
            long int tmp = gnrc_netdev->tx.neighbors[i].cp_phase -
                           gnrc_netdev->gomach.backoff_phase_ticks;
            if (tmp < 0) {
                tmp += RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);

                /* Toggle the neighbor's public channel phase if tmp < 0. */
                if (gnrc_netdev->tx.neighbors[i].pub_chanseq ==
                    gnrc_netdev->gomach.pub_channel_1) {
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

void gnrc_gomach_update_neighbor_pubchan(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev != NULL);

    /* Toggle this device's current channel. */
    if (gnrc_netdev->gomach.cur_pub_channel == gnrc_netdev->gomach.pub_channel_1) {
        gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_2;
    }
    else {
        gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_1;
    }

    /* Toggle TX neighbors' current channel. */
    for (int i = 1; i < GNRC_MAC_NEIGHBOR_COUNT; i++) {
        if (gnrc_netdev->tx.neighbors[i].mac_type == GNRC_GOMACH_TYPE_KNOWN) {
            if (gnrc_netdev->tx.neighbors[i].pub_chanseq == gnrc_netdev->gomach.pub_channel_1) {
                gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_2;
            }
            else {
                gnrc_netdev->tx.neighbors[i].pub_chanseq = gnrc_netdev->gomach.pub_channel_1;
            }
        }
    }
}

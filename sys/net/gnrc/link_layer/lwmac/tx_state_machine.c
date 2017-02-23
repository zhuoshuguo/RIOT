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
 * @{
 *
 * @file
 * @brief       Implementation of TX state machine of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <periph/rtt.h>
#include <net/gnrc.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <random.h>

#include <net/gnrc/mac/internal.h>
#include <net/gnrc/lwmac/timeout.h>
#include "include/tx_state_machine.h"
#include "include/lwmac_internal.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define LOG_LEVEL LOG_WARNING
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [lwmac-tx] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [lwmac-tx] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "[lwmac-tx] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "[lwmac-tx] " __VA_ARGS__)

/* Break out of switch and mark the need for rescheduling */
#define GOTO_TX_STATE(tx_state, do_resched) gnrc_netdev2->tx.state = tx_state; \
                                            reschedule = do_resched; \
                                            break

void lwmac_tx_start(gnrc_netdev2_t* gnrc_netdev2, gnrc_pktsnip_t* pkt, gnrc_mac_tx_neighbor_t* neighbour)
{
    assert(gnrc_netdev2 != NULL);
    assert(pkt != NULL);
    assert(neighbour != NULL);

    if (gnrc_netdev2->tx.packet) {
        LOG_WARNING("Starting but tx.packet is still set\n");
        gnrc_pktbuf_release(gnrc_netdev2->tx.packet);
    }

    gnrc_netdev2->tx.packet = pkt;
    gnrc_netdev2->tx.current_neighbor = neighbour;
    gnrc_netdev2->tx.state = TX_STATE_INIT;
    gnrc_netdev2->tx.wr_sent = 0;

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
    gnrc_netdev2->lwmac.pkt_start_sending_time_ticks = rtt_get_counter();
#endif
}

void lwmac_tx_stop(gnrc_netdev2_t* gnrc_netdev2)
{
    if (!gnrc_netdev2) {
        return;
    }

    lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WR);
    lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NO_RESPONSE);
    lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NEXT_BROADCAST);
    lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_BROADCAST_END);
    gnrc_netdev2->tx.state = TX_STATE_STOPPED;

    /* Release packet in case of failure */
    if (gnrc_netdev2->tx.packet) {
        gnrc_pktbuf_release(gnrc_netdev2->tx.packet);
        gnrc_netdev2->tx.packet = NULL;
    }

    if (gnrc_netdev2->lwmac.extend_tx == false) {
    	gnrc_netdev2->tx.current_neighbor = NULL;
    }

}

/* Returns whether rescheduling is needed or not */
static bool _lwmac_tx_update(gnrc_netdev2_t* gnrc_netdev2)
{
    bool reschedule = false;

    if (!gnrc_netdev2) {
        return reschedule;
    }

    switch (gnrc_netdev2->tx.state)
    {
    case TX_STATE_INIT:
    {
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WR);
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NO_RESPONSE);
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NEXT_BROADCAST);
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_BROADCAST_END);

        /* if found ongoing transmission,
         * quit this cycle for collision avoidance. */
        if (_get_netdev_state(gnrc_netdev2) == NETOPT_STATE_RX){
            gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
            /* drop pointer so it wont be free'd */
            gnrc_netdev2->tx.packet = NULL;
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* check if the packet is for broadcast */
        if (gnrc_netif_hdr_get_flag(gnrc_netdev2->tx.packet) & (GNRC_NETIF_HDR_FLAGS_BROADCAST | GNRC_NETIF_HDR_FLAGS_MULTICAST)) {
            /* Set CSMA retries as configured and enable */
            uint8_t csma_retries = LWMAC_BROADCAST_CSMA_RETRIES;
            gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA_RETRIES,
                                           &csma_retries, sizeof(csma_retries));
            netopt_enable_t csma_enable = NETOPT_ENABLE;
            gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA, &csma_enable, sizeof(csma_enable));

            GOTO_TX_STATE(TX_STATE_SEND_BROADCAST, true);
        } else {
            /* Use CSMA for the first WR */
            netopt_enable_t csma_disable = NETOPT_ENABLE;
            gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA, &csma_disable, sizeof(csma_disable));

            GOTO_TX_STATE(TX_STATE_SEND_WR, true);
        }
    }
    case TX_STATE_SEND_BROADCAST:
    {
        gnrc_pktsnip_t* pkt = gnrc_netdev2->tx.packet;
        bool first = false;

        if (lwmac_timeout_is_running(&gnrc_netdev2->lwmac, TIMEOUT_BROADCAST_END)) {
            if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_BROADCAST_END)) {
                lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NEXT_BROADCAST);
                gnrc_pktbuf_release(pkt);
                gnrc_netdev2->tx.packet = NULL;
                GOTO_TX_STATE(TX_STATE_SUCCESSFUL, true);
            }
        } else {
            LOG_INFO("Initialize broadcasting\n");
            lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_BROADCAST_END, LWMAC_BROADCAST_DURATION_US);

            gnrc_pktsnip_t* pkt_payload;

            /* Prepare packet with LwMAC header*/
            lwmac_frame_broadcast_t hdr = {};
            hdr.header.type = FRAMETYPE_BROADCAST;
            hdr.seq_nr = gnrc_netdev2->tx.bcast_seqnr++;

            pkt_payload = pkt->next;
            pkt->next = gnrc_pktbuf_add(pkt->next, &hdr, sizeof(hdr), GNRC_NETTYPE_LWMAC);
            if (pkt->next == NULL) {
                LOG_ERROR("Cannot allocate pktbuf of type FRAMETYPE_BROADCAST\n");
                gnrc_netdev2->tx.packet->next = pkt_payload;
                gnrc_netdev2->lwmac.extend_tx = false;
                GOTO_TX_STATE(TX_STATE_FAILED, true);
            }

            /* No Auto-ACK for broadcast packets */
            netopt_enable_t autoack = NETOPT_DISABLE;
            gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

            first = true;
        }

        if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_NEXT_BROADCAST) ||
            first) {
            /* Don't let the packet be released yet, we want to send it again */
            gnrc_pktbuf_hold(pkt, 1);

            int res = gnrc_netdev2->send(gnrc_netdev2, pkt);
            if (res < 0){
                LOG_ERROR("Send broadcast pkt failed.");
                gnrc_netdev2->lwmac.extend_tx = false;
                GOTO_TX_STATE(TX_STATE_FAILED, true);
            }
            _set_netdev_state(gnrc_netdev2, NETOPT_STATE_TX);

            lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NEXT_BROADCAST, LWMAC_TIME_BETWEEN_BROADCAST_US);
            LOG_INFO("Broadcast sent\n");
        }

        break;
    }
    case TX_STATE_SEND_WR:
    {
        LOG_DEBUG("TX_STATE_SEND_WR\n");

        gnrc_pktsnip_t* pkt;
        gnrc_pktsnip_t* pkt_lwmac;
        gnrc_netif_hdr_t *nethdr;
        //uint8_t* dst_addr = NULL;
        //int addr_len;

        uint32_t random_backoff;
        random_backoff = random_uint32_range(0, LWMAC_RANDOM_BEFORE_WR_US);
        xtimer_usleep(random_backoff);

        /* if found ongoing transmission,
         * quit this cycle for collision avoidance. */
        if (_get_netdev_state(gnrc_netdev2) == NETOPT_STATE_RX) {
            gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
            /* drop pointer so it wont be free'd */
            gnrc_netdev2->tx.packet = NULL;
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* Get destination address
        addr_len = _get_dest_address(lwmac->tx.packet, &dst_addr);
        if(addr_len <= 0 || addr_len > 8) {
            LOG_ERROR("Invalid address length: %i\n", addr_len);
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }*/

        /* Assemble WR */
        lwmac_frame_wr_t wr_hdr = {};
        wr_hdr.header.type = FRAMETYPE_WR;
        //wr_hdr.dst_addr = gnrc_netdev2->tx.current_neighbor->l2_addr;
        memcpy(&(wr_hdr.dst_addr.addr), gnrc_netdev2->tx.current_neighbor->l2_addr, gnrc_netdev2->tx.current_neighbor->l2_addr_len);
        wr_hdr.dst_addr.len = gnrc_netdev2->tx.current_neighbor->l2_addr_len;

        pkt = gnrc_pktbuf_add(NULL, &wr_hdr, sizeof(wr_hdr), GNRC_NETTYPE_LWMAC);
        if (pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_LWMAC\n");
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* track the location of this lwmac_frame_wr_t header */
        pkt_lwmac = pkt;

        pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
        if (pkt == NULL) {
            LOG_ERROR("Cannot allocate pktbuf of type GNRC_NETTYPE_NETIF\n");
            gnrc_pktbuf_release(pkt_lwmac);
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* We wouldn't get here if adding the NETIF header had failed, so no
           sanity checks needed */
        nethdr = (gnrc_netif_hdr_t*) (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_NETIF))->data;

        /* Construct NETIF header and insert address for WR packet */
        gnrc_netif_hdr_init(nethdr, 0, 0);
        //gnrc_netif_hdr_set_dst_addr(nethdr, dst_addr, addr_len);

        /* Send WR as broadcast*/
        nethdr->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

        /* Disable Auto ACK */
        netopt_enable_t autoack = NETOPT_DISABLE;
        gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_AUTOACK, &autoack, sizeof(autoack));


#if 0
        /* First WR, try to catch wakeup phase */
        if ((gnrc_netdev2->tx.wr_sent == 0) && (gnrc_netdev2->lwmac.extend_tx == false)) {

            /* Calculate wakeup time */
            uint32_t wait_until;
            wait_until  = _phase_to_ticks(gnrc_netdev2->tx.current_neighbor->phase);
            wait_until -= RTT_US_TO_TICKS(LWMAC_WR_BEFORE_PHASE_US);

            /* This output blocks a long time and can prevent correct timing */
            LOG_DEBUG("Phase length:  %"PRIu32"\n", RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
            LOG_DEBUG("Wait until:    %"PRIu32"\n", wait_until);
            LOG_DEBUG("     phase:    %"PRIu32"\n", _ticks_to_phase(wait_until));
            LOG_DEBUG("Ticks to wait: %"PRIu32"\n", (long int)wait_until - rtt_get_counter());

            /* Wait until calculated wakeup time of destination */
            while (rtt_get_counter() < wait_until);
        }
#endif

        /* Prepare WR, this will discard any frame in the transceiver that has
         * possibly arrived in the meantime but we don't care at this point. */
        int res = gnrc_netdev2->send(gnrc_netdev2, pkt);
        if (res < 0){
            LOG_ERROR("Send WR failed.");
            if (pkt != NULL){
                gnrc_pktbuf_release(pkt);
            }
            gnrc_netdev2->lwmac.extend_tx = false;

            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }
        /* Trigger sending frame */
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_TX);

        /* Flush RX queue, TODO: maybe find a way without loosing RX packets */
        gnrc_priority_pktqueue_flush(&gnrc_netdev2->rx.queue);

        GOTO_TX_STATE(TX_STATE_WAIT_WR_SENT, false);
    }
    case TX_STATE_WAIT_WR_SENT:
    {
        LOG_DEBUG("TX_STATE_WAIT_WR_SENT\n");

        if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_UNDEF) {
            LOG_DEBUG("WR not yet completely sent\n");
            break;
        }

        if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_BUSY) {
            gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
            gnrc_netdev2->tx.packet = NULL;
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
            break;
        }

        if (gnrc_netdev2->tx.wr_sent == 0) {
            lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NO_RESPONSE, LWMAC_PREAMBLE_DURATION_US);
            /* Only the first WR use CSMA */
            netopt_enable_t csma_disable = NETOPT_DISABLE;
            gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA, &csma_disable, sizeof(csma_disable));
        }

        gnrc_netdev2->tx.wr_sent++;

        /* This is not needed anymore, because WRs are sent without CSMA/CA */
        /*
        if(lwmac->tx_feedback == TX_FEEDBACK_BUSY) {
            LOG_DEBUG("WR could not be sent, retry\n");
            GOTO_TX_STATE(TX_STATE_SEND_WR, true);
        }
        */

        /* Set timeout for next WR in case no WA will be received */
        lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WR, LWMAC_TIME_BETWEEN_WR_US);

        /* Debug WR timing */
        LOG_DEBUG("Destination phase was: %"PRIu32"\n", gnrc_netdev2->tx.current_neighbor->phase);
        LOG_DEBUG("Phase when sent was:   %"PRIu32"\n", _ticks_to_phase(gnrc_netdev2->tx.timestamp));
        LOG_DEBUG("Ticks when sent was:   %"PRIu32"\n", gnrc_netdev2->tx.timestamp);

        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_IDLE);
        GOTO_TX_STATE(TX_STATE_WAIT_FOR_WA, false);
    }
    case TX_STATE_WAIT_FOR_WA:
    {
        LOG_DEBUG("TX_STATE_WAIT_FOR_WA\n");

        gnrc_pktsnip_t* pkt;
        bool found_wa = false;
        bool postponed = false;
        bool from_expected_destination = false;

        if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_NO_RESPONSE)) {
            LOG_DEBUG("No response from destination\n");
            gnrc_netdev2->lwmac.extend_tx = false;
            gnrc_netdev2->lwmac.quit_tx = true;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_WR)) {

            if (gnrc_netdev2->lwmac.extend_tx == true) {
                puts("tx burst fail");
                gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
                /* drop pointer so it wont be free'd */
                gnrc_netdev2->tx.packet = NULL;

                GOTO_TX_STATE(TX_STATE_FAILED, true);
            } else {
                GOTO_TX_STATE(TX_STATE_SEND_WR, true);
            }
        }

        if (_get_netdev_state(gnrc_netdev2) == NETOPT_STATE_RX) {
            LOG_WARNING("Wait for completion of frame reception\n");
            break;
        }

        while ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev2->rx.queue)) != NULL)
        {
            LOG_DEBUG("Inspecting pkt @ %p\n", pkt);

            /* Parse packet */
            lwmac_packet_info_t info;
            int ret = _parse_packet(pkt, &info);

            if (ret != 0) {
                LOG_DEBUG("Packet could not be parsed: %i\n", ret);
                gnrc_pktbuf_release(pkt);
                continue;
            }

            if (memcmp(&info.src_addr.addr, &gnrc_netdev2->tx.current_neighbor->l2_addr, gnrc_netdev2->tx.current_neighbor->l2_addr_len) == 0) {
                from_expected_destination = true;
            }

            if (info.header->type == FRAMETYPE_BROADCAST) {
                _dispatch_defer(gnrc_netdev2->rx.dispatch_buffer, pkt);
                /* Drop pointer to it can't get released */
                pkt = NULL;
            }

            /* Check if destination is talking to another node. It will sleep
             * after a finished transaction so there's no point in trying any
             * further now. */
            if (!(memcmp(&info.dst_addr.addr, &gnrc_netdev2->l2_addr, gnrc_netdev2->l2_addr_len) == 0) &&
                from_expected_destination) {
                gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
                /* drop pointer so it wont be free'd */
                gnrc_netdev2->tx.packet = NULL;
                postponed = true;
                gnrc_pktbuf_release(pkt);
                break;
            }

            if (info.header->type == FRAMETYPE_BROADCAST) {
                continue;
            }

            /* if found anther node is also trying to send data,
             * quit this cycle for collision avoidance. */
            if (info.header->type == FRAMETYPE_WR){
                gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, gnrc_netdev2->tx.packet);
                /* drop pointer so it wont be free'd */
                gnrc_netdev2->tx.packet = NULL;
                postponed = true;
                gnrc_pktbuf_release(pkt);
                break;
            }

            if (info.header->type != FRAMETYPE_WA) {
                LOG_DEBUG("Packet is not WA: 0x%02x\n", info.header->type);
                gnrc_pktbuf_release(pkt);
                continue;
            }

            /* calculate the phase of the receiver based on WA */
            gnrc_netdev2->tx.timestamp = _phase_now();
            lwmac_frame_wa_t* wa_hdr;
            wa_hdr = (gnrc_pktsnip_search_type(pkt, GNRC_NETTYPE_LWMAC))->data;

            if (gnrc_netdev2->tx.timestamp >= wa_hdr->current_phase){
                gnrc_netdev2->tx.timestamp = gnrc_netdev2->tx.timestamp - wa_hdr->current_phase;
            } else {
                gnrc_netdev2->tx.timestamp += RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US);
                gnrc_netdev2->tx.timestamp -= wa_hdr->current_phase;
            }

            /* No need to keep pkt anymore */
            gnrc_pktbuf_release(pkt);

            if (!from_expected_destination) {
                LOG_DEBUG("Packet is not from expected destination\n");
                break;
            }

            /* All checks passed so this must be a valid WA */
            lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WR);
            lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_NO_RESPONSE);
            found_wa = true;
            break;
        }

        if (postponed) {
            gnrc_netdev2->lwmac.quit_tx = true;
        	gnrc_netdev2->lwmac.extend_tx = false;
            LOG_INFO("Destination is talking to another node, postpone\n");
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        if (!found_wa) {
            LOG_DEBUG("No WA yet\n");
            break;
        }

        /* Save newly calculated phase for destination */
        gnrc_netdev2->tx.current_neighbor->phase = gnrc_netdev2->tx.timestamp;
        LOG_INFO("New phase: %"PRIu32"\n", gnrc_netdev2->tx.timestamp);

        /* We've got our WA, so discard the rest, TODO: no flushing */
        gnrc_priority_pktqueue_flush(&gnrc_netdev2->rx.queue);

        GOTO_TX_STATE(TX_STATE_SEND_DATA, true);
    }
    case TX_STATE_SEND_DATA:
    {
        LOG_DEBUG("TX_STATE_SEND_DATA\n");

        gnrc_pktsnip_t* pkt = gnrc_netdev2->tx.packet;
        gnrc_pktsnip_t* pkt_payload;

        /* Enable Auto ACK again */
        netopt_enable_t autoack = NETOPT_ENABLE;
        gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

        /* It's okay to retry sending DATA. Timing doesn't matter anymore and
         * destination is waiting for a certain amount of time. */
        uint8_t csma_retries = LWMAC_DATA_CSMA_RETRIES;
        gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA_RETRIES, &csma_retries, sizeof(csma_retries));

        netopt_enable_t csma_enable = NETOPT_ENABLE;
        gnrc_netdev2->dev->driver->set(gnrc_netdev2->dev, NETOPT_CSMA, &csma_enable, sizeof(csma_enable));

        pkt_payload = pkt->next;

        /* Insert lwMAC header above NETIF header */
        lwmac_hdr_t hdr;
        if ((gnrc_priority_pktqueue_length(&gnrc_netdev2->tx.current_neighbor->queue) > 0) && (gnrc_netdev2->lwmac.max_tx_num < LWMAC_MAX_TX_BURST_PKT_NUM)) {
            hdr.type = FRAMETYPE_DATA_PENDING;
            gnrc_netdev2->lwmac.extend_tx = true;
            gnrc_netdev2->lwmac.max_tx_num ++;
        } else {
            hdr.type = FRAMETYPE_DATA;
            gnrc_netdev2->lwmac.extend_tx = false;
            gnrc_netdev2->lwmac.quit_tx = true;

        }

        pkt->next = gnrc_pktbuf_add(pkt->next, &hdr, sizeof(hdr), GNRC_NETTYPE_LWMAC);
        if (pkt->next == NULL){
            LOG_ERROR("Cannot allocate pktbuf of type FRAMETYPE_DATA\n");
            gnrc_netdev2->tx.packet->next = pkt_payload;
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        /* Send data */
        int res = gnrc_netdev2->send(gnrc_netdev2, pkt);
        if (res < 0){
            LOG_ERROR("Send data failed.");
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_TX);

        /* Packet has been released by netdev, so drop pointer */
        gnrc_netdev2->tx.packet = NULL;

        DEBUG("[lwmac-tx]: spent %lu WR in TX\n", gnrc_netdev2->tx.wr_sent);

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
        gnrc_netdev2->lwmac.pkt_start_sending_time_ticks = rtt_get_counter() - gnrc_netdev2->lwmac.pkt_start_sending_time_ticks;
        DEBUG("[lwmac-tx]: pkt sending delay in TX: %lu us\n", RTT_TICKS_TO_US(gnrc_netdev2->lwmac.pkt_start_sending_time_ticks));
#endif

        GOTO_TX_STATE(TX_STATE_WAIT_FEEDBACK, false);
    }
    case TX_STATE_WAIT_FEEDBACK:
    {
        LOG_DEBUG("TX_STATE_WAIT_FEEDBACK\n");
        if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_UNDEF) {
            break;
        } else if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_SUCCESS) {
            GOTO_TX_STATE(TX_STATE_SUCCESSFUL, true);
        } else if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_NOACK) {
            LOG_ERROR("Not ACKED\n");
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        } else if (gnrc_netdev2_get_tx_feedback(gnrc_netdev2) == TX_FEEDBACK_BUSY) {
            LOG_ERROR("Channel busy \n");
            gnrc_netdev2->lwmac.extend_tx = false;
            GOTO_TX_STATE(TX_STATE_FAILED, true);
        }

        LOG_ERROR("Tx feedback unhandled: %i\n", gnrc_netdev2_get_tx_feedback(gnrc_netdev2));
        gnrc_netdev2->lwmac.extend_tx = false;
        GOTO_TX_STATE(TX_STATE_FAILED, true);
    }
    case TX_STATE_SUCCESSFUL:
    case TX_STATE_FAILED:
        break;
    case TX_STATE_STOPPED:
        LOG_DEBUG("Transmission state machine is stopped\n");
    }

    return reschedule;
}

void lwmac_tx_update(gnrc_netdev2_t* gnrc_netdev2)
{
    /* Update until no rescheduling needed */
    while (_lwmac_tx_update(gnrc_netdev2));
}

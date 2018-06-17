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
 * @brief       Implementation of GoMacH
 *
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "kernel_types.h"
#include "msg.h"
#include "thread.h"
#include "random.h"
#include "timex.h"
#include "periph/rtt.h"
#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/mac/internal.h"
#include "net/gnrc/netdev/ieee802154.h"
#include "net/gnrc/gomach/gomach.h"
#include "net/gnrc/gomach/types.h"
#include "net/gnrc/gomach/timeout.h"
#include "include/gomach_internal.h"

extern uint64_t ccn_round_time_sum;
extern uint32_t ccn_request_count;

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef LOG_LEVEL
/**
 * @brief Default log level define
 */
#define LOG_LEVEL LOG_WARNING
#endif

#include "log.h"

/**
 * @brief  GoMacH thread's PID
 */
static kernel_pid_t gomach_pid;

static void gomach_reinit_radio(gnrc_netdev_t *gnrc_netdev)
{
    /* Initialize low-level driver. */
    gnrc_netdev->dev->driver->init(gnrc_netdev->dev);

    /* Set MAC address length. */
    uint16_t src_len = gnrc_netdev->l2_addr_len;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_SRC_LEN, &src_len, sizeof(src_len));

    /* Set the MAC address of the device. */
    if (gnrc_netdev->l2_addr_len == IEEE802154_LONG_ADDRESS_LEN) {
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                      NETOPT_ADDRESS_LONG,
                                      gnrc_netdev->l2_addr,
                                      sizeof(gnrc_netdev->l2_addr));
    }
    else {
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev,
                                      NETOPT_ADDR_LEN,
                                      gnrc_netdev->l2_addr,
                                      sizeof(gnrc_netdev->l2_addr));
    }

    /* Enable RX-start and TX-started and TX-END interrupts. */
    netopt_enable_t enable = NETOPT_ENABLE;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RX_END_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

}

static void gomach_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Get the MAC address of the device. */
    gnrc_netdev->l2_addr_len = gnrc_netdev->dev->driver->get(gnrc_netdev->dev,
                                                             NETOPT_ADDRESS_LONG,
                                                             gnrc_netdev->l2_addr,
                                                             sizeof(gnrc_netdev->l2_addr));

    /* Initialize GoMacH's state machines. */
    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_INIT;
    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_PREPARE;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_INIT;
    gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;
    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_INIT;
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_INIT;
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;

    /* Initialize GoMacH's channels. */
    gnrc_netdev->gomach.sub_channel_seq = 20;
    gnrc_netdev->gomach.pub_channel_1 = 26;
    gnrc_netdev->gomach.pub_channel_2 = 11;
    gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_1;
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.cur_pub_channel);

    /* Enable RX-start and TX-started and TX-END interrupts. */
    netopt_enable_t enable = NETOPT_ENABLE;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Initialize broadcast sequence number. This at least differs from board
     * to board. */
    gnrc_netdev->tx.broadcast_seq = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 1];

    /* Reset all timeouts just to be sure. */
    gnrc_gomach_reset_timeouts(gnrc_netdev);

    /* Initialize GoMacH's other key parameters. */
    gnrc_netdev->tx.no_ack_counter = 0;
    gnrc_gomach_set_enter_new_cycle(gnrc_netdev, false);
    gnrc_netdev->rx.vtdma_manag.sub_channel_seq = 26;
    gnrc_netdev->gomach.subchannel_occu_flags = 0;
    gnrc_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_gomach_set_update(gnrc_netdev, false);
    gnrc_gomach_set_duty_cycle_start(gnrc_netdev, false);
    gnrc_gomach_set_quit_cycle(gnrc_netdev, false);

    gnrc_gomach_set_beacon_fail(gnrc_netdev, false);
    gnrc_gomach_set_buffer_full(gnrc_netdev, false);
    gnrc_gomach_set_phase_backoff(gnrc_netdev, false);
    gnrc_netdev->rx.check_dup_pkt.queue_head = 0;
    gnrc_netdev->tx.last_tx_neighbor_id = 0;

    netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
    device_state->seq = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 1];

    /* Initialize GoMacH's duplicate-check scheme. */
    for (int i = 0; i < GNRC_GOMACH_DUPCHK_BUFFER_SIZE; i++) {
        gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
    }

    /* Set the random seed. */
    uint32_t seed = 0;
    seed = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 2];
    seed = seed << 8;
    seed |= gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 1];
    random_init(seed);

    gnrc_netdev->tx.t2u_fail_count = 0;

#if (GNRC_GOMACH_ENABLE_DUTYCYLE_RECORD == 1)
    /* Start duty cycle recording */
    gnrc_netdev->gomach.system_start_time_ticks = xtimer_now_usec64();
    gnrc_netdev->gomach.last_radio_on_time_ticks = gnrc_netdev->gomach.system_start_time_ticks;
    gnrc_netdev->gomach.awake_duration_sum_ticks = 0;
    gnrc_netdev->gomach.gomach_info |= GNRC_GOMACH_INTERNAL_INFO_RADIO_IS_ON;
#endif

    gnrc_netdev->tx.get_bcast_pkt = false;
    gnrc_netdev->tx.get_bcast_pkt_time = xtimer_now_usec();

    ccn_round_time_sum = 0;
    ccn_request_count = 0;
}

static void _gomach_rtt_cb(void *arg)
{
    msg_t msg;

    msg.content.value = ((uint32_t) arg) & 0xffff;
    msg.type = GNRC_GOMACH_EVENT_RTT_TYPE;
    msg_send(&msg, gomach_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

static void _gomach_rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    switch (event & 0xffff) {
        case GNRC_GOMACH_EVENT_RTT_NEW_CYCLE: {
            /* Start duty-cycle scheme. */
            if (!gnrc_gomach_get_duty_cycle_start(gnrc_netdev)) {
                gnrc_gomach_set_duty_cycle_start(gnrc_netdev, true);
                rtt_clear_alarm();
                /* Record the new cycle's starting time. */
                gnrc_netdev->gomach.last_wakeup = rtt_get_counter();
            }
            else {
                /* The duty-cycle scheme has already started,
                 * record the new cycle's starting time. */
                gnrc_netdev->gomach.last_wakeup = rtt_get_alarm();
                gnrc_gomach_set_enter_new_cycle(gnrc_netdev, true);
            }

            gnrc_netdev->gomach.last_wakeup_phase_ms = xtimer_now_usec64();

            /* Set next cycle's starting time. */
            uint32_t alarm = gnrc_netdev->gomach.last_wakeup +
                             RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);
            rtt_set_alarm(alarm, _gomach_rtt_cb, (void *) GNRC_GOMACH_EVENT_RTT_NEW_CYCLE);

            /* Update neighbors' public channel phases. */
            gnrc_gomach_update_neighbor_pubchan(gnrc_netdev);
            gnrc_gomach_set_update(gnrc_netdev, true);
        } break;
        default: {
            LOG_ERROR("ERROR: [GOMACH] error RTT message type\n");
            break;
        }
    }
}

static void gomach_bcast_init(gnrc_netdev_t *gnrc_netdev)
{
	puts("B");
    /* Disable auto-ACK when sending broadcast packets, thus not to receive packet. */
    gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

    /* Firstly turn the radio to public channel 1. */
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
    gnrc_gomach_set_on_pubchan_1(gnrc_netdev, true);

    gnrc_netdev->tx.broadcast_seq++;

    /* Assemble the broadcast packet. */
    gnrc_pktsnip_t *pkt = gnrc_netdev->tx.packet;
    gnrc_pktsnip_t *payload = gnrc_netdev->tx.packet->next;

    gnrc_gomach_frame_broadcast_t gomach_broadcast_hdr;
    gomach_broadcast_hdr.header.type = GNRC_GOMACH_FRAME_BROADCAST;
    gomach_broadcast_hdr.seq_nr = gnrc_netdev->tx.broadcast_seq;
    pkt->next = gnrc_pktbuf_add(pkt->next, &gomach_broadcast_hdr,
                                sizeof(gomach_broadcast_hdr),
                                GNRC_NETTYPE_GOMACH);
    if (pkt->next == NULL) {
        /* Make append payload after netif header again */
        gnrc_netdev->tx.packet->next = payload;
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
        LOG_ERROR("ERROR: [GOMACH] bcast: no memory to assemble bcast packet, drop packet.\n");
        LOG_ERROR("ERROR: [GOMACH] bcast failed, go to listen mode.\n");
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_FINISH,
                            GNRC_GOMACH_SUPERFRAME_DURATION_US);

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_SEND;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static bool _gomach_send_bcast_busy_handle(gnrc_netdev_t *gnrc_netdev)
{
    /* Quit sending broadcast packet if we found ongoing transmissions, for collision avoidance. */
    if ((gnrc_gomach_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) ||
        (gnrc_netdev_get_rx_started(gnrc_netdev) == true)) {
        LOG_DEBUG("[GOMACH] bcast: found ongoing transmission, quit broadcast.\n");
        /* Queue the broadcast packet back to the queue. */
        gnrc_pktsnip_t *payload = gnrc_netdev->tx.packet->next->next;

        /* remove gomach header */
        gnrc_netdev->tx.packet->next->next = NULL;
        gnrc_pktbuf_release(gnrc_netdev->tx.packet->next);

        /* make append payload after netif header again */
        gnrc_netdev->tx.packet->next = payload;

        if (!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, gnrc_netdev->tx.packet)) {
            LOG_DEBUG("[GOMACH] bcast: TX queue full, release packet.\n");
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        }
        gnrc_netdev->tx.packet = NULL;

        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return false;
    }
    return true;
}

static void gomach_send_bcast_packet(gnrc_netdev_t *gnrc_netdev)
{
    /* Quit sending broadcast packet if we found ongoing transmissions, for collision avoidance. */
    if (!_gomach_send_bcast_busy_handle(gnrc_netdev)) {
        return;
    }

    gnrc_pktbuf_hold(gnrc_netdev->tx.packet, 1);

    /* Start sending the broadcast packet. */
    gnrc_gomach_send(gnrc_netdev, gnrc_netdev->tx.packet, NETOPT_DISABLE);

    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_WAIT_TX_FINISH;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_wait_bcast_tx_finish(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL,
                                GNRC_GOMACH_BCAST_INTERVAL_US);
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_WAIT_NEXT_TX;
        gnrc_gomach_set_update(gnrc_netdev, false);
    }

    /* This is to handle no-TX-complete issue. In case there is no no-TX-complete event,
     * we will quit broadcasting, i.e., not getting stucked here. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_FINISH)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL);
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_wait_bcast_wait_next_tx(gnrc_netdev_t *gnrc_netdev)
{
    /* Quit sending broadcast packet if we found ongoing transmissions, for collision avoidance. */
    if (!_gomach_send_bcast_busy_handle(gnrc_netdev)) {
        return;
    }

    /* If the whole broadcast duration timeouts, release the packet and go to t2u end. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_FINISH)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL);
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    /* Toggle the radio channel and go to send the next broadcast packet. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL)) {
        if (gnrc_gomach_get_on_pubchan_1(gnrc_netdev)) {
            gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_2);
            gnrc_gomach_set_on_pubchan_1(gnrc_netdev, false);
        }
        else {
            gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
            gnrc_gomach_set_on_pubchan_1(gnrc_netdev, true);
        }

        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_SEND;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_bcast_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_BCAST_FINISH);

    if (gnrc_netdev->tx.packet) {
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
    }
    gnrc_netdev->tx.current_neighbor = NULL;

    /* Reset the t2u state. */
    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_INIT;

    /* Switch to the listen mode. */
    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_gomach_set_enter_new_cycle(gnrc_netdev, false);
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_bcast_update(gnrc_netdev_t *gnrc_netdev)
{
    /* State machine of GoMacH's broadcast procedure. */
    switch (gnrc_netdev->tx.bcast_state) {
        case GNRC_GOMACH_BCAST_INIT: {
            gomach_bcast_init(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_BCAST_SEND: {
            gomach_send_bcast_packet(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_BCAST_WAIT_TX_FINISH: {
            gomach_wait_bcast_tx_finish(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_BCAST_WAIT_NEXT_TX: {
            gomach_wait_bcast_wait_next_tx(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_BCAST_END: {
            gomach_bcast_end(gnrc_netdev);
            break;
        }
        default: break;
    }
}

static void gomach_init_prepare(gnrc_netdev_t *gnrc_netdev)
{
    rtt_clear_alarm();

    /* Random delay for avoiding the same wake-up phase among devices. */
    uint32_t random_backoff = random_uint32_range(0, GNRC_GOMACH_SUPERFRAME_DURATION_US);
    xtimer_usleep(random_backoff);

    gnrc_gomach_set_quit_cycle(gnrc_netdev, false);
    gnrc_netdev->gomach.subchannel_occu_flags = 0;

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    /* Since devices don't broadcast beacons on default, so no need to collect beacons.
     * Go to announce its chosen sub-channel sequence. */
    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_ANNC_SUBCHAN;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_init_announce_subchannel(gnrc_netdev_t *gnrc_netdev)
{
    /* Choose a sub-channel for the device. */
    gnrc_gomach_init_choose_subchannel(gnrc_netdev);

    /* Announce the device's chosen sub-channel sequence to its neighbors. */
    gnrc_gomach_bcast_subchann_seq(gnrc_netdev, NETOPT_ENABLE);

    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_WAIT_FEEDBACK;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_init_wait_announce_feedback(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_init_end(gnrc_netdev_t *gnrc_netdev)
{
    /* Reset initialization state. */
    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_PREPARE;
    /* Switch to duty-cycle listen mode. */
    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_INIT;

    /* Start duty-cycle scheme. */
    gnrc_gomach_set_duty_cycle_start(gnrc_netdev, false);
    _gomach_rtt_handler(GNRC_GOMACH_EVENT_RTT_NEW_CYCLE, gnrc_netdev);
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_t2k_init(gnrc_netdev_t *gnrc_netdev)
{
	puts("k");
    /* Turn off radio to conserve power */
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);

    gnrc_gomach_set_quit_cycle(gnrc_netdev, false);

    /* Set waiting timer for the targeted device! */
    long int wait_phase_duration = gnrc_netdev->tx.current_neighbor->cp_phase -
                                   gnrc_gomach_phase_now(gnrc_netdev);

    if (wait_phase_duration < 0) {
     	wait_phase_duration += GNRC_GOMACH_SUPERFRAME_DURATION_US;
    }

    /* Upon several times of t2k failure, we now doubt that the phase-lock may fail due to drift.
     * Here is the phase-lock auto-adjust scheme, trying to catch the neighbot's phase in case of
     * phase-lock failure due to timer drift.
     * Firstly, put the calculated phase ahead, check whether the neighbor's phase has gone ahead
     * of the recorded one */
    if (gnrc_netdev->tx.no_ack_counter == (GNRC_GOMACH_REPHASELOCK_THRESHOLD - 2)) {
        if (wait_phase_duration < GNRC_GOMACH_CP_DURATION_US) {
            wait_phase_duration = (wait_phase_duration + GNRC_GOMACH_SUPERFRAME_DURATION_US) -
                                  GNRC_GOMACH_CP_DURATION_US;
        }
        else {
            wait_phase_duration = wait_phase_duration - GNRC_GOMACH_CP_DURATION_US;
        }
    }
    /* If this is the last t2k trial, the phase-lock auto-adjust scheme delays the estimated phase
     *  a little bit, to see if the real phase is behind the original calculated one. */
    if (gnrc_netdev->tx.no_ack_counter == (GNRC_GOMACH_REPHASELOCK_THRESHOLD - 1)) {
        wait_phase_duration = wait_phase_duration + GNRC_GOMACH_CP_DURATION_US;
        if (wait_phase_duration > GNRC_GOMACH_SUPERFRAME_DURATION_US) {
            wait_phase_duration = wait_phase_duration - GNRC_GOMACH_SUPERFRAME_DURATION_US;
        }
    }

    if (wait_phase_duration > GNRC_GOMACH_SUPERFRAME_DURATION_US) {
        wait_phase_duration = wait_phase_duration % GNRC_GOMACH_SUPERFRAME_DURATION_US;
    }
    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_CP, (uint32_t)wait_phase_duration);

    /* Flush the rx-queue. */
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    gnrc_netdev->tx.tx_busy_count = 0;

    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_CP;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_t2k_wait_cp(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_CP)) {
    	gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
    	/* Turn radio onto the neighbor's public channel, which will not change in this cycle. */
    	gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->tx.current_neighbor->pub_chanseq);

        /* Disable auto-ack, don't try to receive packet! */
        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);
        /* Require ACK for the packet waiting to be sent! */
        gnrc_gomach_set_ack_req(gnrc_netdev, NETOPT_ENABLE);

        /* Enable csma for sending the packet! */
        netopt_enable_t csma_enable = NETOPT_ENABLE;
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_CSMA, &csma_enable,
                                      sizeof(netopt_enable_t));


        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_TRANS_IN_CP;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_t2k_trans_in_cp(gnrc_netdev_t *gnrc_netdev)
{
    /* To-do: should we add a rx-start security check and quit t2k when found
     * ongoing transmissions? */

    /* If we are retransmitting the packet, use the same sequence number for the
     * packet to avoid duplicate packet reception at the receiver side. */
    if ((gnrc_netdev->tx.no_ack_counter > 0) || (gnrc_netdev->tx.tx_busy_count > 0)) {
        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        device_state->seq = gnrc_netdev->tx.tx_seq;
    }

    /* Send the data packet here. */
    int res = gnrc_gomach_send_data(gnrc_netdev, NETOPT_ENABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2k transmission fail: %d, drop packet.\n", res);
        gnrc_netdev->tx.no_ack_counter = 0;

        /* If res is < 0, the data packet will not be released in send().
         * so need to release the data here. */
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
        }

        gnrc_netdev->tx.current_neighbor = NULL;
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR,
                            GNRC_GOMACH_NO_TX_ISR_US);

    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_CPTX_FEEDBACK;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_t2k_wait_cp_txfeedback(gnrc_netdev_t *gnrc_netdev)
{
    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR))) {
        /* No TX-ISR, go to sleep. */
        gnrc_netdev->tx.no_ack_counter++;

        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        gnrc_netdev->tx.tx_seq = device_state->seq - 1;

        if (gnrc_netdev->tx.no_ack_counter >= GNRC_GOMACH_REPHASELOCK_THRESHOLD) {
            LOG_DEBUG("[GOMACH] t2k failed, go to t2u.\n");
            /* Here, we don't queue the packet again, but keep it in tx.packet. */
            gnrc_netdev->tx.current_neighbor->mac_type = GNRC_GOMACH_TYPE_UNKNOWN;
            gnrc_netdev->tx.t2u_retry_counter = 0;
        }

        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);
        return;
    }

    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
    	gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

        switch (gnrc_netdev_get_tx_feedback(gnrc_netdev)) {
            case TX_FEEDBACK_SUCCESS: {
                /* Since the packet will not be released by the sending function,
                 * so, here, if TX success, we first release the packet. */
                gnrc_pktbuf_release(gnrc_netdev->tx.packet);
                gnrc_netdev->tx.packet = NULL;

                /* Here is the phase-lock auto-adjust scheme. Use the new adjusted
                 * phase upon success. Here the new phase will be put ahead to the
                 * original phase. */
                if (gnrc_netdev->tx.no_ack_counter == (GNRC_GOMACH_REPHASELOCK_THRESHOLD - 2)) {
                    if (gnrc_netdev->tx.current_neighbor->cp_phase >=
                        GNRC_GOMACH_CP_DURATION_US) {
                        gnrc_netdev->tx.current_neighbor->cp_phase -=
                            GNRC_GOMACH_CP_DURATION_US;
                    }
                    else {
                        gnrc_netdev->tx.current_neighbor->cp_phase +=
                            GNRC_GOMACH_SUPERFRAME_DURATION_US;
                        gnrc_netdev->tx.current_neighbor->cp_phase -=
                            GNRC_GOMACH_CP_DURATION_US;
                    }
                }
                /* Here is the phase-lock auto-adjust scheme. Use the new adjusted
                 * phase upon success. Here the new phase will be put behind the original
                 * phase. */
                if (gnrc_netdev->tx.no_ack_counter == (GNRC_GOMACH_REPHASELOCK_THRESHOLD - 1)) {
                    gnrc_netdev->tx.current_neighbor->cp_phase +=
                        (GNRC_GOMACH_CP_DURATION_US + 20 * US_PER_MS);

                    if (gnrc_netdev->tx.current_neighbor->cp_phase >=
                        GNRC_GOMACH_SUPERFRAME_DURATION_US) {
                        gnrc_netdev->tx.current_neighbor->cp_phase -=
                            GNRC_GOMACH_SUPERFRAME_DURATION_US;
                    }
                }

                gnrc_netdev->tx.no_ack_counter = 0;
                gnrc_netdev->tx.t2u_fail_count = 0;

                /* If has pending packets, join the vTDMA period, first wait for receiver's beacon. */
                if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0) {
                    gnrc_netdev->tx.vtdma_para.slots_num = 0;
                    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON,
                                            GNRC_GOMACH_WAIT_BEACON_TIME_US);
                    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_BEACON;
                }
                else {
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case TX_FEEDBACK_BUSY:
                /* If the channel busy counter is below threshold, retry CSMA immediately,
                 * by knowing that the CP will be automatically extended. */
                if (gnrc_netdev->tx.tx_busy_count < GNRC_GOMACH_TX_BUSY_THRESHOLD) {
                    gnrc_netdev->tx.tx_busy_count++;

                    /* Store the TX sequence number for this packet. Always use the same
                     * sequence number for sending the same packet, to avoid duplicated
                     * packet reception at the receiver. */
                    netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                    gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_TRANS_IN_CP;
                    gnrc_gomach_set_update(gnrc_netdev, true);
                    return;
                }
            case TX_FEEDBACK_NOACK:
            default: {
                gnrc_netdev->tx.no_ack_counter++;

                LOG_DEBUG("[GOMACH] t2k %d times No-ACK.\n", gnrc_netdev->tx.no_ack_counter);

                /* This packet will be retried. Store the TX sequence number for this packet.
                 * Always use the same sequence number for sending the same packet. */
                netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                /* If no_ack_counter reaches the threshold, regarded as phase-lock failed. So
                 * retry to send the packet in t2u, i.e., try to phase-lock with the receiver
                 * again. */
                if (gnrc_netdev->tx.no_ack_counter >= GNRC_GOMACH_REPHASELOCK_THRESHOLD) {
                    LOG_DEBUG("[GOMACH] t2k failed, go to t2u.\n");
                    /* Here, we don't queue the packet again, but keep it in tx.packet. */
                    gnrc_netdev->tx.current_neighbor->mac_type = GNRC_GOMACH_TYPE_UNKNOWN;
                    gnrc_netdev->tx.t2u_retry_counter = 0;
                }
                else {
                    /* If no_ack_counter is below the threshold, retry sending the packet in t2k
                     * procedure in the following cycle. */

                }
                gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
        }
    }
}

static void gomach_t2k_wait_beacon(gnrc_netdev_t *gnrc_netdev)
{
    /* Process the beacon if we receive it. */
    if (gnrc_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_gomach_set_pkt_received(gnrc_netdev, false);
        gnrc_gomach_packet_process_in_wait_beacon(gnrc_netdev);
    }

    /* If we need to quit t2k, don't release the current neighbor pointer. In the
     * next cycle, we will try to send to the same receiver. */
    if (gnrc_gomach_get_quit_cycle(gnrc_netdev)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON);
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    if (gnrc_netdev->tx.vtdma_para.slots_num > 0) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON);

        /* If the sender gets allocated slots, go to attend the receiver's vTDMA for
         * burst sending all the pending packets to the receiver. */
        if (gnrc_netdev->tx.vtdma_para.slots_num > 0) {
            /* Switch the radio to the sub-channel of the receiver. */
            gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->tx.vtdma_para.sub_channel_seq);

            /* If the allocated slots period is not right behind the beacon, i.e., not the first
             * one, turn off the radio and wait for its own slots period. */
            if (gnrc_netdev->tx.vtdma_para.slots_position > 0) {
                gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);

                uint32_t wait_slots_duration = gnrc_netdev->tx.vtdma_para.slots_position *
                                               GNRC_GOMACH_VTDMA_SLOT_SIZE_US;
                gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_SLOTS,
                                        wait_slots_duration);

                gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_SLOTS;
                gnrc_gomach_set_update(gnrc_netdev, true);
            }
            else {
                /* If the allocated slots period is the first one in vTDMA,
                 * start sending packets. */
                gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&(gnrc_netdev->tx.current_neighbor->queue));
                if (pkt != NULL) {
                    gnrc_netdev->tx.packet = pkt;
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
                }
                else {
                    LOG_ERROR("ERROR: [GOMACH] t2k vTDMA: null packet.\n");
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
            }
        }
        else {
            /* No slots get allocated, go to t2k end. */
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
            gnrc_gomach_set_update(gnrc_netdev, true);
        }
        return;
    }

    /* If no beacon during waiting period, go to t2k end.
     * Or, if we have received beacon, but find no allocated slots,
     * go to t2k as well. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON) ||
        !gnrc_gomach_timeout_is_running(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        LOG_DEBUG("[GOMACH] t2k: no beacon.\n");
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_t2k_wait_own_slots(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_SLOTS)) {
        /* The node is now in its scheduled slots period, start burst sending packets. */
        gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);

        gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&(gnrc_netdev->tx.current_neighbor->queue));
        if (pkt != NULL) {
            gnrc_netdev->tx.packet = pkt;
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
        }
        else {
            LOG_ERROR("ERROR: [GOMACH] t2k vTDMA: null packet.\n");
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        }
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_t2k_trans_in_slots(gnrc_netdev_t *gnrc_netdev)
{
    /* If this packet is being retransmitted, use the same recorded MAC sequence number. */
    if (gnrc_netdev->tx.no_ack_counter > 0) {
        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        device_state->seq = gnrc_netdev->tx.tx_seq;
    }

    /* Send data packet in its allocated slots (scheduled slots period). */
    int res = gnrc_gomach_send_data(gnrc_netdev, NETOPT_DISABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2k vTDMA transmission fail: %d, drop packet.\n", res);

        /* If res is < 0, the data packet will not be released in send().
         * so need to release the data here. */
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
        }
        gnrc_netdev->tx.current_neighbor = NULL;
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR,
                            GNRC_GOMACH_NO_TX_ISR_US);

    gnrc_netdev->tx.vtdma_para.slots_num--;
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_VTDMA_FEEDBACK;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_t2k_wait_vtdma_transfeedback(gnrc_netdev_t *gnrc_netdev)
{
    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR))) {
        /* No TX-ISR, go to sleep. */
        gnrc_netdev->tx.no_ack_counter++;

        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        gnrc_netdev->tx.tx_seq = device_state->seq - 1;

        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);
        return;
    }

    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
    	gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

        switch (gnrc_netdev_get_tx_feedback(gnrc_netdev)) {
            case TX_FEEDBACK_SUCCESS: {
                /* First release the packet. */
                gnrc_pktbuf_release(gnrc_netdev->tx.packet);
                gnrc_netdev->tx.packet = NULL;
                gnrc_netdev->tx.no_ack_counter = 0;

                /* If the sender has pending packets and scheduled slots,
                 * continue vTDMA transmission. */
                if ((gnrc_netdev->tx.vtdma_para.slots_num > 0) &&
                    (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0)) {
                    gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.current_neighbor->queue);
                    if (pkt != NULL) {
                        gnrc_netdev->tx.packet = pkt;
                        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
                    }
                    else {
                        LOG_ERROR("ERROR: [GOMACH] t2k vTDMA: null packet.\n");
                        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                    }
                }
                else {
                    /* If no scheduled slots or pending packets, go to t2k end. */
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case TX_FEEDBACK_BUSY:
            case TX_FEEDBACK_NOACK:
            default: {
                /* In case of transmission failure in vTDMA, retransmit the packet in the next
                 * scheduled slot, or the next cycle's t2k procedure. */

                /* Firstly, mark the current TX packet as not ACKed and record the MAC sequence
                 * number, such that the MAC will use the same MAC sequence to send it.
                 * Also, by marking no_ack_counter as non-zero, the neighbor and packet pointers
                 *  will then not be released in t2k-end. Then, the packet can be retried right in
                 *  the following cycle. */
                gnrc_netdev->tx.no_ack_counter = 1;

                netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                /* Do not release the packet here, continue sending the same packet. ***/
                if (gnrc_netdev->tx.vtdma_para.slots_num > 0) {
                    LOG_DEBUG("[GOMACH] no ACK in vTDMA, retry in next slot.\n");
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
                }
                else {
                    /* If no slots for sending, retry in next cycle's t2r, without releasing
                     * tx.packet pointer. */
                    LOG_DEBUG("[GOMACH] no ACK in vTDMA, retry in next cycle.\n");

                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
        }
    }
}

static void gomach_t2k_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);

    /* In GoMacH, normally, in case of transmission failure, no packet will be released
    * in t2k. Failed packet will only be released in t2u. In case of continuous t2k
    * failures, the MAC will goto t2u to retry the packet without releasing it here. */
    if ((gnrc_netdev->tx.packet != NULL) && (gnrc_netdev->tx.no_ack_counter == 0)) {
        LOG_ERROR("ERROR: [GOMACH] t2k: releasing unexpected packet!\n");
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
    }

    /* In case no_ack_counter is not 0 here, it means we will retry to send the packet
     * in t2k or t2u, then, don't release the neighbor pointer. */
    if (gnrc_netdev->tx.no_ack_counter == 0) {
        gnrc_netdev->tx.current_neighbor = NULL;
    }

    /* Clear all timeouts. */
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_CP);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_SLOTS);

    /* Reset t2k_state to the initial state. */
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_INIT;

	gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_gomach_set_enter_new_cycle(gnrc_netdev, false);
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_t2k_update(gnrc_netdev_t *gnrc_netdev)
{
    /* State machine of GoMacH's t2k (transmit to phase-known device) procedure. */
    switch (gnrc_netdev->tx.t2k_state) {
        case GNRC_GOMACH_T2K_INIT: {
            gomach_t2k_init(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_WAIT_CP: {
            gomach_t2k_wait_cp(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_TRANS_IN_CP: {
            gomach_t2k_trans_in_cp(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_WAIT_CPTX_FEEDBACK: {
            gomach_t2k_wait_cp_txfeedback(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_WAIT_BEACON: {
            gomach_t2k_wait_beacon(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_WAIT_SLOTS: {
            gomach_t2k_wait_own_slots(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_VTDMA_TRANS: {
            gomach_t2k_trans_in_slots(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_WAIT_VTDMA_FEEDBACK: {
            gomach_t2k_wait_vtdma_transfeedback(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2K_END: {
            gomach_t2k_end(gnrc_netdev);
            break;
        }
        default: break;
    }
}

static void gomach_t2u_init(gnrc_netdev_t *gnrc_netdev)
{
    /* since t2u is right following CP period (wake-up period), the radio is still on,
     * so we don't need to turn on it again. */

	puts("k");

    LOG_DEBUG("[GOMACH] t2u initialization.\n");

    gnrc_netdev_set_rx_started(gnrc_netdev, false);
    gnrc_gomach_set_quit_cycle(gnrc_netdev, false);
    gnrc_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_netdev->tx.preamble_sent = 0;
    gnrc_gomach_set_got_preamble_ack(gnrc_netdev, false);
    gnrc_gomach_set_buffer_full(gnrc_netdev, false);

    /* Start sending the preamble firstly on public channel 1. */
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);

    /* Disable auto-ACK here! Don't try to reply ACK to any node. */
    gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

    gnrc_gomach_set_on_pubchan_1(gnrc_netdev, true);

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_t2u_send_preamble_prepare(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);

    if (gnrc_netdev->tx.preamble_sent != 0) {
        /* Toggle the radio channel after each preamble transmission. */
        if (gnrc_gomach_get_on_pubchan_1(gnrc_netdev)) {
            gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_2);
            gnrc_gomach_set_on_pubchan_1(gnrc_netdev, false);
        }
        else {
            gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
            gnrc_gomach_set_on_pubchan_1(gnrc_netdev, true);
        }
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL,
                                GNRC_GOMACH_MAX_PREAM_INTERVAL_US);
    }
    else {
        /* Here, for the first preamble, we set the pream_max_interval timeout to
         * 5*MAX_PREAM_INTERVAL due to the fact that the first preamble is
         * using csma for sending, and csma costs some time before actually sending
         * the packet. */
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL,
                                (5 * GNRC_GOMACH_MAX_PREAM_INTERVAL_US));
    }

    gnrc_gomach_set_max_pream_interv(gnrc_netdev, false);
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_SEND_PREAMBLE;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_t2u_send_preamble(gnrc_netdev_t *gnrc_netdev)
{
    /* Now, start sending preamble. */
    int res;

    /* The first preamble is sent with csma for collision avoidance. */
    if (gnrc_netdev->tx.preamble_sent == 0) {
        res = gnrc_gomach_send_preamble(gnrc_netdev, NETOPT_ENABLE);
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION,
                                GNRC_GOMACH_PREAMBLE_DURATION_US);
    }
    else {
        res = gnrc_gomach_send_preamble(gnrc_netdev, NETOPT_DISABLE);
    }

    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2u send preamble failed: %d\n", res);
    }

    /* In case that packet-buffer is full, quit t2u and release packet. */
    if (res == -ENOBUFS) {
        LOG_ERROR("ERROR: [GOMACH] t2u: no pkt-buffer for sending preamble.\n");
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    gnrc_netdev_set_rx_started(gnrc_netdev, false);
    gnrc_netdev->tx.preamble_sent++;
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_PREAMBLE_TX;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_t2u_wait_preamble_tx(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
        /* Set preamble interval timeout. This is a very short timeout (1ms),
         * just to catch the rx-start event of receiving possible preamble-ACK. */
        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE,
                                GNRC_GOMACH_PREAMBLE_INTERVAL_US);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_PREAMBLE_ACK;
        gnrc_gomach_set_update(gnrc_netdev, false);
        return;
    }

    /* This is mainly to handle no-TX-complete error. Once the max preamble interval
     * timeout expired here (i.e., no-TX-complete error), we will quit waiting here
     * and go to send the next preamble, thus the MAC will not get stucked here. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }
}

static bool _handle_in_t2u_send_preamble(gnrc_netdev_t *gnrc_netdev)
{
    /* If packet buffer is full, release one packet to release memory,
     * and reload the next packet.
     * In t2u, we need at least some minimum memory to build the preamble packet. */
    if (gnrc_gomach_get_buffer_full(gnrc_netdev)) {
        gnrc_gomach_set_buffer_full(gnrc_netdev, false);

        gnrc_gomach_set_update(gnrc_netdev, true);

        /* To-do: should we release all the buffered packets in the queue to
         * release memory in such a critical situation? */
        LOG_DEBUG("[GOMACH] t2u: pkt-buffer full, release one pkt.\n");

        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
            gnrc_netdev->tx.no_ack_counter = 0;
        }
        /* Reload the next packet in the neighbor's queue. */
        gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.current_neighbor->queue);

        if (pkt != NULL) {
            gnrc_netdev->tx.packet = pkt;
        }
        else {
            LOG_DEBUG("[GOMACH] t2u: null packet, quit t2u.\n");
            gnrc_netdev->tx.current_neighbor = NULL;
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            return false;
        }
    }

    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL)) {
        gnrc_gomach_set_max_pream_interv(gnrc_netdev, true);
        return true;
    }

    /* if we are receiving packet, wait until RX is completed. */
    if ((!gnrc_gomach_timeout_is_running(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END)) &&
        gnrc_netdev_get_rx_started(gnrc_netdev) &&
        (!gnrc_gomach_get_max_pream_interv(gnrc_netdev))) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);

        /* Set a timeout to wait for the complete of reception. */
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);

        gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END,
                                GNRC_GOMACH_WAIT_RX_END_US);
        return false;
    }

    if (gnrc_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_gomach_set_pkt_received(gnrc_netdev, false);
        gnrc_gomach_process_pkt_in_wait_preamble_ack(gnrc_netdev);
    }

    /* Quit t2u if we have to, e.g., the device found ongoing bcast of other devices. */
    if (gnrc_gomach_get_quit_cycle(gnrc_netdev)) {
        LOG_DEBUG("[GOMACH] quit t2u.\n");
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return false;
    }

    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END)) {
        gnrc_gomach_set_max_pream_interv(gnrc_netdev, true);
    }
    return true;
}

static void gomach_t2u_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{
    if (!_handle_in_t2u_send_preamble(gnrc_netdev)) {
        return;
    }

    if (gnrc_gomach_get_got_preamble_ack(gnrc_netdev)) {

        /* Require ACK for the packet waiting to be sent! */
        gnrc_gomach_set_ack_req(gnrc_netdev, NETOPT_ENABLE);

        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_SEND_DATA;
        gnrc_netdev->tx.t2u_fail_count = 0;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION)) {
        gnrc_netdev->tx.t2u_retry_counter++;

        /* If we reach the maximum t2u retry limit, release the data packet. */
        if (gnrc_netdev->tx.t2u_retry_counter >= GNRC_GOMACH_T2U_RETYR_THRESHOLD) {
            LOG_DEBUG("[GOMACH] t2u failed: no preamble-ACK.\n");
            gnrc_netdev->tx.t2u_retry_counter = 0;
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);

            gnrc_netdev->tx.t2u_fail_count++;
        }
        else {
            /* If we haven't reach the maximum t2u limit, try again. Set quit_current_cycle flag
             * to true such that we will release the current neighbor pointer.  */
            gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        }

        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    /* If we didn't catch the RX-start event, go to send the next preamble. */
    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE)) ||
        gnrc_gomach_get_max_pream_interv(gnrc_netdev)) {
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
        gnrc_gomach_set_update(gnrc_netdev, true);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
    }
}

static void gomach_t2u_send_data(gnrc_netdev_t *gnrc_netdev)
{
    /* If we are retrying to send the data, reload its original MAC sequence. */
    if (gnrc_netdev->tx.no_ack_counter > 0) {
        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        device_state->seq = gnrc_netdev->tx.tx_seq;
    }

    /* Here, we send the data to the receiver. */
    int res = gnrc_gomach_send_data(gnrc_netdev, NETOPT_ENABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2u data sending error: %d.\n", res);

        /* If res is < 0, the data packet will not be released in send().
         * so need to release the data here. */
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
        }

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR,
                            GNRC_GOMACH_NO_TX_ISR_US);

    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_DATA_TX;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_t2u_wait_tx_feedback(gnrc_netdev_t *gnrc_netdev)
{
    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR))) {
        /* No TX-ISR, go to sleep. */
        gnrc_netdev->tx.t2u_retry_counter++;

        gnrc_netdev->tx.no_ack_counter = GNRC_GOMACH_REPHASELOCK_THRESHOLD;
        netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
        gnrc_netdev->tx.tx_seq = device_state->seq - 1;

        gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;

        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    if (gnrc_gomach_get_tx_finish(gnrc_netdev)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

        if (gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_SUCCESS) {
            /* If transmission succeeded, release the data. */
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;

            gnrc_netdev->tx.no_ack_counter = 0;
            gnrc_netdev->tx.t2u_retry_counter = 0;

            /* Attend the vTDMA procedure if the sender has pending packets for the receiver. */
            if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0) {
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;
                gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
                gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION);
                gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
                gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);

                /* Switch to t2k procedure and wait for the beacon of the receiver. */
                gnrc_netdev->tx.vtdma_para.slots_num = 0;
                gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_BEACON,
                                        GNRC_GOMACH_WAIT_BEACON_TIME_US);
                gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

                gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_BEACON;
                gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_KNOWN;
            }
            else {
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            }
        }
        else {
            gnrc_netdev->tx.t2u_retry_counter++;
            /* If we meet t2u retry limit, release the packet. */
            if (gnrc_netdev->tx.t2u_retry_counter >= GNRC_GOMACH_T2U_RETYR_THRESHOLD) {
                LOG_DEBUG("[GOMACH] t2u send data failed on channel %d,"
                          " drop packet.\n", gnrc_netdev->tx.current_neighbor->pub_chanseq);
                gnrc_pktbuf_release(gnrc_netdev->tx.packet);
                gnrc_netdev->tx.packet = NULL;
                gnrc_netdev->tx.current_neighbor = NULL;
                gnrc_netdev->tx.no_ack_counter = 0;
                gnrc_netdev->tx.t2u_retry_counter = 0;
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            }
            else {
                /* Record the MAC sequence of the data, retry t2u in next cycle. */
                gnrc_netdev->tx.no_ack_counter = GNRC_GOMACH_REPHASELOCK_THRESHOLD;
                netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                LOG_DEBUG("[GOMACH] t2u send data failed on channel %d.\n",
                          gnrc_netdev->tx.current_neighbor->pub_chanseq);
                /* Set quit_current_cycle to true, thus not to release current_neighbour pointer
                 * in t2u-end */
                gnrc_gomach_set_quit_cycle(gnrc_netdev, true);
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            }
        }
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_t2u_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAMBLE);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_PREAM_DURATION);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL);
    gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

    /* In case quit_current_cycle is true, don't release neighbor pointer,
     * will retry t2u immediately in next cycle.*/
    if (!gnrc_gomach_get_quit_cycle(gnrc_netdev)) {
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
            gnrc_netdev->tx.no_ack_counter = 0;
            LOG_WARNING("WARNING: [GOMACH] t2u: drop packet.\n");
        }
        gnrc_netdev->tx.current_neighbor = NULL;
    }

    /* Reset t2u state. */
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;

    /* Resume to listen state and go to sleep. */
    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_gomach_set_enter_new_cycle(gnrc_netdev, false);
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_t2u_update(gnrc_netdev_t *gnrc_netdev)
{
    /* State machine of GoMacH's t2u (transmit to phase-unknown device) procedure. */
    switch (gnrc_netdev->tx.t2u_state) {
        case GNRC_GOMACH_T2U_INIT: {
            gomach_t2u_init(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_PREAMBLE_PREPARE: {
            gomach_t2u_send_preamble_prepare(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_SEND_PREAMBLE: {
            gomach_t2u_send_preamble(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_WAIT_PREAMBLE_TX: {
            gomach_t2u_wait_preamble_tx(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_WAIT_PREAMBLE_ACK: {
            gomach_t2u_wait_preamble_ack(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_SEND_DATA: {
            gomach_t2u_send_data(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_WAIT_DATA_TX: {
            gomach_t2u_wait_tx_feedback(gnrc_netdev);
            break;
        }
        case GNRC_GOMACH_T2U_END: {
            gomach_t2u_end(gnrc_netdev);
            break;
        }
        default: break;
    }
}

static void _gomach_phase_backoff(gnrc_netdev_t *gnrc_netdev)
{
    /* Execute phase backoff for avoiding CP (wake-up period) overlap. */
    rtt_clear_alarm();
    rtt_poweroff();
    xtimer_usleep(gnrc_netdev->gomach.backoff_phase_us);

    rtt_poweron();
    rtt_set_counter(0);
    gnrc_netdev->gomach.last_wakeup = rtt_get_counter();

    uint32_t alarm = gnrc_netdev->gomach.last_wakeup +
                     RTT_US_TO_TICKS(GNRC_GOMACH_SUPERFRAME_DURATION_US);

    rtt_set_alarm(alarm, _gomach_rtt_cb, (void *) GNRC_GOMACH_EVENT_RTT_NEW_CYCLE);

    puts("ph-bckf");
}

static void gomach_listen_init(gnrc_netdev_t *gnrc_netdev)
{

	//printf("C:%lu\n",xtimer_now_usec());

	if(((xtimer_now_usec() - gnrc_netdev->tx.get_bcast_pkt_time) > 8000000)&&(gnrc_netdev->tx.get_bcast_pkt == true)) {
	    //if ((xtimer_now_usec() - gnrc_netdev->tx.get_bcast_pkt_time) > 60000000) {
		gnrc_netdev->tx.get_bcast_pkt = false;
	}

    /* Reset last_seq_info, for avoiding receiving duplicate packets.
     * To-do: remove this in the future? */
    for (int i = 0; i < GNRC_GOMACH_DUPCHK_BUFFER_SIZE; i++) {
        if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len != 0) {
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle++;
            if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle >=
                GNRC_GOMACH_RX_DUPCHK_UNIT_LIFE) {
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr[0] = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr[1] = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].seq = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
            }
        }
    }

    if (gnrc_netdev->tx.t2u_fail_count >= GNRC_GOMACH_MAX_T2U_RETYR_THRESHOLD) {
        gnrc_netdev->tx.t2u_fail_count = 0;
        LOG_DEBUG("[GOMACH]: Re-initialize radio.");
        gomach_reinit_radio(gnrc_netdev);
    }
    gnrc_gomach_set_enter_new_cycle(gnrc_netdev, false);

    /* Set listen period timeout. */
    uint32_t listen_period = random_uint32_range(0, GNRC_GOMACH_CP_RANDOM_END_US) +
                             GNRC_GOMACH_CP_DURATION_US;
    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END, listen_period);
    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_MAX, GNRC_GOMACH_CP_DURATION_MAX_US);

    gnrc_netdev_set_rx_started(gnrc_netdev, false);
    gnrc_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_netdev->gomach.cp_extend_count = 0;
    gnrc_gomach_set_quit_cycle(gnrc_netdev, false);
    gnrc_gomach_set_unintd_preamble(gnrc_netdev, false);
    gnrc_gomach_set_beacon_fail(gnrc_netdev, false);
    gnrc_gomach_set_cp_end(gnrc_netdev, false);
    gnrc_gomach_set_got_preamble(gnrc_netdev, false);

    /* Flush RX queue and turn on radio. */
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);

    /* Turn to current public channel. */
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.cur_pub_channel);

    /* Enable Auto-ACK for data packet reception. */
    gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_LISTEN;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_listen_cp_listen(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_gomach_set_pkt_received(gnrc_netdev, false);
        gnrc_gomach_cp_packet_process(gnrc_netdev);

        /* If the device has replied a preamble-ACK, it must waits for the data.
         * Here, we extend the CP. */
        if (gnrc_gomach_get_got_preamble(gnrc_netdev)) {
            gnrc_gomach_set_got_preamble(gnrc_netdev, false);
            gnrc_gomach_set_cp_end(gnrc_netdev, false);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END);
            gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END, GNRC_GOMACH_CP_DURATION_US);
        }
        else if ((!gnrc_gomach_get_unintd_preamble(gnrc_netdev)) &&
                 (!gnrc_gomach_get_quit_cycle(gnrc_netdev))) {
            gnrc_gomach_set_got_preamble(gnrc_netdev, false);
            gnrc_gomach_set_cp_end(gnrc_netdev, false);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END);
            gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END, GNRC_GOMACH_CP_DURATION_US);
        }
    }

    /* If we have reached the maximum CP duration, quit CP. */
    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_MAX)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_MAX);
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
        return;
    }

    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END))) {
        gnrc_gomach_set_cp_end(gnrc_netdev, true);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END);
    }

    /* If CP duration timeouted or we must quit CP, go to CP end. */
    if (gnrc_gomach_get_cp_end(gnrc_netdev) || gnrc_gomach_get_quit_cycle(gnrc_netdev)) {
        /* If we found ongoing reception, wait for reception complete. */
        if ((gnrc_gomach_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) &&
            (gnrc_netdev->gomach.cp_extend_count < GNRC_GOMACH_CP_EXTEND_THRESHOLD)) {
            gnrc_netdev->gomach.cp_extend_count++;
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
            gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END,
                                    GNRC_GOMACH_WAIT_RX_END_US);
        }
        else {
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_END);
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_CP_MAX);
            gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_END;
            gnrc_gomach_set_update(gnrc_netdev, true);
        }
    }
}

static void gomach_listen_cp_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gnrc_mac_dispatch(&gnrc_netdev->rx);

    /* If we need to quit communications in this cycle, go to sleep. */
    if (gnrc_gomach_get_quit_cycle(gnrc_netdev)) {
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
    }
    else {
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SEND_BEACON;
    }
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_listen_send_beacon(gnrc_netdev_t *gnrc_netdev)
{
	/* First check if there are slots needed to be allocated. */
    uint8_t slot_num = 0;
    int i;
    for (i = 0; i < GNRC_GOMACH_SLOSCH_UNIT_COUNT; i++) {
        if (gnrc_netdev->rx.slosch_list[i].queue_indicator > 0) {
        	slot_num += gnrc_netdev->rx.slosch_list[i].queue_indicator;
            break;
        }
    }

    if (slot_num > 0) {
        /* Disable auto-ACK. Thus not to receive packet (attempt to reply ACK) anymore. */
        gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

        /* Assemble and send the beacon. */
        int res = gnrc_gomach_send_beacon(gnrc_netdev);
        if (res < 0) {
            LOG_ERROR("ERROR: [GOMACH] send beacon error: %d.\n", res);
            gnrc_gomach_set_beacon_fail(gnrc_netdev, true);
            gnrc_gomach_set_update(gnrc_netdev, true);
        }
        else {
            gnrc_gomach_set_update(gnrc_netdev, false);
        }
    }
    else {
    	/* No need to send beacon, go to next state. */
        gnrc_gomach_set_beacon_fail(gnrc_netdev, true);
        gnrc_gomach_set_update(gnrc_netdev, true);
    }

    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_WAIT_BEACON_TX;
}

static void gomach_listen_wait_beacon_tx(gnrc_netdev_t *gnrc_netdev)
{
    if ((gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR))) {
        /* No TX-ISR, go to sleep. */
        LOG_DEBUG("[GOMACH]: no TX-finish ISR.");
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
        gnrc_gomach_set_update(gnrc_netdev, true);
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);
        return;
    }

    if (gnrc_gomach_get_tx_finish(gnrc_netdev) ||
        gnrc_gomach_get_beacon_fail(gnrc_netdev)) {
        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_NO_TX_ISR);

        if ((gnrc_netdev->rx.vtdma_manag.total_slots_num > 0) &&
            (!gnrc_gomach_get_beacon_fail(gnrc_netdev))) {
            /* If the device has allocated transmission slots to other nodes,
             *  switch to vTDMA period to receive packets. */
            gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA_INIT;
            gnrc_gomach_set_update(gnrc_netdev, true);
        }
        else {
            /* If the device hasn't allocated transmission slots, check whether it has packets
             * to transmit to neighbor. */
            if (gnrc_gomach_find_next_tx_neighbor(gnrc_netdev)) {
                /* Now, we have packet to send. */

                if (gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]) {
                    /* The packet is for broadcasting. */

                    /* If we didn't find ongoing preamble stream, go to send broadcast packet. */
                    if (!gnrc_gomach_get_unintd_preamble(gnrc_netdev)) {
                        gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                        gnrc_netdev->tx.transmit_state = GNRC_GOMACH_BROADCAST;
                    }
                    else {
                        /* If we find ongoing preamble stream, go to sleep. */
                        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                    }
                }
                else {
                    /* The packet waiting to be sent is for unicast. */
                    switch (gnrc_netdev->tx.current_neighbor->mac_type) {
                        case GNRC_GOMACH_TYPE_UNKNOWN: {
                            /* The neighbor's phase is unknown yet, try to run t2u (transmission
                             * to unknown device) procedure to phase-lock the neighbor. */

                            /* If we didn't find ongoing preamble stream, go to t2u procedure. */
                            if (!gnrc_gomach_get_unintd_preamble(gnrc_netdev)) {
                                gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                                gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;
                            }
                            else {
                                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                            }
                            break;
                        }
                        case GNRC_GOMACH_TYPE_KNOWN: {
                            /* If the neighbor's phase is known, go to t2k (transmission
                             * to known device) procedure. Here, we don't worry that the t2k
                             * unicast transmission will interrupt with possible ongoing
                             * preamble transmissions of other devices. */
                            gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                            gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_KNOWN;
                            break;
                        }
                        default: {
                            LOG_ERROR("ERROR: [GOMACH] vTDMA: unknown MAC type of "
                                      "the neighbor.\n");
                            break;
                        }
                    }
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
            }
            else {
                /* No packet to send, go to sleep. */
                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                gnrc_gomach_set_update(gnrc_netdev, true);
            }
        }
    }
}

static void gomach_vtdma_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Switch the radio to the device's sub-channel. */
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.sub_channel_seq);

    /* Enable Auto ACK again for data reception */
    gnrc_gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);

    /* Set the vTDMA period timeout. */
    uint32_t vtdma_duration = gnrc_netdev->rx.vtdma_manag.total_slots_num *
                              GNRC_GOMACH_VTDMA_SLOT_SIZE_US;
    gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_VTDMA, vtdma_duration);

    gnrc_gomach_set_vTDMA_end(gnrc_netdev, false);

    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA;
    gnrc_gomach_set_update(gnrc_netdev, false);
}

static void gomach_vtdma(gnrc_netdev_t *gnrc_netdev)
{
    /* Process received packet here. */
    if (gnrc_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_gomach_set_pkt_received(gnrc_netdev, false);
        gnrc_gomach_packet_process_in_vtdma(gnrc_netdev);
    }

    if (gnrc_gomach_timeout_is_expired(gnrc_netdev, GNRC_GOMACH_TIMEOUT_VTDMA)) {
        gnrc_gomach_set_vTDMA_end(gnrc_netdev, true);
    }

    /* Go to vTDMA end after vTDMA timeout expires. */
    if (gnrc_gomach_get_vTDMA_end(gnrc_netdev)) {
        /* Wait for reception complete if found ongoing transmission. */
        if (gnrc_gomach_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) {
            gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
            gnrc_gomach_set_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END,
                                    GNRC_GOMACH_WAIT_RX_END_US);
            return;
        }

        gnrc_gomach_clear_timeout(gnrc_netdev, GNRC_GOMACH_TIMEOUT_WAIT_RX_END);
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_vtdma_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gnrc_mac_dispatch(&gnrc_netdev->rx);

    /* Switch the radio to the public-channel. */
    gnrc_gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.cur_pub_channel);

    /* Check if there is packet to send. */
    if (gnrc_gomach_find_next_tx_neighbor(gnrc_netdev)) {
        if (gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]) {
            /* The packet is for broadcasting. */
            if (!gnrc_gomach_get_unintd_preamble(gnrc_netdev)) {
                gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                gnrc_netdev->tx.transmit_state = GNRC_GOMACH_BROADCAST;
            }
            else {
                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
            }
        }
        else {
            switch (gnrc_netdev->tx.current_neighbor->mac_type) {
                /* The packet waiting to be sent is for unicast. */
                case GNRC_GOMACH_TYPE_UNKNOWN: {
                    /* The neighbor's phase is unknown yet, try to run t2u (transmission
                     * to unknown device) procedure to phase-lock the neighbor. */
                    if (!gnrc_gomach_get_unintd_preamble(gnrc_netdev)) {
                        gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                        gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;
                    }
                    else {
                        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                    }
                } break;
                case GNRC_GOMACH_TYPE_KNOWN: {
                    /* If the neighbor's phase is known, go to t2k (transmission
                     * to known device) procedure. Here, we don't worry that the t2k
                     * unicast transmission will interrupt with possible ongoing
                     * preamble transmissions of other devices. */
                    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                    gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_KNOWN;
                } break;
                default: {
                    LOG_ERROR("ERROR: [GOMACH] vTDMA: unknown MAC type of the neighbor.\n");
                    break;
                }
            }
        }
    }
    else {
        /* No packet to send, go to sleep. */
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
    }

    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_sleep_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Turn off the radio during sleep period to conserve power. */
    gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_sleep(gnrc_netdev_t *gnrc_netdev)
{
    /* If we are entering a new cycle, quit sleeping. */
    if (gnrc_gomach_get_enter_new_cycle(gnrc_netdev)) {
    	gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_END;
        gnrc_gomach_set_update(gnrc_netdev, true);
    }
}

static void gomach_sleep_end(gnrc_netdev_t *gnrc_netdev)
{
	if (gnrc_gomach_get_phase_backoff(gnrc_netdev)) {
	    gnrc_gomach_set_phase_backoff(gnrc_netdev, false);
	    _gomach_phase_backoff(gnrc_netdev);
	}

    /* Go to CP (start of the new cycle), start listening on the public-channel. */
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_INIT;
    gnrc_gomach_set_update(gnrc_netdev, true);
}

static void gomach_update(gnrc_netdev_t *gnrc_netdev)
{
    switch (gnrc_netdev->gomach.basic_state) {
        case GNRC_GOMACH_INIT: {
            /* State machine of GoMacH's initialization procedure. */
            switch (gnrc_netdev->gomach.init_state) {
                case GNRC_GOMACH_INIT_PREPARE: {
                    gomach_init_prepare(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_INIT_ANNC_SUBCHAN: {
                    gomach_init_announce_subchannel(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_INIT_WAIT_FEEDBACK: {
                    gomach_init_wait_announce_feedback(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_INIT_END: {
                    gomach_init_end(gnrc_netdev);
                    break;
                }
                default: break;
            }
            break;
        }
        case GNRC_GOMACH_LISTEN: {
            /* State machine of GoMacH's duty-cycled listen procedure. */
            //printf("C-%u\n",gnrc_netdev->rx.listen_state);

            switch (gnrc_netdev->rx.listen_state) {
                case GNRC_GOMACH_LISTEN_CP_INIT: {
                    gomach_listen_init(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_CP_LISTEN: {
                    gomach_listen_cp_listen(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_CP_END: {
                    gomach_listen_cp_end(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_SEND_BEACON: {
                    gomach_listen_send_beacon(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_WAIT_BEACON_TX: {
                    gomach_listen_wait_beacon_tx(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_VTDMA_INIT: {
                    gomach_vtdma_init(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_VTDMA: {
                    gomach_vtdma(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_VTDMA_END: {
                    gomach_vtdma_end(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_SLEEP_INIT: {
                    gomach_sleep_init(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_SLEEP: {
                    gomach_sleep(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_LISTEN_SLEEP_END: {
                    gomach_sleep_end(gnrc_netdev);
                    break;
                }
                default: break;
            }
            break;
        }
        case GNRC_GOMACH_TRANSMIT: {
            /* State machine of GoMacH's basic transmission scheme. */
            switch (gnrc_netdev->tx.transmit_state) {
                case GNRC_GOMACH_TRANS_TO_UNKNOWN: {
                    gomach_t2u_update(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_TRANS_TO_KNOWN: {
                    gomach_t2k_update(gnrc_netdev);
                    break;
                }
                case GNRC_GOMACH_BROADCAST: {
                    gomach_bcast_update(gnrc_netdev);
                    break;
                }
                default: break;
            }
            break;
        }
        default: break;
    }
}

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *) dev->context;

    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = (void *) gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            DEBUG("[GOMACH] gnrc_netdev: possibly lost interrupt.\n");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch (event) {
            case NETDEV_EVENT_RX_STARTED: {
                gnrc_netdev_set_rx_started(gnrc_netdev, true);
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case NETDEV_EVENT_RX_COMPLETE: {
                gnrc_gomach_set_update(gnrc_netdev, true);

                gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);
                if (pkt == NULL) {
                    gnrc_gomach_set_buffer_full(gnrc_netdev, true);

                    LOG_DEBUG("[GOMACH] gnrc_netdev: packet is NULL, memory full?\n");
                    gnrc_gomach_set_pkt_received(gnrc_netdev, false);
                    gnrc_netdev_set_rx_started(gnrc_netdev, false);
                    break;
                }

                if (!gnrc_netdev_get_rx_started(gnrc_netdev)) {
                    LOG_DEBUG("[GOMACH] gnrc_netdev: maybe sending kicked in "
                              "and frame buffer is now corrupted?\n");
                    gnrc_pktbuf_release(pkt);
                    gnrc_netdev_set_rx_started(gnrc_netdev, false);
                    break;
                }

                gnrc_netdev_set_rx_started(gnrc_netdev, false);

                if (!gnrc_mac_queue_rx_packet(&gnrc_netdev->rx, 0, pkt)) {
                    LOG_ERROR("ERROR: [GOMACH] gnrc_netdev: can't push RX packet, queue full?\n");
                    gnrc_pktbuf_release(pkt);
                    gnrc_gomach_set_pkt_received(gnrc_netdev, false);
                    break;
                }
                else {
                    gnrc_gomach_set_pkt_received(gnrc_netdev, true);
                    puts("R");
                }
                break;
            }
            case NETDEV_EVENT_TX_COMPLETE: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_SUCCESS);
                gnrc_gomach_set_tx_finish(gnrc_netdev, true);
                gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case NETDEV_EVENT_TX_NOACK: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_NOACK);
                gnrc_gomach_set_tx_finish(gnrc_netdev, true);
                gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case NETDEV_EVENT_TX_MEDIUM_BUSY: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_BUSY);
                gnrc_gomach_set_tx_finish(gnrc_netdev, true);
                gnrc_gomach_set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            default: {
                DEBUG("gnrc_netdev: warning: unhandled event %u.\n", event);
            }
        }
    }
}

/**
 * @brief   Startup code and event loop of the GoMacH MAC protocol
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_gomach_thread(void *args)
{
    DEBUG("gnrc_netdev: starting thread\n");
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *)args;
    netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[GNRC_GOMACH_IPC_MSG_QUEUE_SIZE];

    /* Setup the MAC layers message queue. */
    msg_init_queue(msg_queue, GNRC_GOMACH_IPC_MSG_QUEUE_SIZE);

    /* Register the event callback with the device driver. */
    dev->event_callback = _event_cb;
    dev->context = (void *) gnrc_netdev;

    /* Register the device to the network stack. */
    gnrc_netif_add(thread_getpid());

    /* Initialize low-level driver. */
    dev->driver->init(dev);

    /* Initialize RTT. */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send message. */
    gomach_pid = gnrc_netdev->pid;

    /* Set MAC address length. */
    uint16_t src_len = IEEE802154_LONG_ADDRESS_LEN;
    dev->driver->set(dev, NETOPT_SRC_LEN, &src_len, sizeof(src_len));

    /* Initialize GoMacH's parameters. */
    gomach_init(gnrc_netdev);

    gnrc_gomach_set_update(gnrc_netdev, true);

    while (gnrc_gomach_get_update(gnrc_netdev)) {
        gnrc_gomach_set_update(gnrc_netdev, false);
        gomach_update(gnrc_netdev);
    }

    /* Start the event loop */
    while (1) {
        DEBUG("gnrc_netdev: waiting for incoming messages\n");
        msg_receive(&msg);
        /* Dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV_MSG_TYPE_EVENT: {
                DEBUG("gnrc_netdev: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            }
            case GNRC_NETAPI_MSG_TYPE_SET: {
                /* Read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                      netopt2str(opt->opt));
                /* Set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->set: %i\n", res);
                /* Send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            }
            case GNRC_NETAPI_MSG_TYPE_GET: {
                /* Read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                      netopt2str(opt->opt));
                /* Get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->get: %i\n", res);
                /* Send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            }
            case GNRC_NETAPI_MSG_TYPE_SND: {
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SND received\n");

                gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)msg.content.ptr;

                if (!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, pkt)) {
                    /* TX packet queue full, release the packet. */
                    DEBUG("[GOMACH] TX queue full, drop packet.\n");
                    gnrc_pktbuf_release(pkt);
                }
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
            case GNRC_GOMACH_EVENT_RTT_TYPE: {
                _gomach_rtt_handler(msg.content.value, gnrc_netdev);
                break;
            }
            case GNRC_GOMACH_EVENT_TIMEOUT_TYPE: {
                /* GoMacH timeout expires. */
                gnrc_gomach_timeout_make_expire((gnrc_gomach_timeout_t *) msg.content.ptr);
                gnrc_gomach_set_update(gnrc_netdev, true);
                break;
            }
#if (GNRC_GOMACH_ENABLE_DUTYCYLE_RECORD == 1)
            case GNRC_MAC_TYPE_GET_DUTYCYCLE: {
                /* Output radio duty-cycle ratio */
                uint64_t duty;
                duty = (uint64_t) xtimer_now_usec64();
                duty = (gnrc_netdev->gomach.awake_duration_sum_ticks) * 100 /
                       (duty - gnrc_netdev->gomach.system_start_time_ticks);

                uint16_t add = 0;
                add = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 2];
                add = seed << 8;
                add |= gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len - 1];

                printf("[ %x ]: active-time (ms): %lu ms \n", add, (uint32_t)(gnrc_netdev->gomach.awake_duration_sum_ticks / 1000));
                printf("[ %x ]: life-time (ms): %lu ms \n", add, (uint32_t)((xtimer_now_usec64() - gnrc_netdev->gomach.system_start_time_ticks) / 1000));
                printf("[ %x ]: achieved radio duty-cycle: %lu %% \n", add, (uint32_t)duty);
                break;
            }
#endif
            default: {
                DEBUG("gnrc_netdev: Unknown command %" PRIu16 "\n", msg.type);
                break;
            }
        }

        while (gnrc_gomach_get_update(gnrc_netdev)) {
            gnrc_gomach_set_update(gnrc_netdev, false);
            gomach_update(gnrc_netdev);
        }
    }
    /* Never reached */
    return NULL;
}

kernel_pid_t gnrc_gomach_init(char *stack, int stacksize, char priority,
                              const char *name, gnrc_netdev_t *gnrc_netdev)
{
    /* Check if given netdev device is defined and the driver is set */
    if (gnrc_netdev == NULL || gnrc_netdev->dev == NULL) {
        LOG_ERROR("ERROR: [GoMacH] No netdev supplied or driver not set.\n");
        return -ENODEV;
    }

    /* Create new gnrc_netdev thread */
    kernel_pid_t res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                                     _gnrc_gomach_thread, (void *)gnrc_netdev, name);
    if (res <= 0) {
        LOG_ERROR("ERROR: [GoMacH] Couldn't create thread.\n");
        return -EINVAL;
    }

    return res;
}

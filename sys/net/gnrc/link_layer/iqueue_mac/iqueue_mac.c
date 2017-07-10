/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iqueue_mac
 * @file
 * @brief       implementation of iqueue_mac
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@iniria.fr>
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
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include "net/gnrc/iqueue_mac/iqueuemac_types.h"
#include "net/gnrc/netdev/ieee802154.h"
#include "include/iqueuemac_internal.h"
#include "net/gnrc/iqueue_mac/timeout.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef LOG_LEVEL
/**
 * @brief Default log level define
 */
#define LOG_LEVEL LOG_WARNING
#endif

#include "log.h"

#define NETDEV_NETAPI_MSG_QUEUE_SIZE 8

static kernel_pid_t iqueuemac_pid;

void iqueuemac_init(gnrc_netdev_t *gnrc_netdev)
{

    gnrc_netdev->l2_addr_len = gnrc_netdev->dev->driver->get(gnrc_netdev->dev,
                                                             NETOPT_ADDRESS_LONG,
                                                             gnrc_netdev->l2_addr,
                                                             sizeof(gnrc_netdev->l2_addr));

    //printf("gomach: gomach's own addrs is: %d, %d . \n ", gomach->own_addr.addr[1], gomach->own_addr.addr[0]);

    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_INIT;  //GNRC_GOMACH_LISTEN;
    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_PREPARE;

    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_INIT;
    gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;

    gnrc_netdev->rx.enter_new_cycle = false;

    gnrc_netdev->rx.router_vtdma_mana.sub_channel_seq = 26;

    gnrc_netdev->gomach.subchannel_occu_flags = 0;

    /*** initiate the sub_channel_num  ***/
    //uint16_t random_channel = gomach->own_addr.addr[0] % 15;
    //gomach->sub_channel_num = 11 + random_channel;
    gnrc_netdev->gomach.sub_channel_num = 13;

    gnrc_netdev->gomach.pub_channel_1 = 26;
    gnrc_netdev->gomach.pub_channel_2 = 11;
    gnrc_netdev->gomach.cur_pub_channel = gnrc_netdev->gomach.pub_channel_1;

    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_INIT;
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_INIT;
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;

    gnrc_netdev->tx.no_ack_counter = 0;

    /* Enable RX-start and TX-started and TX-END interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    //gomach->netdev->dev->driver->set(gomach->netdev->dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    /* Initialize broadcast sequence number. This at least differs from board
     * to board */
    gnrc_netdev->tx.broadcast_seq = gnrc_netdev->l2_addr[0];

    /* First neighbour queue is supposed to be broadcast queue */
    //int broadcast_queue_id = _alloc_neighbour(&gnrc_netdev->gomach);
    //assert(broadcast_queue_id == 0);

    /* Setup broadcast tx queue */
    //uint8_t broadcast_addr[] = {0xff, 0xff};
    //_init_neighbour(_get_neighbour(gnrc_netdev->gomach, 0), broadcast_addr, sizeof(broadcast_addr));

    /* Initialize receive packet queue
       packet_queue_init(&(gomach->rx.queue),
                      gomach->rx._queue_nodes,
                      (sizeof(gomach->rx._queue_nodes) / sizeof(packet_queue_node_t)));
     */

    /* Reset all timeouts just to be sure */
    iqueuemac_reset_timeouts(&gnrc_netdev->gomach);

    gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_netdev->gomach.need_update = false;
    gnrc_netdev->gomach.duty_cycle_started = false;
    gnrc_netdev->gomach.quit_current_cycle = false;
    gnrc_netdev->gomach.send_beacon_fail = false;
    gnrc_netdev->gomach.rx_memory_full = false;
    gnrc_netdev->gomach.phase_backoff = false;

    gnrc_netdev->rx.check_dup_pkt.queue_head = 0;
    gnrc_netdev->tx.last_tx_neighbor_id = 0;

    netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
    device_state->seq = gnrc_netdev->l2_addr[0];

    for (int i = 0; i < IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE; i++) {
        gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
    }

}

static void rtt_cb(void *arg)
{
    msg_t msg;

    msg.content.value = ((uint32_t) arg) & 0xffff;
    msg.type = IQUEUEMAC_EVENT_RTT_TYPE;
    msg_send(&msg, iqueuemac_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}


void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    uint32_t alarm;

    switch (event & 0xffff) {
        /*******************************Router RTT management***************************/
        case IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE: {
            if (gnrc_netdev->gomach.duty_cycle_started == false) {
                gnrc_netdev->gomach.duty_cycle_started = true;
                rtt_clear_alarm();
                /*** record the starting phase of iQueuemac ***/
                gnrc_netdev->gomach.last_wakeup = rtt_get_counter();
            }
            else {
                gnrc_netdev->gomach.last_wakeup = rtt_get_alarm(); //rtt_get_counter();
                gnrc_netdev->rx.enter_new_cycle = true;
                // iqueuemac_stop_lpm();
            }

            //lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;


            //alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
            alarm = gnrc_netdev->gomach.last_wakeup + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
            rtt_set_alarm(alarm, rtt_cb, (void *) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);

            update_neighbor_pubchan(gnrc_netdev);

            gnrc_netdev->gomach.need_update = true;

        } break;

        /********************************************************/
        case IQUEUEMAC_EVENT_RTT_START: {
            gnrc_netdev->gomach.duty_cycle_started = true;
            gnrc_netdev->gomach.need_update = true;
        } break;

        default: break;

    }

}

static void gomach_bcast_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Disable auto-ACK when sending broadcast packets, thus not to receive packet. */
    gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

    /* Firstly turn the radio to public channel 1. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
    gnrc_netdev->tx.t2u_on_public_1 = true;

    gnrc_netdev->tx.broadcast_seq ++;

    /* Assemble the broadcast packet. */
    gnrc_pktsnip_t *pkt = gnrc_netdev->tx.packet;
    gnrc_pktsnip_t *payload = gnrc_netdev->tx.packet->next;

    iqueuemac_frame_broadcast_t iqueuemac_broadcast_hdr;
    iqueuemac_broadcast_hdr.header.type = FRAMETYPE_BROADCAST;
    iqueuemac_broadcast_hdr.seq_nr = gnrc_netdev->tx.broadcast_seq;
    pkt->next = gnrc_pktbuf_add(pkt->next, &iqueuemac_broadcast_hdr,
                                sizeof(iqueuemac_broadcast_hdr),
                                GNRC_NETTYPE_IQUEUEMAC);
    if (pkt->next == NULL) {
        /* Make append payload after netif header again */
        gnrc_netdev->tx.packet->next = payload;
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
        LOG_ERROR("ERROR: [GOMACH] bcast: no memory to assemble bcast packet, drop packet.\n");
        LOG_ERROR("ERROR: [GOMACH] bcast failed, go to listen mode.\n");
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_FINISH,
                          IQUEUEMAC_SUPERFRAME_DURATION_US);

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_SEND;
    gnrc_netdev->gomach.need_update = true;
}

static bool _gomach_send_bcast_busy_handle(gnrc_netdev_t *gnrc_netdev)
{
    /* Quit sending broadcast packet if we found ongoing transmissions, for collision avoidance. */
    if((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) ||
       (gnrc_netdev_get_rx_started(gnrc_netdev) == true)) {
        LOG_WARNING("WARNING: [GOMACH] bcast: found ongoing transmission, quit broadcast.\n");
    	/* Queue the broadcast packet back to the queue. */
    	gnrc_pktsnip_t* payload = gnrc_netdev->tx.packet->next->next;

    	/* remove gomach header */
    	gnrc_netdev->tx.packet->next->next = NULL;
    	gnrc_pktbuf_release(gnrc_netdev->tx.packet->next);

    	/* make append payload after netif header again */
    	gnrc_netdev->tx.packet->next = payload;

    	if(!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, gnrc_netdev->tx.packet)){
    	   	LOG_WARNING("WARNING: [GOMACH] bcast: TX queue full, release packet.\n");
    	   	gnrc_pktbuf_release(gnrc_netdev->tx.packet);
    	}
    	gnrc_netdev->tx.packet = NULL;

    	gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
    	gnrc_netdev->gomach.need_update = true;
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
    gomach_send(gnrc_netdev, gnrc_netdev->tx.packet, NETOPT_DISABLE);

    gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_WAIT_TX_FINISH;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_wait_bcast_tx_finish(gnrc_netdev_t *gnrc_netdev){
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
	    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_INTERVAL,
	                          IQUEUEMAC_BROADCAST_INTERVAL_US);
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_WAIT_NEXT_TX;
        gnrc_netdev->gomach.need_update = false;
    }

    /* This is to handle no-TX-complete issue. In case there is no no-TX-complete event,
     * we will quit broadcasting, i.e., not getting stucked here. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_FINISH)) {
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_INTERVAL);
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_netdev->gomach.need_update = true;
    }
}
static void gomach_wait_bcast_wait_next_tx(gnrc_netdev_t *gnrc_netdev)
{
    /* Quit sending broadcast packet if we found ongoing transmissions, for collision avoidance. */
    if (!_gomach_send_bcast_busy_handle(gnrc_netdev)) {
        return;
    }

    /* If the whole broadcast duration timeouts, release the packet and go to t2u end. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_FINISH)) {
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_INTERVAL);
        gnrc_pktbuf_release(gnrc_netdev->tx.packet);
        gnrc_netdev->tx.packet = NULL;
        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    /* Toggle the radio channel and go to send the next broadcast packet. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_INTERVAL)) {
    	if(gnrc_netdev->tx.t2u_on_public_1 == true){
    	    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_2);
    	    gnrc_netdev->tx.t2u_on_public_1 = false;
    	}else{
    	   	gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
    	   	gnrc_netdev->tx.t2u_on_public_1 = true;
    	}

        gnrc_netdev->tx.bcast_state = GNRC_GOMACH_BCAST_SEND;
        gnrc_netdev->gomach.need_update = true;
    }
}

static void gomach_bcast_end(gnrc_netdev_t *gnrc_netdev)
{
    gomach_turn_off_radio(gnrc_netdev);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_INTERVAL);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_BROADCAST_FINISH);

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
    gnrc_netdev->rx.enter_new_cycle = false;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_bcast_update(gnrc_netdev_t *gnrc_netdev)
{
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
    uint32_t random_backoff = random_uint32_range(0, IQUEUEMAC_SUPERFRAME_DURATION_US);
    xtimer_usleep(random_backoff);

    gnrc_netdev->gomach.quit_current_cycle = false;
    gnrc_netdev->gomach.subchannel_occu_flags = 0;

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    /* Since devices don't broadcast beacons on default, so no need to collect beacons.
     * Go to announce its chosen sub-channel sequence. */
    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_ANNC_SUBCHAN;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_init_announce_subchannel(gnrc_netdev_t *gnrc_netdev)
{
    /* Announce the device's chosen sub-channel sequence to its neighbors. */
	gomach_bcast_subchann_seq(gnrc_netdev, NETOPT_ENABLE);

    gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_WAIT_FEEDBACK;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_init_wait_announce_feedback(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        gnrc_netdev->gomach.init_state = GNRC_GOMACH_INIT_END;
        gnrc_netdev->gomach.need_update = true;
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
    gnrc_netdev->gomach.duty_cycle_started = false;
    rtt_handler(IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE, gnrc_netdev);
    gnrc_netdev->gomach.need_update = true;
}

/* GoMacH: transmit packet to phase-known device*/
static void gomach_t2k_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Turn off radio to conserve power */
    gomach_turn_off_radio(gnrc_netdev);

    /* Turn radio onto the neighbor's public channel, which will not change in this cycle. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->tx.current_neighbor->pub_chanseq);

    gnrc_netdev->gomach.quit_current_cycle = false;

    /* Set waiting timer for the targeted device! */
    uint32_t wait_phase_duration;
    wait_phase_duration = _ticks_until_phase(gnrc_netdev,
                                             gnrc_netdev->tx.current_neighbor->cp_phase);
    wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration);

    /* Upon several times of t2k failure, we now doubt that the phase-lock may fail due to drift.
     * Here is the phase-lock auto-adjust scheme, trying to catch the neighbot's phase in case of
     * phase-lock failure due to timer drift.
     * Firstly, put the calculated phase ahead, check whether the neighbor's phase has gone ahead
     * of the recorded one */
    if(gnrc_netdev->tx.no_ack_counter == (IQUEUEMAC_REPHASELOCK_THRESHOLD -2)) {
    	if (wait_phase_duration < IQUEUEMAC_CP_DURATION_US) {
    		wait_phase_duration = (wait_phase_duration + IQUEUEMAC_SUPERFRAME_DURATION_US) -
    	                          IQUEUEMAC_CP_DURATION_US;
    	} else {
    		wait_phase_duration = wait_phase_duration - IQUEUEMAC_CP_DURATION_US;
    	}
    }
    /* If this is the last t2k trial, the phase-lock auto-adjust scheme delays the estimated phase
     *  a little bit, to see if the real phase is behind the original calculated one. */
    if(gnrc_netdev->tx.no_ack_counter == (IQUEUEMAC_REPHASELOCK_THRESHOLD -1)) {
        wait_phase_duration = wait_phase_duration + IQUEUEMAC_CP_DURATION_US +
                              IQUEUEMAC_REPHASE_ADJUST_US;
    	if (wait_phase_duration > IQUEUEMAC_SUPERFRAME_DURATION_US) {
    		wait_phase_duration = wait_phase_duration - IQUEUEMAC_SUPERFRAME_DURATION_US;
        }
    }

    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_CP, wait_phase_duration);

    /* Flush the rx-queue. */
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    gnrc_netdev->tx.tx_busy_count = 0;

    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_CP;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_t2k_wait_cp(gnrc_netdev_t *gnrc_netdev)
{
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_WAIT_CP)) {
        /* Disable auto-ack, don't try to receive packet! */
        gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);
        /* Require ACK for the packet waiting to be sent! */
        gomach_set_ack_req(gnrc_netdev, NETOPT_ENABLE);

        /* Enable csma for sending the packet! */
        netopt_enable_t csma_enable;
        csma_enable = NETOPT_ENABLE;
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_CSMA, &csma_enable,
                                      sizeof(netopt_enable_t));

        /* Set csma retry number! */
        uint8_t csma_retry_num;
        csma_retry_num = 5;
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RETRANS, &csma_retry_num,
                                      sizeof(csma_retry_num));
        gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_CSMA_RETRIES, &csma_retry_num,
                                      sizeof(csma_retry_num));

        gomach_turn_on_radio(gnrc_netdev);
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_TRANS_IN_CP;
        gnrc_netdev->gomach.need_update = true;
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
    int res = gomach_send_data_packet(gnrc_netdev, NETOPT_ENABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2k transmission fail: %d, drop packet.\n", res);

        gnrc_netdev->tx.no_ack_counter = 0;
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
        }

        gnrc_netdev->tx.current_neighbor = NULL;
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_CPTX_FEEDBACK;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_t2k_wait_cp_txfeedback(gnrc_netdev_t *gnrc_netdev)
{
     if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
        switch (gnrc_netdev_get_tx_feedback(gnrc_netdev)) {
            case TX_FEEDBACK_SUCCESS: {
                /* Since the packet will not be released by the sending function,
                 * so, here, if TX success, we first release the packet. */
                gnrc_pktbuf_release(gnrc_netdev->tx.packet);
                gnrc_netdev->tx.packet = NULL;

                /* Here is the phase-lock auto-adjust scheme. Use the new adjusted
                 * phase upon success. Here the new phase will be put ahead to the
                 * original phase. */
                if(gnrc_netdev->tx.no_ack_counter == (IQUEUEMAC_REPHASELOCK_THRESHOLD -2)) {
                    if(gnrc_netdev->tx.current_neighbor->cp_phase >=
                       RTT_US_TO_TICKS((IQUEUEMAC_CP_DURATION_US + IQUEUEMAC_REPHASE_ADJUST_US))) {
                    	gnrc_netdev->tx.current_neighbor->cp_phase -=
                    	    RTT_US_TO_TICKS((IQUEUEMAC_CP_DURATION_US+IQUEUEMAC_REPHASE_ADJUST_US));
                    } else {
                    	gnrc_netdev->tx.current_neighbor->cp_phase +=
                    	    RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
                    	gnrc_netdev->tx.current_neighbor->cp_phase -=
                    	    RTT_US_TO_TICKS((IQUEUEMAC_CP_DURATION_US+IQUEUEMAC_REPHASE_ADJUST_US));
                    }
                }
                /* Here is the phase-lock auto-adjust scheme. Use the new adjusted
                 * phase upon success. Here the new phase will be put behind the original
                 * phase. */
                if(gnrc_netdev->tx.no_ack_counter == (IQUEUEMAC_REPHASELOCK_THRESHOLD -1)) {
                	gnrc_netdev->tx.current_neighbor->cp_phase +=
                        (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US) +
                         RTT_US_TO_TICKS(4*IQUEUEMAC_REPHASE_ADJUST_US));

                	if(gnrc_netdev->tx.current_neighbor->cp_phase >=
                	   RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US)) {
                		gnrc_netdev->tx.current_neighbor->cp_phase -=
                		    RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
                	}
                }

                gnrc_netdev->tx.no_ack_counter = 0;

                /* If has pending packets, join the vTDMA period, first wait for receiver's beacon. */
                if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0) {
                    gnrc_netdev->tx.vtdma_para.get_beacon = false;
                    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON,
                                          IQUEUEMAC_WAIT_BEACON_TIME_US);
                    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_BEACON;
                }
                else {
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_netdev->gomach.need_update = true;
                break;
            }
            case TX_FEEDBACK_BUSY:
                /* If the channel busy counter is below threshold, retry CSMA immediately,
                 * by knowing that the CP will be automatically extended. */
            	if(gnrc_netdev->tx.tx_busy_count < IQUEUEMAC_MAX_TX_BUSY_COUNTER) {
            		gnrc_netdev->tx.tx_busy_count ++;

            		/* Store the TX sequence number for this packet. Always use the same
            		 * sequence number for sending the same packet, to avoid duplicated
            		 * packet reception at the receiver. */
            		netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
            		gnrc_netdev->tx.tx_seq = device_state->seq - 1;

            		gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_TRANS_IN_CP;
            		gnrc_netdev->gomach.need_update = true;
            		return;
            	}
            case TX_FEEDBACK_NOACK:
            default: {
                gnrc_netdev->tx.no_ack_counter++;

                LOG_WARNING("WARNING: [GOMACH] t2k %d times No-ACK.\n",
                            gnrc_netdev->tx.no_ack_counter);

        		/* This packet will be retried. Store the TX sequence number for this packet.
        		 * Always use the same sequence number for sending the same packet. */
                netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                /* If no_ack_counter reaches the threshold, regarded as phase-lock failed. So
                 * retry to send the packet in t2u, i.e., try to phase-lock with the receiver
                 * again. */
                if (gnrc_netdev->tx.no_ack_counter >= IQUEUEMAC_REPHASELOCK_THRESHOLD) {
                	LOG_WARNING("WARNING: [GOMACH] t2k failed, go to t2u.\n");
                	/* Here, we don't queue the packet again, but keep it in tx.packet. */
                    gnrc_netdev->tx.current_neighbor->mac_type = UNKNOWN;
                    gnrc_netdev->tx.t2u_retry_counter = 0;
                }
                else {
                    /* If no_ack_counter is below the threshold, retry sending the packet in t2k
                     * procedure in the following cycle. */

                }
                gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                gnrc_netdev->gomach.need_update = true;
                break;
            }
        }
    }
}

static void gomach_t2k_wait_beacon(gnrc_netdev_t *gnrc_netdev)
{
    /* Process the beacon if we receive it. */
    if (gnrc_netdev_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
        gomach_wait_beacon_packet_process(gnrc_netdev);
    }

    /* If we need to quit t2k, don't release the current neighbor pointer. In the
     * next cycle, we will try to send to the same receiver. */
    if (gnrc_netdev->gomach.quit_current_cycle == true) {
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON);
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    if (gnrc_netdev->tx.vtdma_para.get_beacon == true) {
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON);

        /* If the sender gets allocated slots, go to attend the receiver's vTDMA for
         * burst sending all the pending packets to the receiver. */
        if (gnrc_netdev->tx.vtdma_para.slots_num > 0) {
            /* Switch the radio to the sub-channel of the receiver. */
            gomach_turn_channel(gnrc_netdev, gnrc_netdev->tx.vtdma_para.sub_channel_seq);

            /* If the allocated slots period is not right behind the beacon, i.e., not the first
             * one, turn off the radio and wait for its own slots period. */
            if (gnrc_netdev->tx.vtdma_para.slots_position > 0) {
                gomach_turn_off_radio(gnrc_netdev);

                uint32_t wait_slots_duration = gnrc_netdev->tx.vtdma_para.slots_position *
                                               IQUEUEMAC_VTDMA_SLOT_SIZE_US;
                iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_OWN_SLOTS,
                                      wait_slots_duration);

                gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_SLOTS;
                gnrc_netdev->gomach.need_update = true;
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
                gnrc_netdev->gomach.need_update = true;
            }
        }
        else {
            /* No slots get allocated, go to t2k end. */
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
            gnrc_netdev->gomach.need_update = true;
        }
        return;
    }

    /* If no beacon during waiting period, go to t2k end. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        LOG_WARNING("WARNING: [GOMACH] t2k: no beacon.\n");
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_netdev->gomach.need_update = true;
    }
}

static void gomach_t2k_wait_own_slots(gnrc_netdev_t *gnrc_netdev)
{
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_WAIT_OWN_SLOTS)) {
    	/* The node is now in its scheduled slots period, start burst sending packets. */
        gomach_turn_on_radio(gnrc_netdev);

        gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&(gnrc_netdev->tx.current_neighbor->queue));
        if (pkt != NULL) {
            gnrc_netdev->tx.packet = pkt;
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
        }
        else {
        	LOG_ERROR("ERROR: [GOMACH] t2k vTDMA: null packet.\n");
            gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        }
        gnrc_netdev->gomach.need_update = true;
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
    int res = gomach_send_data_packet(gnrc_netdev, NETOPT_DISABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2k vTDMA transmission fail: %d, drop packet.\n", res);
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
        }
        gnrc_netdev->tx.current_neighbor = NULL;
        gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    gnrc_netdev->tx.vtdma_para.slots_num--;
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_WAIT_VTDMA_FEEDBACK;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_t2k_wait_vtdma_transfeedback(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
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
                gnrc_netdev->gomach.need_update = true;
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
                    LOG_WARNING("WARNING: [GOMACH] no ACK in vTDMA, retry in next slot.\n");
                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_VTDMA_TRANS;
                }
                else {
                	/* If no slots for sending, retry in next cycle's t2r, without releasing
                	 * tx.packet pointer. */
                    LOG_WARNING("WARNING: [GOMACH] no ACK in vTDMA, retry in next cycle.\n");

                    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_END;
                }
                gnrc_netdev->gomach.need_update = true;
                break;
            }
        }
    }
}

static void gomach_t2k_end(gnrc_netdev_t *gnrc_netdev)
{
    gomach_turn_off_radio(gnrc_netdev);

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
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_CP);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_OWN_SLOTS);

    /* Reset t2k_state to the initial state. */
    gnrc_netdev->tx.t2k_state = GNRC_GOMACH_T2K_INIT;

    /* If the sender's phase has been changed, figure out the related phase of tx-neighbors. */
    if (gnrc_netdev->gomach.phase_changed == true) {
        gomach_figure_neighbors_new_phase(gnrc_netdev);
    }

    /* Enable Auto ACK again for data reception */
    gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);

    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_netdev->rx.enter_new_cycle = false;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_t2k_update(gnrc_netdev_t *gnrc_netdev)
{
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


/****************** device state machines - Transmit to Unknown *****/
static void gomach_t2u_init(gnrc_netdev_t *gnrc_netdev)
{
    /* since t2u is right following CP period (wake-up period), the radio is still on,
     * so we don't need to turn on it again. */

	LOG_DEBUG("[GOMACH] t2u initialization.\n");

    gnrc_netdev_set_rx_started(gnrc_netdev, false);
    gnrc_netdev->gomach.quit_current_cycle = false;
    gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_netdev->tx.preamble_sent = 0;
    gnrc_netdev->tx.got_preamble_ack = false;
    gnrc_netdev->gomach.rx_memory_full = false;

    /* Disable auto-ACK here! Don't try to reply ACK to any node. */
    gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);
    /* Start sending the preamble firstly on public channel 1. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
    gnrc_netdev->tx.t2u_on_public_1 = true;

    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_t2u_send_preamble_prepare(gnrc_netdev_t *gnrc_netdev)
{
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);

    if (gnrc_netdev->tx.preamble_sent != 0) {
        /* Toggle the radio channel after each preamble transmission. */
        if (gnrc_netdev->tx.t2u_on_public_1 == true) {
            gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_2);
            gnrc_netdev->tx.t2u_on_public_1 = false;
        }
        else {
            gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.pub_channel_1);
            gnrc_netdev->tx.t2u_on_public_1 = true;
        }
        iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL,
                              IQUEUEMAC_MAX_PREAM_INTERVAL_US);
    }
    else {
        /* Here, for the first preamble, we set the pream_max_interval timeout to
         * 5*MAX_PREAM_INTERVAL due to the fact that the first preamble is
         * using csma for sending, and csma costs some time before actually sending
         * the packet. */
        iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL,
                              (5 * IQUEUEMAC_MAX_PREAM_INTERVAL_US));
    }

    gnrc_netdev->tx.reach_max_preamble_interval = false;
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_SEND_PREAMBLE;
    gnrc_netdev->gomach.need_update = true;
}

static bool _handle_in_t2u_send_preamble(gnrc_netdev_t *gnrc_netdev)
{
    /* If packet buffer is full, release one packet to release memory,
     * and reload the next packet.
     * In t2u, we need at least some minimum memory to build the preamble packet. */
    if (gnrc_netdev->gomach.rx_memory_full == true) {
        gnrc_netdev->gomach.rx_memory_full = false;

        gnrc_netdev->gomach.need_update = true;

        /* To-do: should we release all the buffered packets in the queue to
         * release memory in such a critical situation? */
        LOG_WARNING("WARNING: [GOMACH] t2u: pkt-buffer full, release one pkt.\n");

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
            LOG_WARNING("WARNING: [GOMACH] t2u: null packet, quit t2u.\n");
            gnrc_netdev->tx.current_neighbor = NULL;
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            return false;
        }
    }

    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL)) {
        gnrc_netdev->tx.reach_max_preamble_interval = true;
    }

    /* if we are receiving packet, wait until RX is completed. */
    if ((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) &&
        (gnrc_netdev->tx.reach_max_preamble_interval == false)) {
        /* Set a timeout to wait for the complete of reception. */
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
        if (gnrc_netdev->gomach.quit_current_cycle == false) {
            iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END,
                                  IQUEUEMAC_WAIT_RX_END_US);
            return false;
        }
    }

    /* if we are here, we are not receiving packet or reception is over. */
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);

    if (gnrc_netdev_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
        iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev);
    }

    /* Quit t2u if we have to, e.g., the device found ongoing bcast of other devices. */
    if (gnrc_netdev->gomach.quit_current_cycle == true) {
        LOG_WARNING("WARNING: [GOMACH] quit t2u.\n");
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_netdev->gomach.need_update = true;
        return false;
    }
    return true;
}

static void gomach_t2u_send_preamble(gnrc_netdev_t *gnrc_netdev)
{
    if (!_handle_in_t2u_send_preamble(gnrc_netdev)) {
        return;
    }

    /* If we have reached the maximum preamble interval, go to send next preamble. */
    if (gnrc_netdev->tx.reach_max_preamble_interval == true) {
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    /* Now, start sending preamble. */
    int res;
    /* The first preamble is sent with csma for collision avoidance. */
    if (gnrc_netdev->tx.preamble_sent == 0) {
        res = gomach_send_preamble(gnrc_netdev, NETOPT_ENABLE);
        iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION,
                              IQUEUEMAC_PREAMBLE_DURATION_US);
    }
    else {
        res = gomach_send_preamble(gnrc_netdev, NETOPT_DISABLE);
    }

    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2u send preamble failed: %d\n", res);
    }

    /* In case that packet-buffer is full, quit t2u and release packet. */
    if (res == -ENOBUFS) {
        LOG_ERROR("ERROR: [GOMACH] t2u: no pkt-buffer for sending preamble.\n");
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    gnrc_netdev->tx.preamble_sent++;
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_PREAMBLE_TX;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_t2u_wait_preamble_tx(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
        /* Set preamble interval timeout. This is a very short timeout (1ms),
         * just to catch the rx-start event of receiving possible preamble-ACK. */
        iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE,
                              IQUEUEMAC_PREAMBLE_INTERVAL_US);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_PREAMBLE_ACK;
        gnrc_netdev->gomach.need_update = false;
        return;
    }

    /* This is mainly to handle no-TX-complete error. Once the max preamble interval
     * timeout expired here (i.e., no-TX-complete error), we will quit waiting here
     * and go to send the next preamble, thus the MAC will not get stucked here. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL)) {
        gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
        gnrc_netdev->gomach.need_update = true;
        return;
    }
}

static void gomach_t2u_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{
    if (!_handle_in_t2u_send_preamble(gnrc_netdev)) {
        return;
    }

    if (gnrc_netdev->tx.got_preamble_ack == true) {
        /* Record the public-channel phase of the receiver. */
        if (gnrc_netdev->tx.t2u_on_public_1 == true) {
            gnrc_netdev->tx.current_neighbor->pub_chanseq = gnrc_netdev->gomach.pub_channel_1;
        }
        else {
            gnrc_netdev->tx.current_neighbor->pub_chanseq = gnrc_netdev->gomach.pub_channel_2;
        }

        /* Require ACK for the packet waiting to be sent! */
        gomach_set_ack_req(gnrc_netdev, NETOPT_ENABLE);

        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_SEND_DATA;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION)) {
        gnrc_netdev->tx.t2u_retry_counter++;

        /* If we reach the maximum t2u retry limit, release the data packet. */
        if (gnrc_netdev->tx.t2u_retry_counter >= IQUEUEMAC_T2U_RETYR_THRESHOLD) {
            LOG_WARNING("WARNING: [GOMACH] t2u failed: no preamble-ACK.\n");
            gnrc_netdev->tx.t2u_retry_counter = 0;
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE);
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);
        }
        else {
            /* If we haven't reach the maximum t2u limit, try again. Set quit_current_cycle
             * to true such that we will release the current neighbor pointer.  */
            gnrc_netdev->gomach.quit_current_cycle = true;
            gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        }

        gnrc_netdev->gomach.need_update = true;
        return;
    }

    /* If we didn't catch the RX-start event, go to send the next preamble. */
    if ((iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE)) ||
        (gnrc_netdev->tx.reach_max_preamble_interval == true)) {
        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_PREAMBLE_PREPARE;
        gnrc_netdev->gomach.need_update = true;
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
    int res;
    res = gomach_send_data_packet(gnrc_netdev, NETOPT_ENABLE);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] t2u data sending error: %d.\n", res);

        gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_WAIT_DATA_TX;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_t2u_wait_tx_feedback(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev)) {
        if (gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_SUCCESS) {
        	/* If transmission succeeded, release the data. */
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;

            gnrc_netdev->tx.no_ack_counter = 0;
            gnrc_netdev->tx.t2u_retry_counter = 0;

            /* Attend the vTDMA procedure if the sender has pending packets for the receiver. */
            if (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0) {
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;
                iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE);
                iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION);
                iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
                iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);

                /* Switch to t2k procedure and wait for the beacon of the receiver. */
                gnrc_netdev->tx.vtdma_para.get_beacon = false;
                iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_BEACON,
                                      IQUEUEMAC_WAIT_BEACON_TIME_US);
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
            if (gnrc_netdev->tx.t2u_retry_counter >= IQUEUEMAC_T2U_RETYR_THRESHOLD) {
                LOG_WARNING("WARNING: [GOMACH] t2u send data failed on channel %d,"
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
                gnrc_netdev->tx.no_ack_counter = IQUEUEMAC_REPHASELOCK_THRESHOLD;
                netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
                gnrc_netdev->tx.tx_seq = device_state->seq - 1;

                LOG_WARNING("WARNING: [GOMACH] t2u send data failed on channel %d.\n",
                            gnrc_netdev->tx.current_neighbor->pub_chanseq);
                /* Set quit_current_cycle to true, thus not to release current_neighbour pointer
                 * in t2u-end */
                gnrc_netdev->gomach.quit_current_cycle = true;
                gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_END;
            }
        }
        gnrc_netdev->gomach.need_update = true;
    }
}

static void gomach_t2u_end(gnrc_netdev_t *gnrc_netdev)
{
    gomach_turn_off_radio(gnrc_netdev);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_PREAMBLE_DURATION);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
    iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_MAX_PREAM_INTERVAL);

    /* In case quit_current_cycle is true, don't release neighbor pointer,
     * will retry t2u immediately in next cycle.*/
    if (gnrc_netdev->gomach.quit_current_cycle == false) {
        if (gnrc_netdev->tx.packet != NULL) {
            gnrc_pktbuf_release(gnrc_netdev->tx.packet);
            gnrc_netdev->tx.packet = NULL;
            gnrc_netdev->tx.no_ack_counter = 0;
            puts("drop pkt");
            LOG_WARNING("WARNING: [GOMACH] t2u: drop packet.\n");
        }
        gnrc_netdev->tx.current_neighbor = NULL;
    }

    /* Reset t2u state. */
    gnrc_netdev->tx.t2u_state = GNRC_GOMACH_T2U_INIT;

    /* If the node's phase has been changed, figure out the related phase of all neighbors. */
    if (gnrc_netdev->gomach.phase_changed == true) {
        gomach_figure_neighbors_new_phase(gnrc_netdev);
    }

    /* Resume to listen state and go to sleep. */
    gnrc_netdev->gomach.basic_state = GNRC_GOMACH_LISTEN;
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_netdev->rx.enter_new_cycle = false;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_t2u_update(gnrc_netdev_t *gnrc_netdev)
{
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
    uint32_t alarm;

    /* Execute phase backoff for avoiding CP (wake-up period) overlap. */
    rtt_clear_alarm();
    alarm = gnrc_netdev->gomach.last_wakeup +
            RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US) +
            gnrc_netdev->gomach.backoff_phase_ticks;
    rtt_set_alarm(alarm, rtt_cb, (void *) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);

    gnrc_netdev->gomach.phase_changed = true;
    LOG_INFO("INFO: [GOMACH] phase backoffed: %lu us.\n",
             RTT_TICKS_TO_US(gnrc_netdev->gomach.backoff_phase_ticks));
}

static void gomach_listen_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Reset last_seq_info, for avoiding receiving duplicate packets.
     * To-do: remove this in the future? */
    for (int i = 0; i < IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE; i++) {
        if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len != 0) {
            gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle++;
            if (gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle >=
                IQUEUEMAC_RX_CHECK_DUPPKT_UNIT_MAX_LIFE) {
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr[0] = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].node_addr.addr[1] = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].seq = 0;
                gnrc_netdev->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
            }
        }
    }

    gnrc_netdev->rx.enter_new_cycle = false;

    /* Set listen period timeout. */
    uint32_t listen_period = random_uint32_range(0, IQUEUEMAC_CP_RANDOM_END_US) +
                             IQUEUEMAC_CP_DURATION_US;
    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END, listen_period);
    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_MAX, IQUEUEMAC_CP_DURATION_MAX_US);

    /* Enable Auto-ACK for data packet reception. */
    gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);

    /* Turn to current public channel. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.cur_pub_channel);

    gnrc_netdev_set_rx_started(gnrc_netdev, false);
    gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
    gnrc_netdev->gomach.cp_backoff_counter = 0;
    gnrc_netdev->gomach.quit_current_cycle = false;
    gnrc_netdev->gomach.get_other_preamble = false;
    gnrc_netdev->gomach.send_beacon_fail = false;
    gnrc_netdev->gomach.cp_end = false;
    gnrc_netdev->gomach.got_preamble = false;
    gnrc_netdev->gomach.phase_changed = false;

    /* Flush RX queue and turn on radio. */
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    gomach_turn_on_radio(gnrc_netdev);

    /* Run phase-backoff if needed, select a new wake-up phase. */
    if (gnrc_netdev->gomach.phase_backoff == true) {
        gnrc_netdev->gomach.phase_backoff = false;
        _gomach_phase_backoff(gnrc_netdev);
    }
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_LISTEN;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_listen_cp_listen(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
        gomach_cp_packet_process(gnrc_netdev);

        /* If the device has replied a preamble-ACK, it must waits for the data.
         * Here, we extend the CP. */
        if (gnrc_netdev->gomach.got_preamble == true) {
            gnrc_netdev->gomach.got_preamble = false;
            gnrc_netdev->gomach.cp_end = false;
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END);
            iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);
        }
        else if ((gnrc_netdev->gomach.get_other_preamble == false) &&
                (gnrc_netdev->gomach.quit_current_cycle == false)) {
            gnrc_netdev->gomach.got_preamble = false;
            gnrc_netdev->gomach.cp_end = false;
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END);
            iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);
        }
    }

    /* If we have reached the maximum CP duration, quit CP. */
    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_CP_MAX)) {
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END);
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_MAX);
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_END;
        gnrc_netdev->gomach.need_update = true;
        return;
    }

    if ((iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_CP_END))) {
        gnrc_netdev->gomach.cp_end = true;
        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END);
    }

    /* If CP duration timeouted or we must quit CP, go to CP end. */
    if ((gnrc_netdev->gomach.cp_end == true) || (gnrc_netdev->gomach.quit_current_cycle == true)) {
        /* If we found ongoing reception, wait for reception complete. */
        if ((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) &&
            (gnrc_netdev->gomach.cp_backoff_counter < IQUEUEMAC_MAX_CP_BACKOFF_COUNTER)) {
            gnrc_netdev->gomach.cp_backoff_counter++;
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
            iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END,
                                  IQUEUEMAC_WAIT_RX_END_US);
        }
        else {
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_END);
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_CP_MAX);
            gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_END;
            gnrc_netdev->gomach.need_update = true;
        }
    }
}

static void gomach_listen_cp_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    _dispatch(gnrc_netdev->rx.dispatch_buffer);

    /* If we need to quit communications in this cycle, go to sleep. */
    if (gnrc_netdev->gomach.quit_current_cycle == true) {
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
    }
    else {
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SEND_BEACON;
    }
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_listen_send_beacon(gnrc_netdev_t *gnrc_netdev)
{
    /* Disable auto-ACK. Thus not to receive packet (attempt to reply ACK) anymore. */
    gomach_set_autoack(gnrc_netdev, NETOPT_DISABLE);

    /* Assemble and send the beacon. */
    int res;
    res = gomach_send_beacon(gnrc_netdev);
    if (res < 0) {
        LOG_ERROR("ERROR: [GOMACH] send beacon error: %d.\n", res);
        gnrc_netdev->gomach.send_beacon_fail = true;
        gnrc_netdev->gomach.need_update = true;
    }
    else {
        if (gnrc_netdev->rx.router_vtdma_mana.total_slots_num == 0) {
            gnrc_netdev->gomach.send_beacon_fail = true;
            gnrc_netdev->gomach.need_update = true;
        }
        else {
            gnrc_netdev->gomach.need_update = false;
        }
    }

    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_WAIT_BEACON_TX;
}

static void gomach_listen_wait_beacon_tx(gnrc_netdev_t *gnrc_netdev)
{
    if (gnrc_netdev_gomach_get_tx_finish(gnrc_netdev) ||
        (gnrc_netdev->gomach.send_beacon_fail == true)) {

        if ((gnrc_netdev->rx.router_vtdma_mana.total_slots_num > 0) &&
            (gnrc_netdev->gomach.send_beacon_fail == false)) {
            /* If the device has allocated transmission slots to other nodes,
             *  switch to vTDMA period to receive packets. */
            gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA_INIT;
            gnrc_netdev->gomach.need_update = true;
        }
        else {
            /* If the device hasn't allocated transmission slots, check whether it has packets
             * to transmit to neighbor. */
            if (gomach_find_next_tx_neighbor(gnrc_netdev)) {
                /* Now, we have packet to send. */

                if (gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]) {
                    /* The packet is for broadcasting. */

                    /* If we didn't find ongoing preamble stream, go to send broadcast packet. */
                    if (gnrc_netdev->gomach.get_other_preamble == false) {
                        gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                        gnrc_netdev->tx.transmit_state = GNRC_GOMACH_BROADCAST;
                    }
                    else {
                    	/* If we find ongoing preamble stream, go to sleep. */
                        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                    }
                    /* If the device's wakeup-phase has been changed,
                     * figure out the new phases of all neighbors. */
                    if (gnrc_netdev->gomach.phase_changed == true) {
                        gomach_figure_neighbors_new_phase(gnrc_netdev);
                    }
                }
                else {
                    /* The packet waiting to be sent is for unicast. */
                    switch (gnrc_netdev->tx.current_neighbor->mac_type) {
                        case UNKNOWN: {
                            /* The neighbor's phase is unknown yet, try to run t2u (transmission
                             * to unknown device) procedure to phase-lock the neighbor. */

                            /* If we didn't find ongoing preamble stream, go to t2u procedure. */
                            if (gnrc_netdev->gomach.get_other_preamble == false) {
                                gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                                gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;
                            }
                            else {
                                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                            }
                            break;
                        }
                        case KNOWN: {
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
                gnrc_netdev->gomach.need_update = true;
            }
            else {
                /* No packet to send, go to sleep. */
                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                gnrc_netdev->gomach.need_update = true;
            }
        }
    }
}

static void gomach_vtdma_init(gnrc_netdev_t *gnrc_netdev)
{
    /* Switch the radio to the device's sub-channel. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.sub_channel_num);

    /* Enable Auto ACK again for data reception */
    gomach_set_autoack(gnrc_netdev, NETOPT_ENABLE);

    /* Set the vTDMA period timeout. */
    uint32_t vtdma_duration = gnrc_netdev->rx.router_vtdma_mana.total_slots_num *
                              IQUEUEMAC_VTDMA_SLOT_SIZE_US;
    iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_VTDMA, vtdma_duration);

    gnrc_netdev->gomach.vtdma_end = false;

    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA;
    gnrc_netdev->gomach.need_update = false;
}

static void gomach_vtdma(gnrc_netdev_t *gnrc_netdev)
{
    /* Process received packet here. */
    if (gnrc_netdev_gomach_get_pkt_received(gnrc_netdev)) {
        gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
        iqueuemac_router_vtdma_receive_packet_process(gnrc_netdev);
    }

    if (iqueuemac_timeout_is_expired(&gnrc_netdev->gomach, TIMEOUT_VTDMA)) {
        gnrc_netdev->gomach.vtdma_end = true;
    }

    /* Go to vTDMA end after vTDMA timeout expires. */
    if (gnrc_netdev->gomach.vtdma_end == true) {
        /* Wait for reception complete if found ongoing transmission. */
        if (_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) {
            iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
            iqueuemac_set_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END,
                                  IQUEUEMAC_WAIT_RX_END_US);
            return;
        }

        iqueuemac_clear_timeout(&gnrc_netdev->gomach, TIMEOUT_WAIT_RX_END);
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_VTDMA_END;
        gnrc_netdev->gomach.need_update = true;
    }
}

static void gomach_vtdma_end(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
    _dispatch(gnrc_netdev->rx.dispatch_buffer);

    /* Switch the radio to the public-channel. */
    gomach_turn_channel(gnrc_netdev, gnrc_netdev->gomach.cur_pub_channel);

    /* Check if there is packet to send. */
    if (gomach_find_next_tx_neighbor(gnrc_netdev)) {
        if (gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]) {
        	/* The packet is for broadcasting. */
            if (gnrc_netdev->gomach.get_other_preamble == false) {
                gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                gnrc_netdev->tx.transmit_state = GNRC_GOMACH_BROADCAST;
            }
            else {
                gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
            }

            /* If the device's wakeup-phase has been changed,
             * figure out the new phases of all neighbors. */
            if (gnrc_netdev->gomach.phase_changed == true) {
                gomach_figure_neighbors_new_phase(gnrc_netdev);
            }
        }
        else {
            switch (gnrc_netdev->tx.current_neighbor->mac_type) {
                /* The packet waiting to be sent is for unicast. */
                case UNKNOWN: {
                    /* The neighbor's phase is unknown yet, try to run t2u (transmission
                     * to unknown device) procedure to phase-lock the neighbor. */
                    if (gnrc_netdev->gomach.get_other_preamble == false) {
                        gnrc_netdev->gomach.basic_state = GNRC_GOMACH_TRANSMIT;
                        gnrc_netdev->tx.transmit_state = GNRC_GOMACH_TRANS_TO_UNKNOWN;
                    }
                    else {
                        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_INIT;
                    }
                } break;
                case KNOWN: {
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

    gnrc_netdev->gomach.need_update = true;
}

static void gomach_sleep_init(gnrc_netdev_t *gnrc_netdev)
{
    /* If the device's wakeup-phase has been changed,
     * figure out the new phases of all neighbors. */
    if (gnrc_netdev->gomach.phase_changed == true) {
        gomach_figure_neighbors_new_phase(gnrc_netdev);
    }

    /* Turn off the radio during sleep period to conserve power. */
    gomach_turn_off_radio(gnrc_netdev);
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_sleep(gnrc_netdev_t *gnrc_netdev)
{
    /* If we are entering a new cycle, quit sleeping. */
    if (gnrc_netdev->rx.enter_new_cycle == true) {
        gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_SLEEP_END;
        gnrc_netdev->gomach.need_update = true;
    }
}

static void gomach_sleep_end(gnrc_netdev_t *gnrc_netdev)
{
   /* Go to CP (start of the new cycle), start listening on the public-channel. */
    gnrc_netdev->rx.listen_state = GNRC_GOMACH_LISTEN_CP_INIT;
    gnrc_netdev->gomach.need_update = true;
}

static void gomach_update(gnrc_netdev_t *gnrc_netdev)
{
    switch (gnrc_netdev->gomach.basic_state) {
        case GNRC_GOMACH_INIT: {
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
 * @param[in] data      optional parameter
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *) dev->context;

    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = (void *) gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch (event) {

            case NETDEV_EVENT_RX_STARTED:
                gnrc_netdev_set_rx_started(gnrc_netdev, true);
                //puts("gomach: rx-started event triggered.");
                gnrc_netdev->gomach.need_update = true;
                break;

            case NETDEV_EVENT_RX_COMPLETE:
            {
                gnrc_netdev->gomach.need_update = true;

                gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                if (pkt == NULL) {
                    gnrc_netdev->gomach.rx_memory_full = true;
                    puts("rx: pkt is NULL, memory full?");
                    gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
                    gnrc_netdev_set_rx_started(gnrc_netdev, false);
                    break;
                }

                if (!gnrc_netdev_get_rx_started(gnrc_netdev)) {
                    //LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
                    puts("rx_pkt corrupted?");
                    gnrc_pktbuf_release(pkt);
                    gnrc_netdev_set_rx_started(gnrc_netdev, false);
                    break;
                }

                gnrc_netdev_set_rx_started(gnrc_netdev, false);

                /* update the seq to avoid duplicate pkt.
                   gnrc_netif_hdr_t* netif_hdr;
                   netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
                   //printf("gomach: the received packet rssi is: %d .\n", netif_hdr->rssi);
                   gomach.rx.last_seq_info.seq = netif_hdr->seq;
                 */

                if (!gnrc_mac_queue_rx_packet(&gnrc_netdev->rx, 0, pkt)) {
                    //LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
                    puts("can't push rx-pkt, memory full?");
                    gnrc_pktbuf_release(pkt);
                    gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, false);
                    break;
                }
                else {
                	gnrc_netdev_gomach_set_pkt_received(gnrc_netdev, true);
                }
            } break;

            case NETDEV_EVENT_TX_COMPLETE: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_SUCCESS);
                gnrc_netdev_gomach_set_tx_finish(gnrc_netdev, true);
                iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
                gnrc_netdev->gomach.need_update = true;
            } break;

            case NETDEV_EVENT_TX_NOACK: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_NOACK);
                gnrc_netdev_gomach_set_tx_finish(gnrc_netdev, true);
                iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
                gnrc_netdev->gomach.need_update = true;
            } break;

            case NETDEV_EVENT_TX_MEDIUM_BUSY: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_BUSY);
                gnrc_netdev_gomach_set_tx_finish(gnrc_netdev, true);
                iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
                gnrc_netdev->gomach.need_update = true;
            } break;
/*
            case NETDEV_EVENT_TX_STARTED:{
                if(tx.got_preamble_ack == true){
                  puts("gomach: data packet transmission tx started!");
                 }
            }break;*/

#ifdef MODULE_NETSTATS_L2
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default: break;

                DEBUG("gnrc_netdev: warning: unhandled event %u.\n", event);
        }
    }
}

/**
 * @brief   Startup code and event loop of the gnrc_netdev layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_iqueuemac_thread(void *args)
{

    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *)args;
    netdev_t *dev = gnrc_netdev->dev;

    /**************************************origin*************************************/
    DEBUG("gnrc_netdev: starting thread\n");
    //gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) args;
    //netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV_NETAPI_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV_NETAPI_MSG_QUEUE_SIZE);
    /***************************************origin************************************/

    /*************************************iqueue-mac**************************************/
    /* RTT is used for scheduling wakeup */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send msg */
    gnrc_netdev->gomach.pid = thread_getpid();
    iqueuemac_pid = thread_getpid();
    /*************************************iqueue-mac**************************************/

    /***************************************************************************/
    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void *) gnrc_netdev;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);
    /***************************************************************************/

    xtimer_sleep(5);

    uint16_t src_len = 8;
    dev->driver->set(dev, NETOPT_SRC_LEN, &src_len, sizeof(src_len));

    iqueuemac_init(gnrc_netdev);

    uint32_t seed;
    seed = (uint32_t)gnrc_netdev->l2_addr[0];

    seed = 0;
    seed = gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len-2];
    seed = seed << 8;
    seed |= gnrc_netdev->l2_addr[gnrc_netdev->l2_addr_len-1];

    random_init(seed);

    gnrc_netdev->gomach.need_update = true;

    /* start the event loop */
    while (1) {
        DEBUG("gnrc_netdev: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                      netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                      netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;

            /**************************************iqueue-mac********************************************/
            case IQUEUEMAC_EVENT_RTT_TYPE: {
                rtt_handler(msg.content.value, gnrc_netdev);
            } break;

            case IQUEUEMAC_EVENT_TIMEOUT_TYPE: {
                // printf("gomach: Hitting a timeout event.\n");
                iqueuemac_timeout_make_expire((gomach_timeout_t *) msg.content.ptr);
                gnrc_netdev->gomach.need_update = true;
            } break;

            case GNRC_NETAPI_MSG_TYPE_SND: {
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SND received\n");

                gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)msg.content.ptr;

                if (!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, pkt)) {
                    LOG(LOG_WARNING, "WARNING: [GoMacH] TX queue full, drop packet\n");
                    gnrc_pktbuf_release(pkt);
                }
                gnrc_netdev->gomach.need_update = true;

            } break;
            /**************************************iqueue-mac********************************************/

            default:
                DEBUG("gnrc_netdev: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }

        while (gnrc_netdev->gomach.need_update == true) {
            gnrc_netdev->gomach.need_update = false;
            gomach_update(gnrc_netdev);
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_iqueuemac_init(char *stack, int stacksize, char priority,
                                 const char *name, gnrc_netdev_t *gnrc_netdev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev == NULL || gnrc_netdev->dev == NULL) {
        return -ENODEV;
    }

    /* create new gnrc_netdev thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                        _gnrc_iqueuemac_thread, (void *)gnrc_netdev, name);
    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}

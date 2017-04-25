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
 * @brief       Implementation of the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
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
#include "timex.h"
#include "random.h"
#include "periph/rtt.h"
#include "net/gnrc.h"
#include "net/netdev.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/lwmac/types.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/mac/internal.h"
#include "net/gnrc/lwmac/timeout.h"
#include "include/tx_state_machine.h"
#include "include/rx_state_machine.h"
#include "include/lwmac_internal.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define LOG_LEVEL LOG_WARNING
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [lwmac] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [lwmac] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "[lwmac] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "[lwmac] " __VA_ARGS__)

/**
 * @brief  Lwmac thread's PID
 */
kernel_pid_t lwmac_pid;

static void rtt_cb(void *arg);
static bool lwmac_update(gnrc_netdev_t *gnrc_netdev);
static void lwmac_set_state(gnrc_netdev_t *gnrc_netdev, lwmac_state_t newstate);
static void lwmac_schedule_update(gnrc_netdev_t *gnrc_netdev);
static bool lwmac_needs_update(gnrc_netdev_t *gnrc_netdev);
static void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev);

inline void lwmac_schedule_update(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_netdev->lwmac.needs_rescheduling = true;
}

inline bool lwmac_needs_update(gnrc_netdev_t *gnrc_netdev)
{
    return gnrc_netdev->lwmac.needs_rescheduling;
}

void lwmac_set_state(gnrc_netdev_t *gnrc_netdev, lwmac_state_t newstate)
{
    lwmac_state_t oldstate = gnrc_netdev->lwmac.state;

    if (newstate == oldstate) {
        return;
    }

    if (newstate >= STATE_COUNT) {
        LOG_ERROR("Trying to set invalid state %u\n", newstate);
        return;
    }

    /* Already change state, but might be reverted to oldstate when needed */
    gnrc_netdev->lwmac.state = newstate;

    /* Actions when leaving old state */
    switch (oldstate) {
        case RECEIVING:
        case TRANSMITTING: {
            /* Enable duty cycling again */
            rtt_handler(LWMAC_EVENT_RTT_RESUME, gnrc_netdev);
#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
            /* Output duty-cycle ratio */
            uint64_t duty;
            duty = (uint64_t) rtt_get_counter();
            duty = ((uint64_t) gnrc_netdev->lwmac.awake_duration_sum_ticks) * 100 /
                   (duty - (uint64_t)gnrc_netdev->lwmac.system_start_time_ticks);
            printf("[lwmac-tx]: achieved duty-cycle: %lu %% \n", (uint32_t)duty);
#endif
            break;
        }
        case SLEEPING: {
            lwmac_clear_timeout(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD);
            break;
        }
        default:
            break;
    }

    /* Actions when entering new state */
    switch (newstate) {
        /*********************** Operation states *********************************/
        case LISTENING: {
            _set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);
            break;
        }
        case SLEEPING: {
            /* Put transceiver to sleep */
            _set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
            /* We may have come here through RTT handler, so timeout may still be active */
            lwmac_clear_timeout(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD);

            if (gnrc_netdev2_get_phase_backoff(gnrc_netdev)) {
                gnrc_netdev2_set_phase_backoff(gnrc_netdev, false);
                uint32_t alarm;

                rtt_clear_alarm();
                alarm = random_uint32_range(RTT_US_TO_TICKS((3 * LWMAC_WAKEUP_DURATION_US / 2)),
                                            RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US -
                                                            (3 * LWMAC_WAKEUP_DURATION_US / 2)));
                LOG_WARNING("phase backoffed: %lu us\n", RTT_TICKS_TO_US(alarm));
                gnrc_netdev->lwmac.last_wakeup = gnrc_netdev->lwmac.last_wakeup + alarm;
                alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
                                            RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
                rtt_set_alarm(alarm, rtt_cb, (void *) LWMAC_EVENT_RTT_WAKEUP_PENDING);
            }

            /* Return immediately, so no rescheduling */
            return;
        }
        /* Trying to send data */
        case TRANSMITTING: {
            rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);    /**< No duty cycling while RXing */
            _set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);  /**< Power up netdev */
            break;
        }
        /* Receiving incoming data */
        case RECEIVING: {
            rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);    /**< No duty cycling while TXing */
            _set_netdev_state(gnrc_netdev, NETOPT_STATE_IDLE);  /**< Power up netdev */
            break;
        }
        case STOPPED: {
            _set_netdev_state(gnrc_netdev, NETOPT_STATE_OFF);
            break;
        }
        /*********************** Control states ***********************************/
        case START: {
            rtt_handler(LWMAC_EVENT_RTT_START, gnrc_netdev);
            lwmac_set_state(gnrc_netdev, LISTENING);
            break;
        }
        case STOP: {
            rtt_handler(LWMAC_EVENT_RTT_STOP, gnrc_netdev);
            lwmac_set_state(gnrc_netdev, STOPPED);
            break;
        }
        case RESET: {
            LOG_WARNING("Reset not yet implemented\n");
            lwmac_set_state(gnrc_netdev, STOP);
            lwmac_set_state(gnrc_netdev, START);
            break;
        }
        /**************************************************************************/
        default: {
            LOG_DEBUG("No actions for entering state %u\n", newstate);
            return;
        }
    }

    lwmac_schedule_update(gnrc_netdev);
}

/* Main state machine. Call whenever something happens */
bool lwmac_update(gnrc_netdev_t *gnrc_netdev)
{
    gnrc_netdev->lwmac.needs_rescheduling = false;

    switch (gnrc_netdev->lwmac.state) {
        case SLEEPING: {
            if (gnrc_netdev2_get_quit_tx(gnrc_netdev)) {
                return false;
            }

            /* If a packet is scheduled, no other (possible earlier) packet can be
             * sent before the first one is handled, even no broadcast
             */
            if (!lwmac_timeout_is_running(gnrc_netdev, TIMEOUT_WAIT_FOR_DEST_WAKEUP)) {
                gnrc_mac_tx_neighbor_t *neighbour;

                /* Check if there is packet remaining for retransmission */
                if (gnrc_netdev->tx.current_neighbor != NULL) {
                    neighbour = gnrc_netdev->tx.current_neighbor;
                }
                else {
                    /* Check if there are broadcasts to send and transmit immediately */
                    if (gnrc_priority_pktqueue_length(&(gnrc_netdev->tx.neighbors[0].queue)) > 0) {
                        gnrc_netdev->tx.current_neighbor = &(gnrc_netdev->tx.neighbors[0]);
                        lwmac_set_state(gnrc_netdev, TRANSMITTING);
                        break;
                    }
                    neighbour = _next_tx_neighbor(gnrc_netdev);
                }

                if (neighbour != NULL) {
                    /* if phase is unknown, send immediately after wakeup period. */
                    if (neighbour->phase > RTT_TICKS_TO_US(LWMAC_WAKEUP_INTERVAL_US)) {
                        gnrc_netdev->tx.current_neighbor = neighbour;
                        gnrc_netdev2_set_tx_continue(gnrc_netdev, false);
                        gnrc_netdev->tx.tx_burst_count = 0;
                        lwmac_set_state(gnrc_netdev, TRANSMITTING);
                        break;
                    }

                    /* Offset in microseconds when the earliest (phase) destination
                     * node wakes up that we have packets for. */
                    int time_until_tx = RTT_TICKS_TO_US(_ticks_until_phase(neighbour->phase));

                    /* If there's not enough time to prepare a WR to catch the phase
                     * postpone to next interval */
                    if (time_until_tx < LWMAC_WR_PREPARATION_US) {
                        time_until_tx += LWMAC_WAKEUP_INTERVAL_US;
                    }

                    time_until_tx -= LWMAC_WR_PREPARATION_US;

                    /* add a random time before goto TX, for avoiding one node for always holding the medium */
                    uint32_t random_backoff;
                    random_backoff = random_uint32_range(0, LWMAC_TIME_BETWEEN_WR_US);
                    time_until_tx = time_until_tx + random_backoff;

                    lwmac_set_timeout(gnrc_netdev, TIMEOUT_WAIT_FOR_DEST_WAKEUP, time_until_tx);

                    /* Register neighbour to be the next */
                    gnrc_netdev->tx.current_neighbor = neighbour;

                    /* Stop dutycycling, we're preparing to send. This prevents the
                     * timeout arriving late, so that the destination phase would
                     * be missed. */
                    // TODO: bad for power savings
                    rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);
                }
            }
            else {
                if (lwmac_timeout_is_expired(gnrc_netdev, TIMEOUT_WAIT_FOR_DEST_WAKEUP)) {
                    LOG_DEBUG("Got timeout for dest wakeup, ticks: %" PRIu32 "\n",
                              rtt_get_counter());
                    gnrc_netdev2_set_tx_continue(gnrc_netdev, false);
                    gnrc_netdev->tx.tx_burst_count = 0;
                    lwmac_set_state(gnrc_netdev, TRANSMITTING);
                }
            }
            break;
        }
        case LISTENING: {
            if ((_next_tx_neighbor(gnrc_netdev) != NULL) ||
                (gnrc_netdev->tx.current_neighbor != NULL)) {
                rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev);
            }

            /* Set timeout for if there's no successful rx transaction that will
             * change state to SLEEPING. */
            if (!lwmac_timeout_is_running(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD)) {
                lwmac_set_timeout(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD, LWMAC_WAKEUP_DURATION_US);
            }
            else if (lwmac_timeout_is_expired(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD)) {
                /* Dispatch first as there still may be broadcast packets. */
                _dispatch(gnrc_netdev->rx.dispatch_buffer);

                gnrc_netdev->lwmac.state = SLEEPING;
                /* Enable duty cycling again */
                rtt_handler(LWMAC_EVENT_RTT_RESUME, gnrc_netdev);

                _set_netdev_state(gnrc_netdev, NETOPT_STATE_SLEEP);
                lwmac_clear_timeout(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD);

                /* if there is a packet for transmission, schedule update. */
                gnrc_mac_tx_neighbor_t *neighbour = _next_tx_neighbor(gnrc_netdev);
                if ((neighbour != NULL) || (gnrc_netdev->tx.current_neighbor != NULL)) {
                    /* This triggers packet sending procedure in sleeping immediately. */
                    lwmac_schedule_update(gnrc_netdev);
                    break;
                }
            }

            if (gnrc_priority_pktqueue_length(&gnrc_netdev->rx.queue) > 0) {
                /* Do wakeup extension after packet reception. */
                lwmac_clear_timeout(gnrc_netdev, TIMEOUT_WAKEUP_PERIOD);
                lwmac_set_state(gnrc_netdev, RECEIVING);
            }
            break;
        }
        case RECEIVING: {
            lwmac_rx_state_t state_rx = gnrc_netdev->rx.state;

            switch (state_rx) {
                case RX_STATE_STOPPED: {
                    lwmac_rx_start(gnrc_netdev);
                    lwmac_rx_update(gnrc_netdev);
                    break;
                }
                case RX_STATE_FAILED: {
                    /* This may happen frequently because we'll receive WA from
                     * every node in range. */
                    LOG_DEBUG("Reception was NOT successful\n");
                    lwmac_rx_stop(gnrc_netdev);

                    if (gnrc_netdev->rx.rx_exten_count >= LWMAC_MAX_RX_EXTENSION_NUM) {
                        gnrc_netdev2_set_quit_rx(gnrc_netdev, true);
                    }

                    if (gnrc_netdev2_get_quit_rx(gnrc_netdev)) {
                        lwmac_set_state(gnrc_netdev, SLEEPING);
                    }
                    else {
                        /* Restart */
                        lwmac_set_state(gnrc_netdev, LISTENING);
                    }
                    break;
                }
                case RX_STATE_SUCCESSFUL: {
                    LOG_DEBUG("Reception was successful\n");
                    lwmac_rx_stop(gnrc_netdev);
                    /* Dispatch received packets, timing is not critical anymore */
                    _dispatch(gnrc_netdev->rx.dispatch_buffer);

                    if (gnrc_netdev2_get_quit_rx(gnrc_netdev)) {
                        lwmac_set_state(gnrc_netdev, SLEEPING);
                    }
                    else {
                        /* Go back to Listen after successful transaction */
                        lwmac_set_state(gnrc_netdev, LISTENING);
                    }
                    break;
                }
                default:
                    lwmac_rx_update(gnrc_netdev);
            }

            /* If state has changed, reschedule main state machine */
            if (state_rx != gnrc_netdev->rx.state) {
                lwmac_schedule_update(gnrc_netdev);
            }
            break;
        }
        case TRANSMITTING: {
            char *tx_success = "";
            lwmac_tx_state_t state_tx = gnrc_netdev->tx.state;

            switch (state_tx) {
                case TX_STATE_STOPPED: {
                    gnrc_pktsnip_t *pkt;

                    if (gnrc_netdev->tx.packet != NULL) {
                        LOG_WARNING("TX %d times retry\n", gnrc_netdev->tx.tx_retry_count);
                        gnrc_netdev->tx.state = TX_STATE_INIT;
                        gnrc_netdev->tx.wr_sent = 0;
                        lwmac_tx_update(gnrc_netdev);
                    }
                    else {
                        if ((pkt = gnrc_priority_pktqueue_pop(
                                 &gnrc_netdev->tx.current_neighbor->queue))) {
                            gnrc_netdev->tx.tx_retry_count = 0;
                            lwmac_tx_start(gnrc_netdev, pkt, gnrc_netdev->tx.current_neighbor);
                            lwmac_tx_update(gnrc_netdev);
                        }
                        else {
                            /* Shouldn't happen, but never observed this case */
                            int id = (gnrc_netdev->tx.current_neighbor -
                                      gnrc_netdev->tx.neighbors);
                            id /= sizeof(gnrc_netdev->tx.current_neighbor);
                            LOG_ERROR("Packet from neighbour's queue (#%d) invalid\n", id);
                            lwmac_schedule_update(gnrc_netdev);
                        }
                    }
                    break;
                }
                case TX_STATE_FAILED: {
                    gnrc_netdev2_set_tx_continue(gnrc_netdev, false);
                    gnrc_netdev2_set_quit_tx(gnrc_netdev, true);
                    tx_success = "NOT ";
                    /* Intended fall-through, TX packet will therefore be dropped. No
                     * automatic resending here, we did our best.
                     */
                }
                case TX_STATE_SUCCESSFUL: {
                    if (gnrc_netdev->tx.current_neighbor == &(gnrc_netdev->tx.neighbors[0])) {
                        LOG_INFO("Broadcast transmission done\n");
                    }
                    else {
                        LOG_INFO("Transmission was %ssuccessful (%" PRIu32 " WRs sent)\n",
                                 tx_success, gnrc_netdev->tx.wr_sent);
                    }
                    lwmac_tx_stop(gnrc_netdev);

                    if ((gnrc_netdev2_get_tx_continue(gnrc_netdev)) &&
                        (gnrc_netdev->tx.tx_burst_count < LWMAC_MAX_TX_BURST_PKT_NUM)) {
                        lwmac_schedule_update(gnrc_netdev);
                    }
                    else {
                        lwmac_set_state(gnrc_netdev, SLEEPING);
                    }
                    break;
                }
                default:
                    lwmac_tx_update(gnrc_netdev);
            }

            /* If state has changed, reschedule main state machine */
            if (state_tx != gnrc_netdev->tx.state) {
                lwmac_schedule_update(gnrc_netdev);
            }
            break;
        }
        default:
            LOG_DEBUG("No actions in state %u\n", gnrc_netdev->lwmac.state);
    }

    return gnrc_netdev->lwmac.needs_rescheduling;
}

static void rtt_cb(void *arg)
{
    msg_t msg;

    msg.content.value = ((uint32_t) arg) & 0xffff;
    msg.type = LWMAC_EVENT_RTT_TYPE;
    msg_send(&msg, lwmac_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    uint32_t alarm;

    switch (event & 0xffff) {
        case LWMAC_EVENT_RTT_WAKEUP_PENDING: {
            gnrc_netdev->lwmac.last_wakeup = rtt_get_alarm();
            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
                                        RTT_US_TO_TICKS(LWMAC_WAKEUP_DURATION_US));
            rtt_set_alarm(alarm, rtt_cb, (void *) LWMAC_EVENT_RTT_SLEEP_PENDING);
            gnrc_netdev2_set_quit_tx(gnrc_netdev, false);
            gnrc_netdev2_set_quit_rx(gnrc_netdev, false);
            gnrc_netdev2_set_phase_backoff(gnrc_netdev, false);
            gnrc_netdev->rx.rx_exten_count = 0;
            lwmac_set_state(gnrc_netdev, LISTENING);
            break;
        }
        case LWMAC_EVENT_RTT_SLEEP_PENDING: {
            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
                                        RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
            rtt_set_alarm(alarm, rtt_cb, (void *) LWMAC_EVENT_RTT_WAKEUP_PENDING);
            lwmac_set_state(gnrc_netdev, SLEEPING);
            break;
        }
        /* Set initial wakeup alarm that starts the cycle */
        case LWMAC_EVENT_RTT_START: {
            LOG_DEBUG("RTT: Initialize duty cycling\n");
            alarm = rtt_get_counter() + RTT_US_TO_TICKS(LWMAC_WAKEUP_DURATION_US);
            rtt_set_alarm(alarm, rtt_cb, (void *) LWMAC_EVENT_RTT_SLEEP_PENDING);
            gnrc_netdev->lwmac.dutycycling_active = true;
            break;
        }
        case LWMAC_EVENT_RTT_STOP:
        case LWMAC_EVENT_RTT_PAUSE: {
            rtt_clear_alarm();
            LOG_DEBUG("RTT: Stop duty cycling, now in state %u\n", gnrc_netdev->lwmac.state);
            gnrc_netdev->lwmac.dutycycling_active = false;
            break;
        }
        case LWMAC_EVENT_RTT_RESUME: {
            LOG_DEBUG("RTT: Resume duty cycling\n");
            alarm = _next_inphase_event(gnrc_netdev->lwmac.last_wakeup,
                                        RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
            rtt_set_alarm(alarm, rtt_cb, (void *) LWMAC_EVENT_RTT_WAKEUP_PENDING);
            gnrc_netdev->lwmac.dutycycling_active = true;
            break;
        }
        default:
            break;
    }
}

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *) dev->context;

    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = (void *) gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            LOG_WARNING("gnrc_netdev: possibly lost interrupt.\n");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch (event) {
            case NETDEV_EVENT_RX_STARTED: {
                LOG_DEBUG("NETDEV_EVENT_RX_STARTED\n");
                gnrc_netdev_set_rx_started(gnrc_netdev, true);
                break;
            }
            case NETDEV_EVENT_RX_COMPLETE: {
                LOG_DEBUG("NETDEV_EVENT_RX_COMPLETE\n");

                gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                /* Prevent packet corruption when a packet is sent before the previous
                 * received packet has been downloaded. This happens e.g. when a timeout
                 * expires that causes the tx state machine to send a packet. When a
                 * packet arrives after the timeout, the notification is queued but the
                 * tx state machine continues to send and then destroys the received
                 * packet in the frame buffer. After completion, the queued notification
                 * will be handled a corrupted packet will be downloaded. Therefore
                 * keep track that RX_STARTED is followed by RX_COMPLETE.
                 *
                 * TODO: transceivers might have 2 frame buffers, so make this optional
                 */
                if (pkt == NULL) {
                    gnrc_netdev_set_rx_started(gnrc_netdev, false);
                    break;
                }

                /*
                   if (!gnrc_netdev2_get_rx_started(gnrc_netdev)) {
                    LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
                    gnrc_pktbuf_release(pkt);
                    gnrc_netdev2_set_rx_started(gnrc_netdev,false);
                    break;
                   }
                 */

                gnrc_netdev_set_rx_started(gnrc_netdev, false);

                if (!gnrc_mac_queue_rx_packet(&gnrc_netdev->rx, 0, pkt)) {
                    LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
                    gnrc_pktbuf_release(pkt);
                    break;
                }
                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            case NETDEV_EVENT_TX_STARTED: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_UNDEF);
                gnrc_netdev_set_rx_started(gnrc_netdev, false);
                break;
            }
            case NETDEV_EVENT_TX_COMPLETE: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_SUCCESS);
                gnrc_netdev_set_rx_started(gnrc_netdev, false);
                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            case NETDEV_EVENT_TX_NOACK: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_NOACK);
                gnrc_netdev_set_rx_started(gnrc_netdev, false);
                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            case NETDEV_EVENT_TX_MEDIUM_BUSY: {
                gnrc_netdev_set_tx_feedback(gnrc_netdev, TX_FEEDBACK_BUSY);
                gnrc_netdev_set_rx_started(gnrc_netdev, false);
                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            default:
                LOG_WARNING("Unhandled netdev event: %u\n", event);
        }
    }
}

/**
 * @brief   Startup code and event loop of the LWMAC layer
 *
 * @param[in] args          expects a pointer to the underlying netdev device
 *
 * @return                  never returns
 */
static void *_lwmac_thread(void *args)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t *)args;
    netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[LWMAC_IPC_MSG_QUEUE_SIZE];

    LOG_INFO("Starting lwMAC\n");

    /* RTT is used for scheduling wakeup */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send msg */
    lwmac_pid = thread_getpid();

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, LWMAC_IPC_MSG_QUEUE_SIZE);

    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void *) gnrc_netdev;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);

    /* Enable RX- and TX-started interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    dev->driver->set(dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    dev->driver->set(dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    dev->driver->set(dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    uint16_t src_len = 8;
    dev->driver->set(dev, NETOPT_SRC_LEN, &src_len, sizeof(src_len));

    /* Get own address from netdev */
    gnrc_netdev->l2_addr_len = dev->driver->get(dev, NETOPT_ADDRESS_LONG,
                                                &gnrc_netdev->l2_addr,
                                                IEEE802154_LONG_ADDRESS_LEN);
    assert(gnrc_netdev->l2_addr_len > 0);

    /* Initialize broadcast sequence number. This at least differs from board
     * to board */
    gnrc_netdev->tx.bcast_seqnr = gnrc_netdev->l2_addr[0];

    /* Reset all timeouts just to be sure */
    lwmac_reset_timeouts(gnrc_netdev);

    /* Start duty cycling */
    lwmac_set_state(gnrc_netdev, START);

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
    /* Start duty cycle recording */
    gnrc_netdev->lwmac.system_start_time_ticks = rtt_get_counter();
    gnrc_netdev->lwmac.last_radio_on_time_ticks = gnrc_netdev->lwmac.system_start_time_ticks;
    gnrc_netdev->lwmac.awake_duration_sum_ticks = 0;
    gnrc_netdev->lwmac.radio_is_on = true;
#endif

    /* start the event loop */
    while (1) {
        msg_receive(&msg);

        /* Handle NETDEV, NETAPI, RTT and TIMEOUT messages */
        switch (msg.type) {
            /* RTT raised an interrupt */
            case LWMAC_EVENT_RTT_TYPE: {
                if (gnrc_netdev->lwmac.dutycycling_active) {
                    rtt_handler(msg.content.value, gnrc_netdev);
                    lwmac_schedule_update(gnrc_netdev);
                }
                else {
                    LOG_DEBUG("Ignoring late RTT event while dutycycling is off\n");
                }
                break;
            }
            /* An lwmac timeout occured */
            case LWMAC_EVENT_TIMEOUT_TYPE: {
                lwmac_timeout_make_expire((lwmac_timeout_t *) msg.content.ptr);
                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            /* Transceiver raised an interrupt */
            case NETDEV_MSG_TYPE_EVENT: {
                LOG_DEBUG("GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                /* Forward event back to driver */
                dev->driver->isr(dev);
                break;
            }
            /* TX: Queue for sending */
            case GNRC_NETAPI_MSG_TYPE_SND: {
                // TODO: how to announce failure to upper layers?
                LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SND received\n");
                gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *) msg.content.ptr;

                if (!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, pkt)) {
                    gnrc_pktbuf_release(pkt);
                    LOG_WARNING("TX queue full, drop packet\n");
                }

                lwmac_schedule_update(gnrc_netdev);
                break;
            }
            /* NETAPI set/get. Can't this be refactored away from here? */
            case GNRC_NETAPI_MSG_TYPE_SET: {
                LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SET received\n");
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;

                /* Depending on option forward to NETDEV or handle here */
                switch (opt->opt) {
                    /* Handle state change requests */
                    case NETOPT_STATE: {
                        netopt_state_t *state = (netopt_state_t *) opt->data;
                        res = opt->data_len;
                        switch (*state) {
                            case NETOPT_STATE_OFF: {
                                lwmac_set_state(gnrc_netdev, STOP);
                                break;
                            }
                            case NETOPT_STATE_IDLE: {
                                lwmac_set_state(gnrc_netdev, START);
                                break;
                            }
                            case NETOPT_STATE_RESET: {
                                lwmac_set_state(gnrc_netdev, RESET);
                                break;
                            }
                            default:
                                res = -EINVAL;
                                LOG_ERROR("NETAPI tries to set unsupported state %u\n",
                                          *state);
                        }
                        lwmac_schedule_update(gnrc_netdev);
                        break;
                    }
                    /* Forward to netdev by default*/
                    default:
                        /* set option for device driver */
                        res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                        LOG_DEBUG("Response of netdev->set: %i\n", res);
                }

                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            }
            case GNRC_NETAPI_MSG_TYPE_GET: {
                /* TODO: filter out MAC layer options -> for now forward
                         everything to the device driver */
                LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_GET received\n");
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                LOG_DEBUG("Response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            }
            default:
                LOG_ERROR("Unknown command %" PRIu16 "\n", msg.type);
                break;
        }

        /* Execute main state machine because something just happend*/
        while (lwmac_needs_update(gnrc_netdev)) {
            lwmac_update(gnrc_netdev);
        }
    }

    LOG_ERROR("terminated\n");

    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_lwmac_init(char *stack, int stacksize, char priority,
                             const char *name, gnrc_netdev_t *dev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (dev == NULL || dev->dev == NULL) {
        LOG_ERROR("No netdev supplied or driver not set\n");
        return -ENODEV;
    }

    /* create new LWMAC thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                        _lwmac_thread, (void *)dev, name);
    if (res <= 0) {
        LOG_ERROR("Couldn't create thread\n");
        return -EINVAL;
    }

    return res;
}

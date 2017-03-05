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

#include <kernel_types.h>
//#include <lpm.h>
#include <msg.h>
#include <thread.h>
#include <timex.h>
#include <periph/rtt.h>
#include <net/gnrc.h>
#include <net/netdev2.h>
#include <net/gnrc/netdev2.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <net/gnrc/lwmac/types.h>
#include <net/gnrc/mac/internal.h>


#include "include/tx_state_machine.h"
#include "include/rx_state_machine.h"
#include "include/lwmac_internal.h"
#include <net/gnrc/lwmac/timeout.h>

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

/* Internal state of lwMAC */
//static lwmac_t lwmac = LWMAC_INIT;
kernel_pid_t lwmac_pid;

static bool lwmac_update(gnrc_netdev2_t* gnrc_netdev2);
static void lwmac_set_state(gnrc_netdev2_t* gnrc_netdev2, lwmac_state_t newstate);
static void lwmac_schedule_update(gnrc_netdev2_t* gnrc_netdev2);
static bool lwmac_needs_update(gnrc_netdev2_t* gnrc_netdev2);
static void rtt_handler(uint32_t event, gnrc_netdev2_t* gnrc_netdev2);

inline void lwmac_schedule_update(gnrc_netdev2_t* gnrc_netdev2)
{
    gnrc_netdev2->lwmac.needs_rescheduling = true;
}

inline bool lwmac_needs_update(gnrc_netdev2_t* gnrc_netdev2)
{
    return gnrc_netdev2->lwmac.needs_rescheduling;
}

void lwmac_set_state(gnrc_netdev2_t* gnrc_netdev2, lwmac_state_t newstate)
{
    lwmac_state_t oldstate = gnrc_netdev2->lwmac.state;

    if (newstate == oldstate) {
        return;
    }

    if (newstate >= STATE_COUNT) {
        LOG_ERROR("Trying to set invalid state %u\n", newstate);
        return;
    }

    /* Already change state, but might be reverted to oldstate when needed */
    gnrc_netdev2->lwmac.state = newstate;

    /* Actions when leaving old state */
    switch (oldstate)
    {
    case RECEIVING:
    case TRANSMITTING:
        /* Enable duty cycling again */
        rtt_handler(LWMAC_EVENT_RTT_RESUME, gnrc_netdev2);

#if 0
        /* Output duty-cycle ratio */
        uint64_t duty;
        duty = (uint64_t) rtt_get_counter();
        duty = ((uint64_t) gnrc_netdev2->lwmac.awake_duration_sum_ticks)*100 / (duty - (uint64_t)gnrc_netdev2->lwmac.system_start_time_ticks);
        printf("[lwmac-tx]: achieved duty-cycle: %lu %% \n", (uint32_t)duty);
#endif
        break;

    case SLEEPING:
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD);
        break;

    default:
        break;
    }

    /* Actions when entering new state */
    switch (newstate)
    {
    /*********************** Operation states *********************************/
    case LISTENING:
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_IDLE);
        break;

    case SLEEPING:
        /* Put transceiver to sleep */
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_SLEEP);
        /* We may have come here through RTT handler, so timeout may still be active */
        lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD);
        /* Return immediately, so no rescheduling */
        return;

    /* Trying to send data */
    case TRANSMITTING:
        rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev2); /* No duty cycling while RXing */
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_IDLE);  /* Power up netdev */
        break;

    /* Receiving incoming data */
    case RECEIVING:
        rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev2); /* No duty cycling while TXing */
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_IDLE);  /* Power up netdev */
        break;

    case STOPPED:
        _set_netdev_state(gnrc_netdev2, NETOPT_STATE_OFF);
        break;

    /*********************** Control states ***********************************/
    case START:
        rtt_handler(LWMAC_EVENT_RTT_START, gnrc_netdev2);
        lwmac_set_state(gnrc_netdev2, LISTENING);
        break;

    case STOP:
        rtt_handler(LWMAC_EVENT_RTT_STOP, gnrc_netdev2);
        lwmac_set_state(gnrc_netdev2, STOPPED);
        break;

    case RESET:
        LOG_WARNING("Reset not yet implemented\n");
        lwmac_set_state(gnrc_netdev2, STOP);
        lwmac_set_state(gnrc_netdev2, START);
        break;

    /**************************************************************************/
    default:
        LOG_DEBUG("No actions for entering state %u\n", newstate);
        return;
    }

    lwmac_schedule_update(gnrc_netdev2);
}

/* Main state machine. Call whenever something happens */
bool lwmac_update(gnrc_netdev2_t* gnrc_netdev2)
{
    gnrc_netdev2->lwmac.needs_rescheduling = false;

    switch (gnrc_netdev2->lwmac.state)
    {
    case SLEEPING:

    	if(gnrc_netdev2->lwmac.quit_tx == true) {
    		return false;
    	}
        /* If a packet is scheduled, no other (possible earlier) packet can be
         * sent before the first one is handled, even no broadcast
         */
        if (!lwmac_timeout_is_running(&gnrc_netdev2->lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP)) {

            /* Check if there are broadcasts to send and transmit immediately */
            if (gnrc_priority_pktqueue_length(&(gnrc_netdev2->tx.neighbors[0].queue)) > 0) {
                gnrc_netdev2->tx.current_neighbor = &(gnrc_netdev2->tx.neighbors[0]);
                lwmac_set_state(gnrc_netdev2, TRANSMITTING);
                break;
            }

            gnrc_mac_tx_neighbor_t* neighbour = _next_tx_neighbour(gnrc_netdev2);

            if (neighbour != NULL) {

            	/* if phase unkown, send immediately after CP. */
            	if(neighbour->phase >= RTT_TICKS_TO_US(LWMAC_WAKEUP_INTERVAL_US)) {
            	    gnrc_netdev2->tx.current_neighbor = neighbour;

            	    gnrc_netdev2->lwmac.extend_tx = false;
            	    gnrc_netdev2->lwmac.max_tx_num = 0;
            	    lwmac_set_state(gnrc_netdev2, TRANSMITTING);
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
                lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP, time_until_tx);

                /* Register neighbour to be the next */
                gnrc_netdev2->tx.current_neighbor = neighbour;

                /* Stop dutycycling, we're preparing to send. This prevents the
                 * timeout arriving late, so that the destination phase would
                 * be missed. */
                // TODO: bad for power savings
                rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev2);
            } else {
                /* LOG_WARNING("Nothing to send, why did we get called?\n"); */
            }
        } else {
            if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_WAIT_FOR_DEST_WAKEUP)) {
                LOG_DEBUG("Got timeout for dest wakeup, ticks: %"PRIu32"\n", rtt_get_counter());
                gnrc_netdev2->lwmac.extend_tx = false;
                gnrc_netdev2->lwmac.max_tx_num = 0;
                lwmac_set_state(gnrc_netdev2, TRANSMITTING);
            } else {
                /* LOG_DEBUG("Nothing to do, why did we get called?\n"); */
            }
        }
        break;

    case LISTENING:

    	if (_next_tx_neighbour(gnrc_netdev2) != NULL) {
    	    rtt_handler(LWMAC_EVENT_RTT_PAUSE, gnrc_netdev2);
    	}

        /* Restart Listen if found extended transmissions */
        if(gnrc_netdev2->lwmac.extend_wakeup == true) {
            gnrc_netdev2->lwmac.extend_wakeup = false;
            lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD);
        }

        /* Set timeout for if there's no successful rx transaction that will
         * change state to SLEEPING. */
        if (!lwmac_timeout_is_running(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD)) {
            lwmac_set_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD, LWMAC_WAKEUP_DURATION_US);
        } else if (lwmac_timeout_is_expired(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD)) {
            /* Dispatch first as there still may be broadcast packets. */
            _dispatch(gnrc_netdev2->rx.dispatch_buffer);

            gnrc_netdev2->lwmac.state = SLEEPING;
            /* Enable duty cycling again */
            rtt_handler(LWMAC_EVENT_RTT_RESUME, gnrc_netdev2);

            _set_netdev_state(gnrc_netdev2, NETOPT_STATE_SLEEP);
            lwmac_clear_timeout(&gnrc_netdev2->lwmac, TIMEOUT_WAKEUP_PERIOD);

            gnrc_mac_tx_neighbor_t* neighbour = _next_tx_neighbour(gnrc_netdev2);
            if (neighbour != NULL) {
             	lwmac_schedule_update(gnrc_netdev2);
                break;
            } else {
            	/* only try to send pkt after CP-check is clear */
                gnrc_netdev2->lwmac.quit_tx = true;
            }
        }

        if (gnrc_priority_pktqueue_length(&gnrc_netdev2->rx.queue) > 0) {
            lwmac_set_state(gnrc_netdev2, RECEIVING);
        }
        break;

    case RECEIVING:
    {
        lwmac_rx_state_t state_rx = gnrc_netdev2->rx.state;

        switch (state_rx)
        {
        case RX_STATE_STOPPED:
        {
            lwmac_rx_start(gnrc_netdev2);
            lwmac_rx_update(gnrc_netdev2);
            break;
        }
        case RX_STATE_FAILED:
            /* This may happen frequently because we'll receive WA from
             * every node in range. */
            LOG_DEBUG("Reception was NOT successful\n");
            lwmac_rx_stop(gnrc_netdev2);
            /* Restart */
            lwmac_set_state(gnrc_netdev2, LISTENING);
            break;
        case RX_STATE_SUCCESSFUL:
            LOG_INFO("Reception was successful\n");
            lwmac_rx_stop(gnrc_netdev2);
            /* Dispatch received packets, timing is not critical anymore */
            _dispatch(gnrc_netdev2->rx.dispatch_buffer);
            /* Go back to Listen after successful transaction */
            lwmac_set_state(gnrc_netdev2, LISTENING);
            break;
        default:
            lwmac_rx_update(gnrc_netdev2);
        }

        /* If state has changed, reschedule main state machine */
        if (state_rx != gnrc_netdev2->rx.state)
        {
            lwmac_schedule_update(gnrc_netdev2);
        }
        break;
    }
    case TRANSMITTING:
    {
        char* tx_success = "";
        lwmac_tx_state_t state_tx = gnrc_netdev2->tx.state;

        switch (state_tx)
        {
        case TX_STATE_STOPPED:
        {
            gnrc_pktsnip_t* pkt;

            if ((pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev2->tx.current_neighbor->queue)))
            {
                lwmac_tx_start(gnrc_netdev2, pkt, gnrc_netdev2->tx.current_neighbor);
                lwmac_tx_update(gnrc_netdev2);
            } else {
                /* Shouldn't happen, but never observed this case */
                int id = (gnrc_netdev2->tx.current_neighbor - gnrc_netdev2->tx.neighbors);
                id /= sizeof(gnrc_netdev2->tx.current_neighbor);
                LOG_ERROR("Packet from neighbour's queue (#%d) invalid\n", id);
                lwmac_schedule_update(gnrc_netdev2);
            }
            break;
        }

        case TX_STATE_FAILED:
        	gnrc_netdev2->lwmac.quit_tx = true;
        	gnrc_netdev2->lwmac.extend_tx = false;
            tx_success = "NOT ";
            /* Intended fall-through, TX packet will therefore be dropped. No
             * automatic resending here, we did our best.
             */
        case TX_STATE_SUCCESSFUL:
            if (gnrc_netdev2->tx.current_neighbor == &(gnrc_netdev2->tx.neighbors[0])) {
                LOG_INFO("Broadcast transmission done\n");
            } else {
                LOG_INFO("Transmission was %ssuccessful (%"PRIu32" WRs sent)\n",
                         tx_success, gnrc_netdev2->tx.wr_sent);
            }
            lwmac_tx_stop(gnrc_netdev2);

            if (gnrc_netdev2->lwmac.max_tx_num >= LWMAC_MAX_TX_BURST_PKT_NUM) {
                gnrc_netdev2->lwmac.quit_tx = true;
                lwmac_set_state(gnrc_netdev2, SLEEPING);
                break;
            }

            if (gnrc_netdev2->lwmac.extend_tx == true) {
            	lwmac_schedule_update(gnrc_netdev2);
            } else {
            	gnrc_netdev2->lwmac.quit_tx = true;
                lwmac_set_state(gnrc_netdev2, SLEEPING);
            }

            break;
        default:
            lwmac_tx_update(gnrc_netdev2);
        }

        /* If state has changed, reschedule main state machine */
        if (state_tx != gnrc_netdev2->tx.state)
        {
            lwmac_schedule_update(gnrc_netdev2);
        }
        break;
    }
    default:
        LOG_DEBUG("No actions in state %u\n", gnrc_netdev2->lwmac.state);
    }

    return gnrc_netdev2->lwmac.needs_rescheduling;
}

static void rtt_cb(void* arg)
{
    msg_t msg;
    msg.content.value = ((uint32_t) arg ) & 0xffff;
    msg.type = LWMAC_EVENT_RTT_TYPE;
    msg_send(&msg, lwmac_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

void rtt_handler(uint32_t event, gnrc_netdev2_t* gnrc_netdev2)
{
    uint32_t alarm;
    switch (event & 0xffff)
    {
    case LWMAC_EVENT_RTT_WAKEUP_PENDING:
        gnrc_netdev2->lwmac.last_wakeup = rtt_get_alarm();
        alarm = _next_inphase_event(gnrc_netdev2->lwmac.last_wakeup, RTT_US_TO_TICKS(LWMAC_WAKEUP_DURATION_US));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_SLEEP_PENDING);
        gnrc_netdev2->lwmac.extend_wakeup = false;
        gnrc_netdev2->lwmac.quit_tx = false;
        lwmac_set_state(gnrc_netdev2, LISTENING);
        break;

    case LWMAC_EVENT_RTT_SLEEP_PENDING:
        alarm = _next_inphase_event(gnrc_netdev2->lwmac.last_wakeup, RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_WAKEUP_PENDING);

        if (_next_tx_neighbour(gnrc_netdev2) != NULL) {
            lwmac_schedule_update(gnrc_netdev2);
            break;
        } else {
       	    /* only try to send pkt after CP-check is free*/
            gnrc_netdev2->lwmac.quit_tx = true;
        }

        lwmac_set_state(gnrc_netdev2, SLEEPING);

        if(((rtt_get_counter()-gnrc_netdev2->lwmac.system_start_time_ticks) > RTT_US_TO_TICKS(LWMACMAC_DUTYCYCLE_RECORD_US))
        		&&(!gnrc_netdev2->lwmac.exp_end)){
        	/* Output duty-cycle ratio */
        	uint64_t duty;
        	duty = (uint64_t)rtt_get_counter();

        	gnrc_netdev2->lwmac.exp_end = true;

        	printf("Device awake_duration_sum: %lu us \n", RTT_TICKS_TO_US(gnrc_netdev2->lwmac.awake_duration_sum_ticks));
        	printf("Device life time : %lu us \n", RTT_TICKS_TO_US((uint32_t)(duty - (uint64_t)gnrc_netdev2->lwmac.system_start_time_ticks)));

        	duty = ((uint64_t) gnrc_netdev2->lwmac.awake_duration_sum_ticks)*100 / (duty - (uint64_t)gnrc_netdev2->lwmac.system_start_time_ticks);
        	printf("Device achieved duty-cycle: %lu %% \n", (uint32_t)duty);
        }
        break;

    /* Set initial wakeup alarm that starts the cycle */
    case LWMAC_EVENT_RTT_START:
        LOG_DEBUG("RTT: Initialize duty cycling\n");
        alarm = rtt_get_counter() + RTT_US_TO_TICKS(LWMAC_WAKEUP_DURATION_US);
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_SLEEP_PENDING);
        gnrc_netdev2->lwmac.dutycycling_active = true;
        break;

    case LWMAC_EVENT_RTT_STOP:
    case LWMAC_EVENT_RTT_PAUSE:
        rtt_clear_alarm();
        LOG_DEBUG("RTT: Stop duty cycling, now in state %u\n", gnrc_netdev2->lwmac.state);
        gnrc_netdev2->lwmac.dutycycling_active = false;
        break;

    case LWMAC_EVENT_RTT_RESUME:
        LOG_DEBUG("RTT: Resume duty cycling\n");
        alarm = _next_inphase_event(gnrc_netdev2->lwmac.last_wakeup, RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
        rtt_set_alarm(alarm, rtt_cb, (void*) LWMAC_EVENT_RTT_WAKEUP_PENDING);
        gnrc_netdev2->lwmac.dutycycling_active = true;
        break;
    }
}

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event         type of event
 * @param[in] data          optional parameter
 */
static void _event_cb(netdev2_t* dev, netdev2_event_t event)
{
    gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) dev->context;

    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV2_MSG_TYPE_EVENT;
        msg.content.ptr = (void*) gnrc_netdev2;

        if (msg_send(&msg, gnrc_netdev2->pid) <= 0) {
            puts("gnrc_netdev2: possibly lost interrupt.");
        }
    } else {

        DEBUG("gnrc_netdev2: event triggered -> %i\n", event);
        switch (event)
        {
        case NETDEV2_EVENT_RX_STARTED:
            LOG_DEBUG("NETDEV_EVENT_RX_STARTED\n");
            //lwmac.rx_started = true;
            gnrc_netdev2_set_rx_started(gnrc_netdev2,true);
            break;
            case NETDEV2_EVENT_RX_COMPLETE:
        {
            LOG_DEBUG("NETDEV_EVENT_RX_COMPLETE\n");

            gnrc_pktsnip_t *pkt = gnrc_netdev2->recv(gnrc_netdev2);

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
            if (pkt == NULL){
                gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
                break;
            }

            if (!gnrc_netdev2_get_rx_started(gnrc_netdev2)) {
                LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
                gnrc_pktbuf_release(pkt);
                gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
                break;
            }

            gnrc_netdev2_set_rx_started(gnrc_netdev2,false);

            if(!gnrc_mac_queue_rx_packet(&gnrc_netdev2->rx, 0, pkt))
            {
                LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
                gnrc_pktbuf_release(pkt);
                break;
            }
            lwmac_schedule_update(gnrc_netdev2);
            break;
        }
        case NETDEV2_EVENT_TX_STARTED:
            //lwmac.tx_feedback = TX_FEEDBACK_UNDEF;
            gnrc_netdev2_set_tx_feedback(gnrc_netdev2,TX_FEEDBACK_UNDEF);
            gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
            //lwmac_schedule_update();
            break;
        case NETDEV2_EVENT_TX_COMPLETE:
            //lwmac.tx_feedback = TX_FEEDBACK_SUCCESS;
            gnrc_netdev2_set_tx_feedback(gnrc_netdev2,TX_FEEDBACK_SUCCESS);
            gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
            lwmac_schedule_update(gnrc_netdev2);
            break;
        case NETDEV2_EVENT_TX_NOACK:
            //lwmac.tx_feedback = TX_FEEDBACK_NOACK;
            gnrc_netdev2_set_tx_feedback(gnrc_netdev2,TX_FEEDBACK_NOACK);
            gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
            lwmac_schedule_update(gnrc_netdev2);
            break;
        case NETDEV2_EVENT_TX_MEDIUM_BUSY:
            //lwmac.tx_feedback = TX_FEEDBACK_BUSY;
            gnrc_netdev2_set_tx_feedback(gnrc_netdev2,TX_FEEDBACK_BUSY);
            gnrc_netdev2_set_rx_started(gnrc_netdev2,false);
            lwmac_schedule_update(gnrc_netdev2);
            break;

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
// TODO: Don't use global variables
static void *_lwmac_thread(void *args)
{
    gnrc_netdev2_t* gnrc_netdev2 = (gnrc_netdev2_t *)args;
    netdev2_t* dev = gnrc_netdev2->dev;

    gnrc_netdev2->pid = thread_getpid();

    gnrc_netapi_opt_t* opt;
    int res;
    msg_t msg, reply, msg_queue[LWMAC_IPC_MSG_QUEUE_SIZE];

    LOG_INFO("Starting lwMAC\n");

    /* RTT is used for scheduling wakeup */
    rtt_init();

    /* Store pid globally, so that IRQ can use it to send msg */
    lwmac_pid = thread_getpid();
    gnrc_netdev2->lwmac.pid = lwmac_pid;

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, LWMAC_IPC_MSG_QUEUE_SIZE);

    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void*) gnrc_netdev2;

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
    //lwmac.l2_addr.len = dev->driver->get(dev, NETOPT_ADDRESS_LONG, &lwmac.l2_addr.addr, sizeof(lwmac.l2_addr.addr));
    gnrc_netdev2->l2_addr_len = dev->driver->get(dev, NETOPT_ADDRESS_LONG, &gnrc_netdev2->l2_addr, IEEE802154_LONG_ADDRESS_LEN);
    assert(gnrc_netdev2->l2_addr_len > 0);

    /* Initialize broadcast sequence number. This at least differs from board
     * to board */
    gnrc_netdev2->tx.bcast_seqnr = gnrc_netdev2->l2_addr[7];

    /* Reset all timeouts just to be sure */
    lwmac_reset_timeouts(&gnrc_netdev2->lwmac);

    /* Start duty cycling */
    lwmac_set_state(gnrc_netdev2, START);


    xtimer_sleep(5);

    uint32_t seed;

    seed = 0;
    seed = gnrc_netdev2->l2_addr[6];
    seed = seed << 8;
    seed |= gnrc_netdev2->l2_addr[7];

    random_init(seed);

    printf("seed: %lx\n",seed);

    /* Start exp setting */
    gnrc_netdev2->lwmac.exp_duration = 300; //seconds
    gnrc_netdev2->lwmac.cycle_duration = 100;  //ms
    gnrc_netdev2->lwmac.cp_duration = 8; //ms

    if(0) {

    	lwmac_send_exp_setting(gnrc_netdev2);

    } else {

    	puts("wait for expset message");
    	/* Wait for exp setting message */
    	 while (1) {

    	        msg_receive(&msg);

    	        /* Handle NETDEV, NETAPI, RTT and TIMEOUT messages */
    	        switch (msg.type) {
    	        /* Transceiver raised an interrupt */
    	        case NETDEV2_MSG_TYPE_EVENT:
    	            LOG_DEBUG("GNRC_NETDEV_MSG_TYPE_EVENT received\n");
    	            /* Forward event back to driver */
    	            dev->driver->isr(dev);
    	            break;
    	            /* NETAPI set/get. Can't this be refactored away from here? */
    	            /* RTT raised an interrupt */
    	        case LWMAC_EVENT_RTT_TYPE: {
    	                LOG_DEBUG("Ignoring late RTT event while dutycycling is off\n");
    	        }break;

    	        case GNRC_NETAPI_MSG_TYPE_SET:
    	        {
    	            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SET received\n");
    	            opt = (gnrc_netapi_opt_t *)msg.content.ptr;

	                /* set option for device driver */
	                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
	                LOG_DEBUG("Response of netdev->set: %i\n", res);
    	            /* send reply to calling thread */
    	            reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
    	            reply.content.value = (uint32_t)res;
    	            msg_reply(&msg, &reply);
    	            break;
    	        }

    	        case GNRC_NETAPI_MSG_TYPE_GET:
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

    	        default:
    	            LOG_ERROR("Unknown command %" PRIu16 "\n", msg.type);
    	            break;
    	        }

    	        gnrc_pktsnip_t* pkt_exp;
    	        if ((pkt_exp = gnrc_priority_pktqueue_pop(&gnrc_netdev2->rx.queue)) != NULL)
    	        {
    	            LOG_DEBUG("Inspecting pkt @ %p\n", pkt_exp);

    	            /* Parse packet */
    	            lwmac_packet_info_t info;
    	            int ret = _parse_packet(pkt_exp, &info);

    	            if (ret != 0) {
    	                LOG_DEBUG("Packet could not be parsed: %i\n", ret);
    	                gnrc_pktbuf_release(pkt_exp);
    	            }

    	            if (info.header->type == FRAMETYPE_EXP_SETTING) {

    	            	puts("get expset command");
    	    	        uint16_t *payload;

    	    	        payload = pkt_exp->data;

    	    	        gnrc_netdev2->lwmac.exp_duration = payload[1];
    	    	        gnrc_netdev2->lwmac.cycle_duration = payload[3];
    	    	        gnrc_netdev2->lwmac.cp_duration = payload[4];

    	                if (!gnrc_netapi_dispatch_receive(GNRC_NETTYPE_APP, GNRC_NETREG_DEMUX_CTX_ALL, pkt_exp)) {
    	                    gnrc_pktbuf_release(pkt_exp);
    	                    puts("dispatch pkt fail, drop it");
    	                }

    	                break;

    	            }
    	        }// end of pkt process

    	 } //end of while(1)

    }

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
    /* Start duty cycle recording */
    rtt_set_counter(0);
    gnrc_netdev2->lwmac.system_start_time_ticks = rtt_get_counter();
    gnrc_netdev2->lwmac.last_radio_on_time_ticks = gnrc_netdev2->lwmac.system_start_time_ticks;
    gnrc_netdev2->lwmac.awake_duration_sum_ticks = 0;
    gnrc_netdev2->lwmac.radio_is_on = true;
    gnrc_netdev2->lwmac.exp_end = false;
#endif

    //puts("start exp");
    /* start the event loop */
    while (1) {

        msg_receive(&msg);

        /* Handle NETDEV, NETAPI, RTT and TIMEOUT messages */
        switch (msg.type) {

        /* RTT raised an interrupt */
        case LWMAC_EVENT_RTT_TYPE:
            if (gnrc_netdev2->lwmac.dutycycling_active) {
                rtt_handler(msg.content.value, gnrc_netdev2);
                lwmac_schedule_update(gnrc_netdev2);
            } else {
                LOG_DEBUG("Ignoring late RTT event while dutycycling is off\n");
            }
            break;

        /* An lwmac timeout occured */
        case LWMAC_EVENT_TIMEOUT_TYPE:
        {
            lwmac_timeout_make_expire((lwmac_timeout_t*) msg.content.ptr);
            lwmac_schedule_update(gnrc_netdev2);
            break;
        }

        /* Transceiver raised an interrupt */
        case NETDEV2_MSG_TYPE_EVENT:
            LOG_DEBUG("GNRC_NETDEV_MSG_TYPE_EVENT received\n");
            /* Forward event back to driver */
            dev->driver->isr(dev);
            break;

        /* TX: Queue for sending */
        case GNRC_NETAPI_MSG_TYPE_SND:
        {
            // TODO: how to announce failure to upper layers?

            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SND received\n");
            gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) msg.content.ptr;

            gnrc_mac_queue_tx_packet(&gnrc_netdev2->tx, 0, pkt);

            lwmac_schedule_update(gnrc_netdev2);
            break;
        }
        /* NETAPI set/get. Can't this be refactored away from here? */
        case GNRC_NETAPI_MSG_TYPE_SET:
        {
            LOG_DEBUG("GNRC_NETAPI_MSG_TYPE_SET received\n");
            opt = (gnrc_netapi_opt_t *)msg.content.ptr;

            /* Depending on option forward to NETDEV or handle here */
            switch (opt->opt)
            {
            /* Handle state change requests */
            case NETOPT_STATE:
            {
                netopt_state_t* state = (netopt_state_t*) opt->data;
                res = opt->data_len;
                switch (*state)
                {
                case NETOPT_STATE_OFF:
                    lwmac_set_state(gnrc_netdev2, STOP);
                    break;
                case NETOPT_STATE_IDLE:
                    lwmac_set_state(gnrc_netdev2, START);
                    break;
                case NETOPT_STATE_RESET:
                    lwmac_set_state(gnrc_netdev2, RESET);
                    break;
                default:
                    res = -EINVAL;
                    LOG_ERROR("NETAPI tries to set unsupported state %u\n",
                              *state);
                }
                lwmac_schedule_update(gnrc_netdev2);
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
        case GNRC_NETAPI_MSG_TYPE_GET:
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

        default:
            LOG_ERROR("Unknown command %" PRIu16 "\n", msg.type);
            break;
        }

        /* Execute main state machine because something just happend*/
        while (lwmac_needs_update(gnrc_netdev2)) {
            lwmac_update(gnrc_netdev2);
        }
    }

    LOG_ERROR("terminated\n");

    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_lwmac_init(char *stack, int stacksize, char priority,
                             const char *name, gnrc_netdev2_t *dev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (dev == NULL || dev->dev == NULL) {
        LOG_ERROR("No netdev supplied or driver not set\n");
        return -ENODEV;
    }

    /* Prevent sleeping until first RTT alarm is set */
    //lpm_prevent_sleep |= LWMAC_LPM_MASK;

    /* create new LWMAC thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                        _lwmac_thread, (void *)dev, name);
    if (res <= 0) {
        LOG_ERROR("Couldn't create thread\n");
        //lpm_prevent_sleep &= ~(LWMAC_LPM_MASK);
        return -EINVAL;
    }

    return res;
}

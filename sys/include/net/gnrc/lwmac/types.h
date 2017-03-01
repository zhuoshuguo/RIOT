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
 * @brief       Internal types of LWMAC
 * @internal
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_LWMAC_TYPES_H_
#define GNRC_LWMAC_TYPES_H_

#include <net/gnrc/lwmac/hdr.h>
#include <net/gnrc/lwmac/timeout.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LWMAC_EVENT_RTT_TYPE            (0x4300)
#define LWMAC_EVENT_RTT_START           (0x4301)
#define LWMAC_EVENT_RTT_STOP            (0x4302)
#define LWMAC_EVENT_RTT_PAUSE           (0x4303)
#define LWMAC_EVENT_RTT_RESUME          (0x4304)
#define LWMAC_EVENT_RTT_WAKEUP_PENDING  (0x4305)
#define LWMAC_EVENT_RTT_SLEEP_PENDING   (0x4306)
#define LWMAC_EVENT_TIMEOUT_TYPE        (0x4400)


/**
 * @brief   Enable/disable duty-cycle record and print out.
 *          Set "1" to enable, set "0" to disable.
 */
#ifndef LWMAC_ENABLE_DUTYCYLE_RECORD
#define LWMAC_ENABLE_DUTYCYLE_RECORD             (1U)
#endif

/**
 * @brief   Internal states of Lwmac
 */
typedef enum {
    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,
    LISTENING,
    RECEIVING,      /**< RX is handled in own state machine */
    TRANSMITTING,   /**< TX is handled in own state machine */
    SLEEPING,
    STATE_COUNT
} lwmac_state_t;

/**
 * @brief   TX states of Lwmac
 */
typedef enum {
    TX_STATE_STOPPED = 0,
    TX_STATE_INIT,          /**< Initiate transmission */
    TX_STATE_SEND_BROADCAST,/**< directly goes to SUCCESSFUL or FAILED when finished */
    TX_STATE_SEND_WR,       /**< Send a wakeup request */
    TX_STATE_WAIT_WR_SENT,  /**< Wait until WR sent to set timeout */
    TX_STATE_WAIT_FOR_WA,   /**< Wait for dest node's wakeup ackknowledge */
    TX_STATE_SEND_DATA,     /**< Send the actual payload data */
    TX_STATE_WAIT_FEEDBACK, /**< Wait if packet was ACKed */
    TX_STATE_SUCCESSFUL,    /**< Transmission has finished successfully */
    TX_STATE_FAILED         /**< Payload data couldn't be delivered to dest */
} lwmac_tx_state_t;
#define LWMAC_TX_STATE_INIT TX_STATE_STOPPED

/**
 * @brief   RX states of Lwmac
 */
typedef enum {
    RX_STATE_STOPPED = 0,
    RX_STATE_INIT,          /**< Initiate reception */
    RX_STATE_WAIT_FOR_WR,   /**< Wait for a wakeup request */
    RX_STATE_SEND_WA,       /**< Send wakeup ackknowledge to requesting node */
    RX_STATE_WAIT_WA_SENT,  /**< Wait until WA sent to set timeout */
    RX_STATE_WAIT_FOR_DATA, /**< Wait for actual payload data */
    RX_STATE_SUCCESSFUL,    /**< Recption has finished successfully */
    RX_STATE_FAILED         /**< Reception over, but nothing received */
} lwmac_rx_state_t;
#define LWMAC_RX_STATE_INIT RX_STATE_STOPPED

#define LWMAC_RX_INIT { \
/* rx::state */             LWMAC_RX_STATE_INIT, \
/* rx::queue */             {}, \
/* rx::_queue_nodes */      {}, \
/* rx::l2_addr */           LWMAC_L2_ADDR_INIT, \
/* rx::dispatch_buffer */   {}, \
}

#define LWMAX_NEIGHBOUR_INIT        { LWMAC_L2_ADDR_INIT, {}, 0 }

#define LWMAC_PHASE_UNINITIALIZED   (0)
#define LWMAC_PHASE_MAX             (-1)

#define LWMAC_TX_INIT { \
/* tx::state */             LWMAC_TX_STATE_INIT, \
/* tx::neighbours */        { LWMAX_NEIGHBOUR_INIT }, \
/* tx::_queue_nodes */      {}, \
/* tx::wr_sent */           0, \
/* tx::packet */            NULL, \
/* tx::current_neighbour */ NULL, \
/* tx::timestamp */         0, \
/* tx:bcast_seqnr */        0, \
}

/**
 * @brief   Internal unit for store states of Lwmac
 */
typedef struct lwmac {
    /* PID of lwMAC thread */
    kernel_pid_t pid;

    /* Internal state of MAC layer */
    lwmac_state_t state;

    /* Used to calculate wakeup times */
    uint32_t last_wakeup;
    /* Keep track of duty cycling to avoid late RTT events after stopping */
    bool dutycycling_active;
    /* Used internally for rescheduling state machine update, e.g. after state
     * transition caused in update */
    bool needs_rescheduling;

    /* Store timeouts used for protocol */
    lwmac_timeout_t timeouts[LWMAC_TIMEOUT_COUNT];

    bool extend_wakeup;
    bool extend_tx;
    bool quit_tx;
    uint8_t max_tx_num;

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
    /* parameters for recording duty-cycle */
    bool radio_is_on;
    uint32_t last_radio_on_time_ticks;
    uint32_t radio_off_time_ticks;
    uint32_t system_start_time_ticks;
    uint32_t awake_duration_sum_ticks;
    uint32_t pkt_start_sending_time_ticks;

    bool exp_end;
#endif

    uint32_t exp_duration;
    uint32_t cycle_duration;
    uint32_t cp_duration;

} lwmac_t;

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_TYPES_H_ */
/** @} */

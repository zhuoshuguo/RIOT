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

/**
 * @brief Lwmac RTT event type.
 */
#define LWMAC_EVENT_RTT_TYPE            (0x4300)

/**
 * @brief Lwmac RTT start event type.
 */
#define LWMAC_EVENT_RTT_START           (0x4301)

/**
 * @brief Lwmac RTT stop event type.
 */
#define LWMAC_EVENT_RTT_STOP            (0x4302)

/**
 * @brief Lwmac RTT pause event type.
 */    
#define LWMAC_EVENT_RTT_PAUSE           (0x4303)

/**
 * @brief Lwmac RTT resume event type.
 */
#define LWMAC_EVENT_RTT_RESUME          (0x4304)

/**
 * @brief Lwmac RTT wakeup pending event type.
 */
#define LWMAC_EVENT_RTT_WAKEUP_PENDING  (0x4305)

/**
 * @brief Lwmac RTT sleep pending event type.
 */
#define LWMAC_EVENT_RTT_SLEEP_PENDING   (0x4306)

/**
 * @brief Lwmac timeout event type.
 */
#define LWMAC_EVENT_TIMEOUT_TYPE        (0x4400)

/**
 * @brief Enable/disable duty-cycle record and print out.
 *        Set "1" to enable, set "0" to disable.
 */
#ifndef LWMAC_ENABLE_DUTYCYLE_RECORD
#define LWMAC_ENABLE_DUTYCYLE_RECORD             (0U)
#endif

/**
 * @brief Internal states of Lwmac
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
 * @brief TX states of Lwmac
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

/**
 * @brief Static initializer for lwmac_tx_state_t.
 */
#define LWMAC_TX_STATE_INIT TX_STATE_STOPPED

/**
 * @brief RX states of Lwmac
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

/**
 * @brief Static initializer for lwmac_rx_state_t.
 */
#define LWMAC_RX_STATE_INIT RX_STATE_STOPPED

/**
 * @brief Lwmac uninitialized phase value
 */
#define LWMAC_PHASE_UNINITIALIZED   (0)

/**
 * @brief Lwmac max phase value
 */
#define LWMAC_PHASE_MAX             (-1)

/**
 * @brief Lwmac specific structure for storing internal states.
 */
typedef struct lwmac {
    lwmac_state_t state;                           /**< Internal state of MAC layer */
    uint32_t last_wakeup;                          /**< Used to calculate wakeup times */
    bool dutycycling_active;                       /**< Keep track of duty cycling to avoid 
                                                        late RTT events after stopping */
    bool needs_rescheduling;                       /**< Used internally for rescheduling state
                                                        machine update, e.g. after state 
                                                        transition caused in update */
    lwmac_timeout_t timeouts[LWMAC_TIMEOUT_COUNT]; /**< Store timeouts used for protocol */

#if (LWMAC_ENABLE_DUTYCYLE_RECORD == 1)
    /* parameters for recording duty-cycle */
    bool radio_is_on;                             /**< check if radio is on */
    uint32_t last_radio_on_time_ticks;            /**< the last time in ticks when radio is on */
    uint32_t radio_off_time_ticks;                /**< the time in ticks when radio is off */
    uint32_t system_start_time_ticks;             /**< the time in ticks when chip is started */
    uint32_t awake_duration_sum_ticks;            /**< the sum of time in ticks when radio is on */
    uint32_t pkt_start_sending_time_ticks;        /**< the time in ticks when the packet is started 
                                                       to be sent */
#endif
} lwmac_t;

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_TYPES_H_ */
/** @} */

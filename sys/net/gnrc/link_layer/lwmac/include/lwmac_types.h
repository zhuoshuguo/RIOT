/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Internal types of LWMAC
 * @{
 *
 * @file
 * @brief       Internal types used by the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_LWMAC_TYPES_H_
#define GNRC_LWMAC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel_types.h>
#include <xtimer.h>
#include <net/netdev2.h>
#include <net/gnrc/netdev2.h>
#include <net/gnrc.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <net/gnrc/lwmac/hdr.h>
#include <net/gnrc/lwmac/packet_queue.h>
#include <net/gnrc/lwmac/timeout.h>

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/

typedef struct {
    /* Internal state of reception state machine */
    lwmac_rx_state_t state;
    packet_queue_t queue;
    packet_queue_node_t _queue_nodes[LWMAC_RX_QUEUE_SIZE];
    l2_addr_t l2_addr;
    gnrc_pktsnip_t* dispatch_buffer[LWMAC_DISPATCH_BUFFER_SIZE];
} lwmac_rx_t;

#define LWMAC_RX_INIT { \
/* rx::state */             LWMAC_RX_STATE_INIT, \
/* rx::queue */             {}, \
/* rx::_queue_nodes */      {}, \
/* rx::l2_addr */           LWMAC_L2_ADDR_INIT, \
/* rx::dispatch_buffer */   {}, \
}

/******************************************************************************/

typedef struct {
    /* Address of neighbour node */
    l2_addr_t l2_addr;
    /* TX queue for this particular node */
    packet_queue_t queue;
    /* Phase relative to lwmac::last_wakeup */
    uint32_t phase;
} lwmac_tx_neighbour_t;
#define LWMAX_NEIGHBOUR_INIT        { LWMAC_L2_ADDR_INIT, {}, 0 }

#define LWMAC_PHASE_UNINITIALIZED   (0)
#define LWMAC_PHASE_MAX             (-1)

/******************************************************************************/

typedef struct {
    /* Internal state of transmission state machine */
    lwmac_tx_state_t state;
    /* TX queues for neighbouring nodes. First queue is broadcast (+1) */
    lwmac_tx_neighbour_t neighbours[LWMAC_NEIGHBOUR_COUNT + 1];
    /* Shared buffer for TX queue nodes */
    packet_queue_node_t _queue_nodes[LWMAC_TX_QUEUE_SIZE];
    /* Count how many WRs were sent until WA received */
    uint32_t wr_sent;
    /* Packet that is currently scheduled to be sent */
    gnrc_pktsnip_t* packet;
    /* Queue of destination node to which the current packet will be sent */
    lwmac_tx_neighbour_t* current_neighbour;
    uint32_t timestamp;
    /* Sequence number for broadcast data to filter at receiver */
    uint8_t bcast_seqnr;
} lwmac_tx_t;

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

/******************************************************************************/

typedef struct lwmac {
    /* PID of lwMAC thread */
    kernel_pid_t pid;
    /* NETDEV device used by lwMAC */
	gnrc_netdev2_t* netdev;
	const netdev2_driver_t* netdev2_driver;
    /* Internal state of MAC layer */
    lwmac_state_t state;
    /* Track if a transmission might have corrupted a received packet */
    //bool rx_started;
    /* Own address */
    //l2_addr_t l2_addr;
    lwmac_rx_t rx;
    lwmac_tx_t tx;

    /* Store timeouts used for protocol */
    lwmac_timeout_t timeouts[LWMAC_TIMEOUT_COUNT];
    /* Used to calculate wakeup times */
    uint32_t last_wakeup;
    /* Keep track of duty cycling to avoid late RTT events after stopping */
    bool dutycycling_active;
    /* Used internally for rescheduling state machine update, e.g. after state
     * transition caused in update */
    bool needs_rescheduling;
} lwmac_t;

#define LWMAC_INIT { \
/* pid */                   KERNEL_PID_UNDEF,  \
/* netdev */                NULL, \
/* netdev2_driver */		NULL, \
/* state */                 UNDEF, \
/* rx */                    LWMAC_RX_INIT, \
/* tx */                    LWMAC_TX_INIT, \
/* timeouts */              { LWMAC_TIMEOUT_INIT }, \
/* last_wakeup */           0, \
/* dutycycling_active */    false, \
/* needs_rescheduling */    false \
}

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_TYPES_H_ */
/** @} */

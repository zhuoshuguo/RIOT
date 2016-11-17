/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_mac  A common MAC type for providing key MAC parameters and helper functions
 * @ingroup     net
 * @brief       A common MAC type for providing key MAC parameters and helper functions.
 * @{
 *
 * @file
 * @brief       Internal types used by the GNRC_MAC entity
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_MAC_TYPES_H_
#define GNRC_MAC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel_types.h>
#include <xtimer.h>
#include <net/netdev2.h>
#include <net/gnrc.h>
#include <net/gnrc/mac/mac.h>
#include <net/gnrc/priority_pktqueue.h>
#include <net/ieee802154.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief definition for device transmission feedback types
 */
typedef enum {
    TX_FEEDBACK_UNDEF = 0,    /**< Transmission just start, no Tx feedback yet */
    TX_FEEDBACK_SUCCESS,      /**< Transmission succeeded */
    TX_FEEDBACK_NOACK,        /**< No ACK for the transmitted packet */
    TX_FEEDBACK_BUSY          /**< found medium busy when doing transmission */
} gnrc_mac_tx_feedback_t;

/**
 * @brief MAC internal type for storing reception state parameters and
 *        state machines.
 *        This structure can be extended to contain more needed
 *        states and parameters. Please #ifdef+NEWMAC them out for particular
 *        new MAC protocols.
 */
typedef struct {
    /* RX queue for storing received packets */
    gnrc_priority_pktqueue_t queue;

    /* RX queue nodes */
    gnrc_priority_pktqueue_node_t _queue_nodes[GNRC_MAC_RX_QUEUE_SIZE];

    /* buffer for storing dispatching packets */
    gnrc_pktsnip_t* dispatch_buffer[GNRC_MAC_DISPATCH_BUFFER_SIZE];
} gnrc_mac_rx_t;

/**
 * @brief type for storing states of tx neighbor node.
 */
typedef struct {
    uint8_t  l2_addr[IEEE802154_LONG_ADDRESS_LEN]; /* Address of neighbour node */
    uint8_t  l2_addr_len;                          /* Neighbour address length */
    gnrc_priority_pktqueue_t queue;                /* TX queue for this particular Neighbour */
    uint32_t phase;                                /* Neighbour's wake-up Phase */
} gnrc_mac_tx_neighbour_t;

/**
 * @brief Uninitialized phase value.
 */
#define GNRC_MAC_PHASE_UNINITIALIZED   (0)

/**
 * @brief Maximum phase value.
 */
#define GNRC_MAC_PHASE_MAX             (-1)

/**
 * @brief MAC internal type for storing transmission state parameters and
 *        state machines.
 *        This structure can be extended to contain more needed
 *        states and parameters. Please #ifdef+NEWMAC them out for particular
 *        new MAC protocols.
 */
typedef struct {
#if GNRC_MAC_NEIGHBOUR_COUNT != 0
    /* TX queues for neighbouring nodes. First queue is for broadcast (+1) */
    gnrc_mac_tx_neighbour_t neighbours[GNRC_MAC_NEIGHBOUR_COUNT + 1];
#endif
    /* Shared buffer for TX queue nodes */
    gnrc_priority_pktqueue_node_t _queue_nodes[GNRC_MAC_TX_QUEUE_SIZE];

    /* Packet that is currently scheduled to be sent */
    gnrc_pktsnip_t* packet;

    /* Queue of destination node to which the current packet will be sent */
    gnrc_mac_tx_neighbour_t* current_neighbour;

} gnrc_mac_tx_t;



#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_TYPES_H_ */
/** @} */

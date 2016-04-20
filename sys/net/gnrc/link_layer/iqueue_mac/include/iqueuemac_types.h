/*
 * Copyright (C) 2016 Shuguo Zhuo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_iqueue_mac
 * @ingroup     net
 * @brief       Internal types of IQUEUEMAC
 * @{
 *
 * @file
 * @brief       Internal types used by the IQUEUEMAC protocol
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_IQUEUEMAC_TYPES_H_
#define GNRC_IQUEUEMAC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel_types.h>
#include <xtimer.h>
#include <net/gnrc.h>
#include <net/netdev2.h>
#include <net/gnrc/netdev2.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include "net/gnrc/iqueue_mac/hdr.h"
//#include <net/gnrc/lwmac/hdr.h>
#include <net/gnrc/iqueue_mac/packet_queue.h>
#include "timeout.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

#define IQUEUEMAC_EVENT_RTT_TYPE            (0x4300)
#define IQUEUEMAC_EVENT_RTT_ENTER_CP          (0x4301)
#define IQUEUEMAC_EVENT_RTT_ENTER_BEACON           (0x4302)
#define IQUEUEMAC_EVENT_RTT_ENTER_VTDMA            (0x4303)
#define IQUEUEMAC_EVENT_RTT_ENTER_SLEEP           (0x4304)
#define IQUEUEMAC_EVENT_RTT_START           (0x4305)

#define IQUEUEMAC_EVENT_TIMEOUT_TYPE        (0x4400)

#define IQUEUEMAC_PHASE_UNINITIALIZED (0)

/******************************************************************************/

typedef enum {
 /*   UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,   */
    R_CP,
    R_BEACON,
    R_VTDMA,
    R_SLEEPING,
   // STATE_COUNT
} iqueuemac_router_state_t;

typedef enum {
/*    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,  */
    N_CP,
    N_BEACON,
    N_VTDMA,
    N_SLEEPING,
    //STATE_COUNT
} iqueuemac_node_state_t;

/******************************************************************************/

typedef enum {
    TX_FEEDBACK_UNDEF = -1,
    TX_FEEDBACK_SUCCESS,
    TX_FEEDBACK_NOACK,
    TX_FEEDBACK_BUSY
} iqueuemac_tx_feedback_t;
#define LWMAC_TX_FEEDBACK_INIT TX_FEEDBACK_UNDEF

/******************************************************************************/

typedef struct {
    /* Address of neighbour node */
    l2_addr_t l2_addr;
    /* TX queue for this particular node */
    packet_queue_t queue;
    /* MAC type of the neighbor*/
    iqueuemac_type_t mac_type;  /* UNKONW when this neighbor is not phase-locked yet*/
    /* Phase relative to iqueuemac: the start of its CP period */
    uint32_t cp_phase;
    /* Indicating that whether this neighbor is within the same cluster*/
    bool in_same_cluster;
} iqueuemac_tx_neighbour_t;


/******************************************************************************/
typedef struct iqueuemac {
    /* PID of IQUEUEMAC thread */
    kernel_pid_t pid;
    /* NETDEV device used by lwMAC */
	gnrc_netdev2_t* netdev;
	const netdev2_driver_t* netdev2_driver;

    /* Internal state of MAC layer */
	iqueuemac_type_t mac_type;
	iqueuemac_router_state_t router_state;
	iqueuemac_node_state_t   node_state;

	iqueuemac_timeout_t timeouts[IQUEUEMAC_TIMEOUT_COUNT];

	packet_queue_node_t _queue_nodes[IQUEUEMAC_TX_QUEUE_SIZE];

	packet_queue_t iqueue_mac_tx_queue;

	iqueuemac_tx_neighbour_t neighbours[IQUEUEMAC_NEIGHBOUR_COUNT];

    /* Track if a transmission might have corrupted a received packet */
    bool rx_started;
    /* Own address */
    l2_addr_t own_addr;
    l2_addr_t father_router_addr;
    
    /* Feedback of last packet that was sent */
    iqueuemac_tx_feedback_t tx_feedback;
    
    /* Used to calculate wakeup times */
    uint32_t last_wakeup;    
   
} iqueuemac_t;



#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_TYPES_H_ */
/** @} */

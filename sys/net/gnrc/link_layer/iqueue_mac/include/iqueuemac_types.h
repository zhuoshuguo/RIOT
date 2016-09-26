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
 * @brief       Internal types of iqueue_mac
 * @{
 *
 * @file
 * @brief       Internal types used by the iqueue_mac protocol
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

#define IQUEUEMAC_EVENT_RTT_START           (0x4301)

/****************************** node RTT event types **********************************/
#define IQUEUEMAC_EVENT_RTT_N_ENTER_CP           (0x4302)
#define IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP           (0x4303)
#define IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE           (0x4305)
/****************************** router RTT event types **********************************/
#define IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE           (0x4304)


#define IQUEUEMAC_EVENT_TIMEOUT_TYPE        (0x4400)

#define IQUEUEMAC_PHASE_UNINITIALIZED (0)

#define IQUEUEMAC_PHASE_MAX             (-1)

/******************************************************************************/
typedef enum {
	DEVICE_BROADCAST_INIT = 0,
	DEVICE_SEND_BROADCAST,
	DEVICE_WAIT_BROADCAST_FEEDBACK,
	DEVICE_BROADCAST_END
} iqueuemac_device_broadcast_state_t;

typedef enum {
	DEVICE_T2N_WAIT_CP_INIT = 0,
	DEVICE_T2N_WAIT_CP,
	DEVICE_T2N_TRANS_IN_CP,
	DEVICE_T2N_WAIT_CPTRANS_FEEDBACK,
	DEVICE_T2N_RE_PHASE_LOCK_PREPARE,
	DEVICE_T2N_TRANS_END
} iqueuemac_device_t2n_state_t;

typedef enum {
	DEVICE_T2R_WAIT_CP_INIT = 0,
	DEVICE_T2R_WAIT_CP,
	DEVICE_T2R_TRANS_IN_CP,
	DEVICE_T2R_WAIT_CPTRANS_FEEDBACK,
	DEVICE_T2R_RE_PHASE_LOCK_PREPARE,
	DEVICE_T2R_WAIT_BEACON,
	DEVICE_T2R_WAIT_OWN_SLOTS,
	DEVICE_T2R_TRANS_IN_VTDMA,
	DEVICE_T2R_WAIT_VTDMATRANS_FEEDBACK,
	DEVICE_T2R_TRANS_END
} iqueuemac_device_t2r_state_t;

typedef enum {
	DEVICE_T2U_SEND_PREAMBLE_INIT = 0,
	DEVICE_T2U_SEND_PREAMBLE,
	DEVICE_T2U_WAIT_PREAMBLE_TX_END,
	DEVICE_T2U_WAIT_PREAMBLE_ACK,
	DEVICE_T2U_SEND_DATA,
	DEVICE_T2U_WAIT_TX_FEEDBACK,
	DEVICE_T2U_END
} iqueuemac_device_t2u_state_t;

/******************************router state machinies**********************************/
typedef enum {
/*    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,  */
	/*Basic mode of simple mode*/
	R_LISTENNING,
	R_TRANSMITTING,
	R_INIT
} mac_router_basic_state_t;

typedef enum {
	/*Listening states of simple mode*/
	R_INIT_PREPARE,
	R_INIT_COLLECT_BEACONS,
	R_INIT_WAIT_BUSY_END,
	R_INIT_ANNOUNCE_SUBCHANNEL,
	R_INIT_WAIT_ANNOUNCE_FEEDBACK,
	R_INIT_END
} mac_router_init_state_t;

typedef enum {
	/*Listening states of simple mode*/
	R_LISTEN_CP_INIT,
	R_LISTEN_CP_LISTEN,
	R_LISTEN_CP_END,
	//R_LISTEN_CREATE_BEACON,
	R_LISTEN_SEND_BEACON,
	R_LISTEN_WAIT_BEACON_FEEDBACK,
	R_LISTEN_VTDMA_INIT,
	R_LISTEN_VTDMA,
	R_LISTEN_VTDMA_END,
	R_LISTEN_SLEEPING_INIT,
	R_LISTEN_SLEEPING,
	R_LISTEN_SLEEPING_END
} mac_router_listen_state_t;

typedef enum {
	/*Transmitting states of router*/
	R_TRANS_TO_UNKOWN,
	R_TRANS_TO_ROUTER,
	R_TRANS_TO_NODE,
	R_BROADCAST
} mac_router_trans_state_t;

typedef enum {
	R_T2U_SEND_PREAMBLE_INIT = 0,
	R_T2U_SEND_PREAMBLE,
	R_T2U_WAIT_PREAMBLE_ACK,
	R_T2U_SEND_DATA,
	R_T2U_WAIT_TX_FEEDBACK,
	R_T2U_END
} mac_router_t2u_state_t;

typedef enum {
	/*Transmitting states of router*/
	R_T2R_WAIT_CP_INIT = 0,
	R_T2R_WAIT_CP,
	R_T2R_TRANS_IN_CP,
	R_T2R_WAIT_CPTRANS_FEEDBACK,
	R_T2R_WAIT_BEACON,
	R_T2R_WAIT_OWN_SLOTS,
	R_T2R_TRANS_IN_VTDMA,
	R_T2R_WAIT_VTDMATRANS_FEEDBACK,
	R_T2R_TRANS_END
} mac_router_t2r_state_t;

typedef enum {
	/*Transmitting states of router*/
	R_T2N_WAIT_CP_INIT = 0,
	R_T2N_WAIT_CP,
	R_T2N_TRANS_IN_CP,
	R_T2N_WAIT_CPTRANS_FEEDBACK,
	R_T2N_TRANS_END
} mac_router_t2n_state_t;

/******************************router state machinies**********************************/

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

/******************************node state machinies**********************************/
typedef enum {
/*    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,  */
	/*Basic mode of simple mode*/
	N_LISTENNING,
	N_TRANSMITTING,
	N_INIT,
} mac_node_basic_state_t;

typedef enum {
	/*Listening states of simple mode*/
	N_INIT_PREPARE,
	N_INIT_WAIT_TIMEOUT,
	N_INIT_END
} mac_node_init_state_t;

typedef enum {
	/*Listening states of simple mode*/
	N_LISTEN_CP_INIT,
	N_LISTEN_CP_LISTEN,
	N_LISTEN_CP_END,
	N_LISTEN_SLEEPING_INIT,
	N_LISTEN_SLEEPING,
	N_LISTEN_SLEEPING_END
} mac_node_listen_state_t;

typedef enum {
	/*Transmitting states of simple mode*/
	N_TRANS_TO_UNKOWN,
	N_TRANS_TO_ROUTER,
	N_TRANS_TO_NODE,
	N_BROADCAST
} mac_node_trans_state_t;

typedef enum {
	N_T2U_SEND_PREAMBLE_INIT = 0,
	N_T2U_SEND_PREAMBLE,
	N_T2U_WAIT_PREAMBLE_ACK,
	N_T2U_SEND_DATA,
	N_T2U_END
} mac_node_t2u_state_t;

typedef enum {
	/*Transmitting states of simple mode*/
	N_T2R_WAIT_CP_INIT = 0,
	N_T2R_WAIT_CP,
	N_T2R_TRANS_IN_CP,
	N_T2R_WAIT_CPTRANS_FEEDBACK,
	N_T2R_WAIT_BEACON,
	N_T2R_WAIT_OWN_SLOTS,
	N_T2R_TRANS_IN_VTDMA,
	N_T2R_WAIT_VTDMATRANS_FEEDBACK,
	N_T2R_TRANS_END
	/*Listening states of simple mode*/
} mac_node_t2r_state_t;

typedef enum {
	/*Transmitting states of router*/
	N_T2N_WAIT_CP_INIT = 0,
	N_T2N_WAIT_CP,
	N_T2N_TRANS_IN_CP,
	N_T2N_WAIT_CPTRANS_FEEDBACK,
	N_T2N_TRANS_END
} mac_node_t2n_state_t;


/******************************************************************************/

typedef enum {
    TX_FEEDBACK_UNDEF = -1,
    TX_FEEDBACK_SUCCESS,
    TX_FEEDBACK_NOACK,
    TX_FEEDBACK_BUSY
} iqueuemac_tx_feedback_t;

#define IQUEUEMAC_TX_FEEDBACK_INIT TX_FEEDBACK_UNDEF

/******************************************************************************/
typedef struct {
    uint8_t  addr[IQUEUEMAC_MAX_L2_ADDR_LEN];
} l2_id_t;


typedef struct {
    /* Address of neighbour node */
    l2_addr_t l2_addr;
    /* TX queue for this particular node */
    packet_queue_t queue;
    /* MAC type of the neighbor*/
    iqueuemac_type_t mac_type;  /* UNKONW when this neighbor is not phase-locked yet*/
    /* Phase relative to iqueuemac: the start of its CP period */
    uint32_t cp_phase;   /* in ticks*/
    /* Indicating that whether this neighbor is within the same cluster*/
    bool in_same_cluster;
} iqueuemac_tx_neighbour_t;

typedef struct {

	mac_node_basic_state_t node_basic_state;
	mac_node_init_state_t node_init_state;

	mac_node_listen_state_t node_listen_state;
	mac_node_trans_state_t node_trans_state;

	bool in_cp_period;
	bool node_new_cycle;

} node_states_t;


typedef struct {

	mac_router_basic_state_t router_basic_state;
	mac_router_init_state_t router_init_state;

	mac_router_listen_state_t router_listen_state;
	mac_router_trans_state_t router_trans_state;

	bool extend_cp;
	bool router_new_cycle;
	bool init_retry;
	uint16_t subchannel_occu_flags;

} router_states_t;

typedef struct {

	iqueuemac_device_broadcast_state_t device_broadcast_state;
	iqueuemac_device_t2n_state_t iqueuemac_device_t2n_state;
	iqueuemac_device_t2r_state_t iqueuemac_device_t2r_state;
	iqueuemac_device_t2u_state_t iqueuemac_device_t2u_state;

} device_states_t;




typedef struct {
	l2_addr_t node_addr;
	uint8_t queue_indicator;
	iqueuemac_type_t mac_type;
}rx_slots_schedule_unit;

/***  only record node-type neighbor ***/
typedef struct {
	l2_addr_t node_addr;
	iqueuemac_type_t mac_type;
}in_cluster_neighbor_info_t;

typedef struct {
	uint8_t total_slots_num;
	uint8_t sub_channel_seq;
}rx_vtdma_mana_t;


typedef struct {
	l2_addr_t node_addr;
	uint8_t seq;
}last_seq_info_t;


typedef struct {
    /* Internal state of reception state machine */

    packet_queue_t queue;
    packet_queue_node_t _queue_nodes[IQUEUEMAC_RX_QUEUE_SIZE];
    l2_addr_t l2_addr;
    gnrc_pktsnip_t* dispatch_buffer[IQUEUEMAC_DISPATCH_BUFFER_SIZE];

    rx_slots_schedule_unit rx_register_list[IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT];

    rx_vtdma_mana_t router_vtdma_mana;
    last_seq_info_t last_seq_info;

} iqueuemac_rx_t;


typedef struct {
	uint8_t sub_channel_seq;
	uint8_t slots_position;
	uint8_t slots_num;
	bool get_beacon;
}vtdma_para_t;


typedef struct {
    /* Internal state of reception state machine */
	packet_queue_node_t _queue_nodes[IQUEUEMAC_TX_QUEUE_SIZE];

	iqueuemac_tx_neighbour_t neighbours[IQUEUEMAC_NEIGHBOUR_COUNT];

	uint32_t preamble_sent;
	bool got_preamble_ack;
	uint32_t broadcast_seq;

	/* Packet that is currently scheduled to be sent */
	gnrc_pktsnip_t* tx_packet;
	/* Queue of destination node to which the current packet will be sent */
	iqueuemac_tx_neighbour_t* current_neighbour;

	/* Feedback of last packet that was sent */
	iqueuemac_tx_feedback_t tx_feedback;
	bool tx_finished;

	vtdma_para_t vtdma_para;

	uint8_t cp_retries;

	uint8_t no_ack_contuer;


} iqueuemac_tx_t;

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

	node_states_t   node_states;
	router_states_t router_states;
	device_states_t device_states;

	iqueuemac_rx_t rx;
	iqueuemac_tx_t tx;

	in_cluster_neighbor_info_t in_cluster_node_list[IQUEUEMAC_MAX_IN_CLUSTER_NEIGH_INFO_NUM];

	iqueuemac_timeout_t timeouts[IQUEUEMAC_TIMEOUT_COUNT];

    /* Own address */
    l2_addr_t own_addr;
    l2_addr_t father_router_addr;

    uint16_t sub_channel_num;
    uint16_t public_channel_num;

    /* Used to calculate wakeup times */
    uint32_t last_wakeup;    

    uint32_t backoff_phase_ticks;

    /* Track if a transmission might have corrupted a received packet */
    bool rx_started;
    bool packet_received;
    bool quit_current_cycle;
    bool got_preamble;
    bool cp_end;
    bool get_other_preamble;
    bool need_update;
    bool duty_cycle_started;
    bool phase_backoff;
    bool phase_changed;

    bool send_beacon_fail;
   
} iqueuemac_t;



#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_TYPES_H_ */
/** @} */

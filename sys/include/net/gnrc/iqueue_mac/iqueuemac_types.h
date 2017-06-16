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
#include <net/netdev.h>

#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include "net/gnrc/iqueue_mac/hdr.h"
#include "net/gnrc/iqueue_mac/timeout.h"

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
	DEVICE_T2U_SEND_PREAMBLE_PREPARE,
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

/******************************************************************************/
typedef struct {
    uint8_t  addr[IQUEUEMAC_MAX_L2_ADDR_LEN];
} l2_id_t;


typedef struct {
    /* Address of neighbour node */
    l2_addr_t l2_addr;
    /* MAC type of the neighbor*/

    /* Phase relative to iqueuemac: the start of its CP period */

    /* Indicating that whether this neighbor is within the same cluster*/
    //bool in_same_cluster;

} iqueuemac_tx_neighbour_t;

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
	iqueuemac_device_t2r_state_t iqueuemac_device_t2r_state;
	iqueuemac_device_t2u_state_t iqueuemac_device_t2u_state;

} device_states_t;

typedef struct {
	l2_addr_t node_addr;
	uint8_t queue_indicator;
	iqueuemac_type_t mac_type;
}rx_slots_schedule_unit;

typedef struct {
	uint8_t total_slots_num;
	uint8_t sub_channel_seq;
}rx_vtdma_mana_t;

typedef struct {
	l2_addr_t node_addr;
	uint8_t seq;
	uint8_t life_cycle;
}last_seq_info_t;

typedef struct {
	last_seq_info_t last_nodes[IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE];
	uint8_t queue_head;

}check_dup_pkt_t;

typedef struct {
	uint8_t sub_channel_seq;
	uint8_t slots_position;
	uint8_t slots_num;
	bool get_beacon;
}vtdma_para_t;

/******************************************************************************/
typedef struct iqueuemac {
    /* PID of IQUEUEMAC thread */
    kernel_pid_t pid;

    /* Internal state of MAC layer */
	iqueuemac_type_t mac_type;
	iqueuemac_router_state_t router_state;

	router_states_t router_states;
	device_states_t device_states;

	iqueuemac_timeout_t timeouts[IQUEUEMAC_TIMEOUT_COUNT];

    uint16_t sub_channel_num;
    uint16_t pub_channel_1;
    uint16_t pub_channel_2;
    uint16_t cur_pub_channel;
    uint8_t cp_backoff_counter;

    /* Used to calculate wakeup times */
    uint32_t last_wakeup;    

    uint32_t backoff_phase_ticks;

    /* Track if a transmission might have corrupted a received packet */
    bool rx_started;
    bool packet_received;
    bool quit_current_cycle;
    bool got_preamble;
    bool cp_end;
    bool vtdma_end;
    bool get_other_preamble;
    bool need_update;
    bool duty_cycle_started;
    bool phase_backoff;
    bool phase_changed;

    bool send_beacon_fail;
    bool rx_memory_full;
   
} iqueuemac_t;



#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_TYPES_H_ */
/** @} */

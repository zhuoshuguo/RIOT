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
#define GOMACH_EVENT_RTT_NEW_CYCLE           (0x4304)


#define IQUEUEMAC_EVENT_TIMEOUT_TYPE        (0x4400)

#define IQUEUEMAC_PHASE_UNINITIALIZED (0)

#define IQUEUEMAC_PHASE_MAX             (-1)

/******************************************************************************/
typedef enum {
	GNRC_GOMACH_BCAST_INIT = 0,
	GNRC_GOMACH_BCAST_SEND,
	GNRC_GOMACH_BCAST_WAIT_TX_FINISH,
	GNRC_GOMACH_BCAST_WAIT_NEXT_TX,
	GNRC_GOMACH_BCAST_END
} gnrc_gomach_bcast_state_t;

typedef enum {
    GNRC_GOMACH_T2K_INIT = 0,
	GNRC_GOMACH_T2K_WAIT_CP,
	GNRC_GOMACH_T2K_TRANS_IN_CP,
	GNRC_GOMACH_T2K_WAIT_CPTX_FEEDBACK,
	GNRC_GOMACH_T2K_WAIT_BEACON,
	GNRC_GOMACH_T2K_WAIT_SLOTS,
	GNRC_GOMACH_T2K_VTDMA_TRANS,
	GNRC_GOMACH_T2K_WAIT_VTDMA_FEEDBACK,
	GNRC_GOMACH_T2K_END
} gnrc_gomach_t2k_state_t;

typedef enum {
	GNRC_GOMACH_T2U_INIT = 0,
	GNRC_GOMACH_T2U_PREAMBLE_PREPARE,
	GNRC_GOMACH_T2U_SEND_PREAMBLE,
	GNRC_GOMACH_T2U_WAIT_PREAMBLE_TX,
	GNRC_GOMACH_T2U_WAIT_PREAMBLE_ACK,
	GNRC_GOMACH_T2U_SEND_DATA,
	GNRC_GOMACH_T2U_WAIT_DATA_TX,
	GNRC_GOMACH_T2U_END
} gnrc_gomach_t2u_state_t;

/******************************router state machinies**********************************/
typedef enum {
/*    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,  */
    /*Basic mode of simple mode*/
    GNRC_GOMACH_INIT = 0,
    GNRC_GOMACH_LISTEN,
    GNRC_GOMACH_TRANSMIT
} gnrc_gomach_basic_state_t;

typedef enum {
    GNRC_GOMACH_INIT_PREPARE = 0,
	GNRC_GOMACH_INIT_ANNC_SUBCHAN,
	GNRC_GOMACH_INIT_WAIT_FEEDBACK,
	GNRC_GOMACH_INIT_END
} gnrc_gomach_init_state_t;

typedef enum {
    /*Listening states of simple mode*/
	GNRC_GOMACH_LISTEN_CP_INIT = 0,
	GNRC_GOMACH_LISTEN_CP_LISTEN,
	GNRC_GOMACH_LISTEN_CP_END,
	GNRC_GOMACH_LISTEN_SEND_BEACON,
	GNRC_GOMACH_LISTEN_WAIT_BEACON_TX,
	GNRC_GOMACH_LISTEN_VTDMA_INIT,
	GNRC_GOMACH_LISTEN_VTDMA,
	GNRC_GOMACH_LISTEN_VTDMA_END,
	GNRC_GOMACH_LISTEN_SLEEP_INIT,
	GNRC_GOMACH_LISTEN_SLEEP,
	GNRC_GOMACH_LISTEN_SLEEP_END
} gnrc_gomach_listen_state_t;

typedef enum {
    GNRC_GOMACH_TRANS_TO_UNKNOWN,
    GNRC_GOMACH_TRANS_TO_KNOWN,
    GNRC_GOMACH_BROADCAST
} gnrc_gomach_transmit_state_t;

/******************************************************************************/
typedef struct {
    uint8_t addr[IQUEUEMAC_MAX_L2_ADDR_LEN];
} l2_id_t;

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
    uint16_t sub_channel_seq;
    uint8_t slots_position;
    uint8_t slots_num;
    bool get_beacon;
}vtdma_para_t;

/******************************************************************************/
typedef struct gomach {
    /* PID of IQUEUEMAC thread */
    kernel_pid_t pid;

    /* Internal state of MAC layer */
    gnrc_gomach_basic_state_t basic_state;
    gnrc_gomach_init_state_t init_state;

    gomach_timeout_t timeouts[IQUEUEMAC_TIMEOUT_COUNT];

    uint16_t subchannel_occu_flags;
    uint16_t sub_channel_num;
    uint16_t pub_channel_1;
    uint16_t pub_channel_2;
    uint16_t cur_pub_channel;
    uint8_t cp_backoff_counter;

    /* Used to calculate wakeup times */
    uint32_t last_wakeup;

    uint32_t backoff_phase_ticks;

    /* Track if a transmission might have corrupted a received packet */
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

} gomach_t;

#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_TYPES_H_ */
/** @} */

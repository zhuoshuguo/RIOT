/*
 * Copyright (C) 2016 Shuguo Zhuo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_gnrc_gomach
 * @{
 *
 * @file
 * @brief       Internal types of GoMacH
 * @internal
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef NET_GNRC_GOMACH_GOMACH_TYPES_H
#define NET_GNRC_GOMACH_GOMACH_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include "kernel_types.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/gomach/hdr.h"
#include "net/gnrc/gomach/gomach.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNRC_GOMACH_EVENT_RTT_TYPE          (0x4300)

#define GNRC_GOMACH_EVENT_RTT_NEW_CYCLE     (0x4301)

#define GNRC_GOMACH_EVENT_TIMEOUT_TYPE      (0x4400)

#define GNRC_GOMACH_PHASE_UNINITIALIZED     (0)

#define GNRC_GOMACH_PHASE_MAX               (-1)

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

typedef enum {
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

/* @brief   Type to pass information about parsing */
typedef struct {
    iqueuemac_hdr_t *header;    /**< iqueuemac header of packet */
    l2_addr_t src_addr;         /**< copied source address of packet  */
    l2_addr_t dst_addr;         /**< copied destination address of packet */
    uint8_t seq;                /**< seq of the received packet */
} iqueuemac_packet_info_t;

typedef struct {
    uint16_t sub_channel_seq;
    uint8_t slots_position;
    uint8_t slots_num;
    bool get_beacon;
}vtdma_para_t;

typedef enum {
    TIMEOUT_DISABLED = 0,
    TIMEOUT_BROADCAST_FINISH,
    TIMEOUT_BROADCAST_INTERVAL,
    TIMEOUT_PREAMBLE,
    TIMEOUT_MAX_PREAM_INTERVAL,
    TIMEOUT_PREAMBLE_DURATION,
    TIMEOUT_WAIT_CP,
    TIMEOUT_WAIT_BEACON,
    TIMEOUT_WAIT_OWN_SLOTS,
    TIMEOUT_WAIT_RE_PHASE_LOCK,
    /*****************router******************/
    TIMEOUT_CP_END,
    TIMEOUT_CP_MAX,
    TIMEOUT_WAIT_RX_END,
    TIMEOUT_VTDMA,
    /*****************simple-node******************/
    TIMEOUT_BEACON_END

} gomach_timeout_type_t;

typedef struct {
    xtimer_t timer;
    msg_t msg;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    gomach_timeout_type_t type;
} gomach_timeout_t;

typedef struct gomach {
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

#endif /* NET_GNRC_GOMACH_GOMACH_TYPES_H */
/** @} */

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

#ifdef __cplusplus
extern "C" {
#endif

#define GNRC_GOMACH_EVENT_RTT_TYPE          (0x4300)

#define GNRC_GOMACH_EVENT_RTT_NEW_CYCLE     (0x4301)

#define GNRC_GOMACH_EVENT_TIMEOUT_TYPE      (0x4400)

#define GNRC_GOMACH_PHASE_UNINITIALIZED     (0)

#define GNRC_GOMACH_PHASE_MAX               (-1)

#ifndef GNRC_GOMACH_CHECK_DUPPKT_BUFFER_SIZE
#define GNRC_GOMACH_CHECK_DUPPKT_BUFFER_SIZE             (8U)
#endif

#ifndef GNRC_GOMACH_TIMEOUT_COUNT
#define GNRC_GOMACH_TIMEOUT_COUNT             (5U)
#endif

#ifndef GNRC_GOMACH_SLOTS_SCHEDULE_UNIT
#define GNRC_GOMACH_SLOTS_SCHEDULE_UNIT           (11U)
#endif

typedef enum {
	GNRC_GOMACH_TYPE_UNKNOWN = 0,
	GNRC_GOMACH_TYPE_KNOWN,
} gnrc_gomach_type_t;

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
    uint8_t addr[IEEE802154_LONG_ADDRESS_LEN];
} gnrc_gomach_l2_id_t;

typedef struct {
    gnrc_gomach_l2_addr_t node_addr;
    uint8_t queue_indicator;
    gnrc_gomach_type_t mac_type;
} gnrc_gomach_slots_sched_unit_t;

typedef struct {
    uint8_t total_slots_num;
    uint8_t sub_channel_seq;
} gnrc_gomach_vtdma_manag_t;

typedef struct {
    gnrc_gomach_l2_addr_t node_addr;
    uint8_t seq;
    uint8_t life_cycle;
} gnrc_gomach_dupchk_unit_t;

typedef struct {
    gnrc_gomach_dupchk_unit_t last_nodes[GNRC_GOMACH_CHECK_DUPPKT_BUFFER_SIZE];
    uint8_t queue_head;
} gnrc_gomach_dupchk_t;

/* @brief   Type to pass information about parsing */
typedef struct {
    gnrc_gomach_hdr_t *header;    /**< GoMacH header of packet */
    gnrc_gomach_l2_addr_t src_addr;         /**< copied source address of packet  */
    gnrc_gomach_l2_addr_t dst_addr;         /**< copied destination address of packet */
    uint8_t seq;                /**< seq of the received packet */
} gnrc_gomach_packet_info_t;

typedef struct {
    uint16_t sub_channel_seq;
    uint8_t slots_position;
    uint8_t slots_num;
} gnrc_gomach_vtdma_t;

typedef enum {
	GNRC_GOMACH_TIMEOUT_DISABLED = 0,
	GNRC_GOMACH_TIMEOUT_BCAST_FINISH,
	GNRC_GOMACH_TIMEOUT_BCAST_INTERVAL,
	GNRC_GOMACH_TIMEOUT_PREAMBLE,
	GNRC_GOMACH_TIMEOUT_MAX_PREAM_INTERVAL,
	GNRC_GOMACH_TIMEOUT_PREAM_DURATION,
	GNRC_GOMACH_TIMEOUT_WAIT_CP,
	GNRC_GOMACH_TIMEOUT_WAIT_BEACON,
	GNRC_GOMACH_TIMEOUT_WAIT_SLOTS,
	GNRC_GOMACH_TIMEOUT_CP_END,
	GNRC_GOMACH_TIMEOUT_CP_MAX,
	GNRC_GOMACH_TIMEOUT_WAIT_RX_END,
	GNRC_GOMACH_TIMEOUT_VTDMA,
} gnrc_gomach_timeout_type_t;

typedef struct {
    xtimer_t timer;
    msg_t msg;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    gnrc_gomach_timeout_type_t type;
} gnrc_gomach_timeout_t;

#define GNRC_GOMACH_TIMEOUT_INIT  { {}, {}, false, GNRC_GOMACH_TIMEOUT_DISABLED }

typedef struct gomach {
    /* Internal state of MAC layer */
    gnrc_gomach_basic_state_t basic_state;
    gnrc_gomach_init_state_t init_state;

    gnrc_gomach_timeout_t timeouts[GNRC_GOMACH_TIMEOUT_COUNT];

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

} gnrc_gomach_t;

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_GOMACH_GOMACH_TYPES_H */
/** @} */

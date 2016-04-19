/*
 * Copyright (C) 2016 Shuguo Zhuo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_iqueue_mac traffic adaptive  MAC layer
 * @ingroup     net
 * @{
 *
 * @file
 * @brief       Header definition of IQUEUE_MAC protocol
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_IQUEUEMAC_HDR_H_
#define GNRC_IQUEUEMAC_HDR_H_

#include <stdint.h>
#include <stdbool.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

typedef struct {
    uint8_t  addr[IQUEUEMAC_MAX_L2_ADDR_LEN];
    uint8_t  len;
} l2_addr_t;
#define IQUEUEMAC_L2_ADDR_INIT      { {0}, 0 }

/******************************************************************************/

typedef enum {
    FRAMETYPE_BEACON = 1,
    FRAMETYPE_DATA,
    FRAMETYPE_PREAMBLE,
	FRAMETYPE_PREAMBLE_ACK,
    FRAMETYPE_BROADCAST
} iqueuemac_frame_type_t;

/******************************************************************************/

/**
 * @brief   iqueuemac header
 */
typedef struct __attribute__((packed)) {
	iqueuemac_frame_type_t type; /**< type of frame */
} iqueuemac_hdr_t;


/**
 * @brief   iqueuemac Beacon frame
 */
typedef struct __attribute__((packed)) {
	iqueuemac_hdr_t header;
	//l2_addr_t source_addr;
	uint32_t next_cp_time;     /* phase of this device*/
	uint8_t sub_channel_seq;
	uint8_t schedulelist_size;
} iqueuemac_frame_beacon_t;

/**
 * @brief   iqueuemac unicast data frame
 */
typedef struct __attribute__((packed)) {
	iqueuemac_hdr_t header;
	uint8_t queue_indicator;  /* The first bit of the indicator is used to indicate MAC type: router or dimple node*/
} iqueuemac_frame_data_t;


/**
 * @brief   iqueuemac broadcast preamble frame
 */
typedef struct __attribute__((packed)) {
	iqueuemac_hdr_t header;
	l2_addr_t dst_addr;  /* Preamble is broadcast, so dst addr is needed to show who is the intended receiver*/
} iqueuemac_frame_preamble_t;


/**
 * @brief   iqueuemac broadcast preamble_ack frame
 */
typedef struct __attribute__((packed)) {
	iqueuemac_hdr_t header;
	l2_addr_t dst_addr;  /* Preamble_ACK is broadcast, so dst addr is needed to show who is the intended receiver*/
	iqueuemac_type_t device_type;
	l2_addr_t father_router;
	uint32_t next_cp_time;   /* phase of this device*/
} iqueuemac_frame_preamble_ack_t;


/**
 * @brief   iqueuemac broadcast data frame
 */
typedef struct __attribute__((packed)) {
    lwmac_hdr_t header;
    uint8_t seq_nr;
} iqueuemac_frame_broadcast_t;

//void lwmac_print_hdr(lwmac_hdr_t* hdr);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_IQUEUEMAC_HDR_H_ */
/** @} */

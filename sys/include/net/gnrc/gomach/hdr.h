/*
 * Copyright (C) 2016 INRIA
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
 * @brief       Header definition of GoMacH
 * @internal
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef NET_GNRC_GOMACH_HDR_H
#define NET_GNRC_GOMACH_HDR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GNRC_GOMACH_MAX_L2_ADDR_LEN
#define GNRC_GOMACH_MAX_L2_ADDR_LEN           (8U)
#endif

typedef struct {
    uint8_t addr[GNRC_GOMACH_MAX_L2_ADDR_LEN];
    uint8_t len;
} gnrc_gomach_l2_addr_t;

#define GNRC_GOMACH_L2_ADDR_INIT      { { 0 }, 0 }

typedef enum {
    GNRC_GOMACH_FRAME_BEACON = 1,
    GNRC_GOMACH_FRAME_DATA,
    GNRC_GOMACH_FRAME_PREAMBLE,
    GNRC_GOMACH_FRAME_PREAMBLE_ACK,
    GNRC_GOMACH_FRAME_BROADCAST,
    GNRC_GOMACH_FRAME_ANNOUNCE
} gnrc_gomach_frame_type_t;

/**
 * @brief   GoMacH header
 */
typedef struct {
    gnrc_gomach_frame_type_t type; /**< type of frame */
} gnrc_gomach_hdr_t;

/**
 * @brief   GoMacH Beacon frame
 */
typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    uint8_t sub_channel_seq;
    uint8_t schedulelist_size;
} gnrc_gomach_frame_beacon_t;

/**
 * @brief   GoMacH data frame
 */
typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    uint8_t queue_indicator;
} gnrc_gomach_frame_data_t;

typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    uint8_t subchannel_seq;
} gnrc_gomach_frame_announce_t;

/**
 * @brief   GoMacH broadcast preamble frame
 */
typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    gnrc_gomach_l2_addr_t dst_addr;
} gnrc_gomach_frame_preamble_t;


/**
 * @brief   GoMacH broadcast preamble_ack frame
 */
typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    gnrc_gomach_l2_addr_t dst_addr;
    uint32_t phase_in_ticks;    /* phase of this device*/
} gnrc_gomach_frame_preamble_ack_t;

/**
 * @brief   GoMacH broadcast data frame
 */
typedef struct __attribute__((packed)) {
    gnrc_gomach_hdr_t header;
    uint8_t seq_nr;
} gnrc_gomach_frame_broadcast_t;

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_GOMACH_HDR_H */
/** @} */

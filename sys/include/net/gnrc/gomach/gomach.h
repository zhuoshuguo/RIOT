/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_gnrc_gomach A traffic-adaptive multi-channel MAC
 * @ingroup     net
 * @brief       A traffic adaptive MAC protocol that provides high traffic adaptability,
 *              high energy efficiency and high robustness.
 * @{
 *
 * @file
 * @brief       Implementation of GoMacH protocol
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef NET_GNRC_GOMACH_GOMACH_H
#define NET_GNRC_GOMACH_GOMACH_H

#include "kernel_types.h"
#include "net/gnrc/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef GNRC_GOMACH_SUPERFRAME_DURATION_US
#define GNRC_GOMACH_SUPERFRAME_DURATION_US        (200U * 1000)
#endif

#ifndef GNRC_GOMACH_CP_DURATION_US
#define GNRC_GOMACH_CP_DURATION_US        (10U * 1000)
#endif

#ifndef GNRC_GOMACH_CP_RANDOM_END_US
#define GNRC_GOMACH_CP_RANDOM_END_US        (1U * 1000)
#endif

#ifndef GNRC_GOMACH_CP_DURATION_MAX_US
#define GNRC_GOMACH_CP_DURATION_MAX_US        (5 * GNRC_GOMACH_CP_DURATION_US)
#endif

//this can be merged into GNRC_GOMACH_CP_DURATION_MAX_US.
#ifndef GNRC_GOMACH_WAIT_BEACON_TIME_US
#define GNRC_GOMACH_WAIT_BEACON_TIME_US        (GNRC_GOMACH_CP_DURATION_MAX_US)
#endif

#ifndef GNRC_GOMACH_CP_MIN_GAP_US
#define GNRC_GOMACH_CP_MIN_GAP_US        (25U * 1000)
#endif

#ifndef GNRC_GOMACH_REPHASE_ADJUST_US
#define GNRC_GOMACH_REPHASE_ADJUST_US        (5U * 1000)
#endif

#ifndef GNRC_GOMACH_WAIT_RX_END_US
#define GNRC_GOMACH_WAIT_RX_END_US        (3U * 1000)
#endif

#ifndef GNRC_GOMACH_MAX_PREAM_INTERVAL_US
#define GNRC_GOMACH_MAX_PREAM_INTERVAL_US        (5U * 1000)
#endif

#ifndef GNRC_GOMACH_PREAMBLE_INTERVAL_US
#define GNRC_GOMACH_PREAMBLE_INTERVAL_US        (1U * 1000)
#endif

#ifndef GNRC_GOMACH_BCAST_INTERVAL_US
#define GNRC_GOMACH_BCAST_INTERVAL_US        (1U * 1000)
#endif

#ifndef GNRC_GOMACH_PREAMBLE_DURATION_US
#define GNRC_GOMACH_PREAMBLE_DURATION_US        (21 * GNRC_GOMACH_SUPERFRAME_DURATION_US / 10)
#endif

#ifndef GNRC_GOMACH_VTDMA_SLOT_SIZE_US
#define GNRC_GOMACH_VTDMA_SLOT_SIZE_US        (5U * 1000)
#endif

#ifndef GNRC_GOMACH_TX_BUSY_THRESHOLD
#define GNRC_GOMACH_TX_BUSY_THRESHOLD      (5U)
#endif

#ifndef GNRC_GOMACH_CP_EXTEND_THRESHOLD
#define GNRC_GOMACH_CP_EXTEND_THRESHOLD      (5U)
#endif

#ifndef GNRC_GOMACH_RX_DUPCHK_UNIT_LIFE
#define GNRC_GOMACH_RX_DUPCHK_UNIT_LIFE            (30U)
#endif

#ifndef GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM
#define GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM           (25U)
#endif

#ifndef GNRC_GOMACH_REPHASELOCK_THRESHOLD
#define GNRC_GOMACH_REPHASELOCK_THRESHOLD      (4U)
#endif

#ifndef GNRC_GOMACH_T2U_RETYR_THRESHOLD
#define GNRC_GOMACH_T2U_RETYR_THRESHOLD      (2U)
#endif

/**
 * @brief   Initialize an instance of the GoMacH layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the GoMacH instance
 * @param[in] name          name of the thread housing the GoMacH instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of GoMacH thread on success
 * @return                  -EINVAL if creation of thread fails
 * @return                  -ENODEV if *dev* is invalid
 */
kernel_pid_t gnrc_gomach_init(char *stack, int stacksize, char priority,
                                 const char *name, gnrc_netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_GOMACH_GOMACH_H */
/** @} */

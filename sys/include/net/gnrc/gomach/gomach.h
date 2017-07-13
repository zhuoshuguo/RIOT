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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gnrc_netdev gnrc_netdev_t;

typedef enum {
    UNKNOWN = 0,
	KNOWN,
} iqueuemac_type_t;

#ifndef IQUEUEMAC_WAIT_RTT_STABLE_US
#define IQUEUEMAC_WAIT_RTT_STABLE_US        (3000U * 1000)
#endif

#ifndef IQUEUEMAC_SUPERFRAME_DURATION_US
#define IQUEUEMAC_SUPERFRAME_DURATION_US        (200U * 1000)
#endif

#ifndef IQUEUEMAC_CP_DURATION_US
#define IQUEUEMAC_CP_DURATION_US        (10U * 1000)
#endif

#ifndef IQUEUEMAC_CP_RANDOM_END_US
#define IQUEUEMAC_CP_RANDOM_END_US        (1U * 1000)
#endif

#ifndef IQUEUEMAC_CP_DURATION_MAX_US
#define IQUEUEMAC_CP_DURATION_MAX_US        (5 * IQUEUEMAC_CP_DURATION_US)
#endif

//this can be merged into IQUEUEMAC_CP_DURATION_MAX_US.
#ifndef IQUEUEMAC_WAIT_BEACON_TIME_US
#define IQUEUEMAC_WAIT_BEACON_TIME_US        (IQUEUEMAC_CP_DURATION_MAX_US)
#endif

#ifndef IQUEUEMAC_RECEPTION_MAGIN_US
#define IQUEUEMAC_RECEPTION_MAGIN_US        (5U * 1000)
#endif

#ifndef IQUEUEMAC_CP_MIN_GAP_US
#define IQUEUEMAC_CP_MIN_GAP_US        (25U * 1000)
#endif

#ifndef IQUEUEMAC_REPHASE_ADJUST_US
#define IQUEUEMAC_REPHASE_ADJUST_US        (5U * 1000)
#endif

#ifndef IQUEUEMAC_WAIT_RX_END_US
#define IQUEUEMAC_WAIT_RX_END_US        (3U * 1000)
#endif

#ifndef IQUEUEMAC_MAX_PREAM_INTERVAL_US
#define IQUEUEMAC_MAX_PREAM_INTERVAL_US        (5U * 1000)
#endif

#ifndef IQUEUEMAC_PREAMBLE_INTERVAL_US
#define IQUEUEMAC_PREAMBLE_INTERVAL_US        (1U * 1000)
#endif

#ifndef IQUEUEMAC_BROADCAST_INTERVAL_US
#define IQUEUEMAC_BROADCAST_INTERVAL_US        (1U * 1000)
#endif

#ifndef IQUEUEMAC_PREAMBLE_DURATION_US
#define IQUEUEMAC_PREAMBLE_DURATION_US        (21 * IQUEUEMAC_SUPERFRAME_DURATION_US / 10)
#endif

#ifndef IQUEUEMAC_RE_PHASE_LOCK_ADVANCE_US
#define IQUEUEMAC_RE_PHASE_LOCK_ADVANCE_US        (IQUEUEMAC_SUPERFRAME_DURATION_US / 10)
#endif

#ifndef IQUEUEMAC_VTDMA_DURATION_US
#define IQUEUEMAC_VTDMA_DURATION_US        (10U * 1000)
#endif

#ifndef IQUEUEMAC_VTDMA_SLOT_SIZE_US
#define IQUEUEMAC_VTDMA_SLOT_SIZE_US        (5U * 1000)
#endif

#ifndef IQUEUEMAC_WAIT_CP_SECUR_GAP_US
#define IQUEUEMAC_WAIT_CP_SECUR_GAP_US        (5U * 1000)
#endif

#ifndef IQUEUEMAC_QUIT_CP_MARGIN_US
#define IQUEUEMAC_QUIT_CP_MARGIN_US        (IQUEUEMAC_CP_DURATION_US / 2)
#endif

#ifndef IQUEUEMAC_TIMEOUT_COUNT
#define IQUEUEMAC_TIMEOUT_COUNT             (5U)
#endif

#ifndef IQUEUEMAC_MAX_TX_BUSY_COUNTER
#define IQUEUEMAC_MAX_TX_BUSY_COUNTER      (5U)
#endif

#ifndef IQUEUEMAC_MAX_CP_BACKOFF_COUNTER
#define IQUEUEMAC_MAX_CP_BACKOFF_COUNTER      (5U)
#endif

#ifndef IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE
#define IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE             (8U)
#endif

#ifndef IQUEUEMAC_RX_CHECK_DUPPKT_UNIT_MAX_LIFE
#define IQUEUEMAC_RX_CHECK_DUPPKT_UNIT_MAX_LIFE            (30U)
#endif

#ifndef IQUEUEMAC_MAX_L2_ADDR_LEN
#define IQUEUEMAC_MAX_L2_ADDR_LEN           (8U)
#endif


#ifndef IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT
#define IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT           (11U)
#endif

#ifndef IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM
#define IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM           (25U)
#endif

#ifndef IQUEUEMAC_REPHASELOCK_THRESHOLD
#define IQUEUEMAC_REPHASELOCK_THRESHOLD      (4U)
#endif

#ifndef IQUEUEMAC_T2U_RETYR_THRESHOLD
#define IQUEUEMAC_T2U_RETYR_THRESHOLD      (2U)
#endif

#define IQUEUEMAC_LPM_MASK                  (1 << 17)


/**
 * @brief   Initialize an instance of the IQUEUE_MAC layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the IQUEUE_MAC instance
 * @param[in] name          name of the thread housing the IQUEUE_MAC instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of IQUEUE_MAC thread on success
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

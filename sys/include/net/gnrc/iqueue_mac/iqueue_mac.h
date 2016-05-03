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
 * @brief       Traffic adaptive MAC protocol that allows for duty cycling to save
 *              energy.
 * @{
 *
 * @file
 * @brief       Interface definition for the IQUEUE_MAC protocol
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_IQUEUE_MAC_H
#define GNRC_IQUEUE_MAC_H

#include <kernel_types.h>
#include <net/gnrc/netdev2.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
	UNKNOWN = 0,
    ROUTER,
    NODE
} iqueuemac_type_t;

#define MAC_TYPE   NODE // ROUTER //               NODE //        ROUTER

#ifndef IQUEUEMAC_SUPERFRAME_DURATION_US
#define IQUEUEMAC_SUPERFRAME_DURATION_US        (3000U * 1000)
#endif

#ifndef IQUEUEMAC_CP_DURATION_US
#define IQUEUEMAC_CP_DURATION_US        (2600U * 1000)
#endif

#ifndef IQUEUEMAC_PREAMBLE_INTERVAL_US
#define IQUEUEMAC_PREAMBLE_INTERVAL_US        (1000U * 1000)
#endif

#ifndef IQUEUEMAC_PREAMBLE_DURATION_US
#define IQUEUEMAC_PREAMBLE_DURATION_US        (4000U * 1000)
#endif

#ifndef IQUEUEMAC_SLEEP_DURATION_US
#define IQUEUEMAC_SLEEP_DURATION_US        (450U * 1000)
#endif

#ifndef IQUEUEMAC_VTDMA_DURATION_US
#define IQUEUEMAC_VTDMA_DURATION_US        (10U * 1000)
#endif

#ifndef IQUEUEMAC_VTDMA_LONG_DURATION_US
#define IQUEUEMAC_VTDMA_LONG_DURATION_US        (100U * 1000)
#endif

#ifndef IQUEUEMAC_VTDMA_LONG_LONG_DURATION_US
#define IQUEUEMAC_VTDMA_LONG_LONG_DURATION_US        (110U * 1000)
#endif

#ifndef IQUEUEMAC_TIMEOUT_COUNT
#define IQUEUEMAC_TIMEOUT_COUNT             (3U)
#endif

/******* it seems that this should not be larger than 7 !!!!! *******/
#ifndef IQUEUEMAC_TX_QUEUE_SIZE
#define IQUEUEMAC_TX_QUEUE_SIZE             (7U)
#endif

#ifndef IQUEUEMAC_RX_QUEUE_SIZE
#define IQUEUEMAC_RX_QUEUE_SIZE             (8U)
#endif

/******* it seems that this should not be larger than 3 !!!!! *******/
#ifndef IQUEUEMAC_DISPATCH_BUFFER_SIZE
#define IQUEUEMAC_DISPATCH_BUFFER_SIZE      (3U)
#endif

#ifndef IQUEUEMAC_MAX_L2_ADDR_LEN
#define IQUEUEMAC_MAX_L2_ADDR_LEN           (2U)
#endif


#ifndef IQUEUEMAC_NEIGHBOUR_COUNT
#define IQUEUEMAC_NEIGHBOUR_COUNT           (8U)
#endif


void iqueuemac_set_rtt_alarm(uint32_t alarm, void *arg);


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
kernel_pid_t gnrc_iqueuemac_init(char *stack, int stacksize, char priority,
						   const char *name, gnrc_netdev2_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_H */
/** @} */

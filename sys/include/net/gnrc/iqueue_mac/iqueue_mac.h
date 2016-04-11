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


#ifndef IQUEUE_CP_DURATION_US
#define IQUEUE_CP_DURATION_US        (2000U * 1000)
#endif

#ifndef IQUEUE_SLEEP_DURATION_US
#define IQUEUE_SLEEP_DURATION_US        (4000U * 1000)
#endif

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

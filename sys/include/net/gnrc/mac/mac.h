/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_gnrc_mac  A common MAC type for providing key MAC parameters and helper functions
 * @ingroup     net
 * @brief       A common MAC type for providing key MAC parameters and helper functions.
 *              
 * @{
 *
 * @file
 * @brief       Interface definition for the GNRC_MAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_MAC_H
#define GNRC_MAC_H

#include <kernel_types.h>
#include <net/gnrc/netdev2.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Count of parallel MAC timeouts.
 */
#ifndef GNRC_MAC_TIMEOUT_COUNT
#define GNRC_MAC_TIMEOUT_COUNT             (3U)
#endif

/**
 * @brief   The default rx queue size for incoming packets
 */
#ifndef GNRC_MAC_RX_QUEUE_SIZE
#define GNRC_MAC_RX_QUEUE_SIZE             (8U)
#endif

/**
 * @brief   The default buffer size for storing dispatching packets.
 */
#ifndef GNRC_MAC_DISPATCH_BUFFER_SIZE
#define GNRC_MAC_DISPATCH_BUFFER_SIZE      (8U)
#endif

/**
 * @brief   Count of nodes in one-hop distance whose wakeup phase is tracked
 */
#ifndef GNRC_MAC_NEIGHBOUR_COUNT
#define GNRC_MAC_NEIGHBOUR_COUNT           (8U)
#endif

/**
 * @brief   The default queue size for packets coming from higher layers
 */
#ifndef GNRC_MAC_TX_QUEUE_SIZE
#define GNRC_MAC_TX_QUEUE_SIZE             (8U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_H */
/** @} */

/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Lightweight MAC protocol that allows for duty cycling to save
 *              energy.
 * @{
 *
 * @file
 * @brief       Interface definition for the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_MAC_H
#define GNRC_MAC_H

#include <kernel_types.h>
#include <net/gnrc/netdev2.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Count of parallel timeouts. Shouldn't needed to be changed.
 */
#ifndef GNRC_MAC_TIMEOUT_COUNT
#define GNRC_MAC_TIMEOUT_COUNT             (3U)
#endif

/* Max link layer address length in bytes */
#ifndef GNRC_MAC_MAX_L2_ADDR_LEN
#define GNRC_MAC_MAX_L2_ADDR_LEN           (8U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_H */
/** @} */

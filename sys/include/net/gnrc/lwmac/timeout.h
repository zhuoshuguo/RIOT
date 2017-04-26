/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup net_lwmac
 * @{
 *
 * @file
 * @brief       Timeout handling of LWMAC
 *
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_LWMAC_TIMEOUT_H
#define GNRC_LWMAC_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>

#include "net/gnrc/netdev.h"
#include "net/gnrc/lwmac/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Static initializer for lwmac_timeout_t.
 */
#define LWMAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

/**
 * @brief   Set Lwmac timeout of type @p type of offset @p offset.
 *
 * @param[in,out] gnrc_netdev  gnrc_netdev structure
 * @param[in]     type         Lwmac timeout type
 * @param[in]     offset       timeout offset
 */
void lwmac_set_timeout(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type, uint32_t offset);

/**
 * @brief   Clear Lwmac timeout of type @p type.
 *
 * @param[in,out] gnrc_netdev  gnrc_netdev structure
 * @param[in]     type         Lwmac timeout type
 */
void lwmac_clear_timeout(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type);

/**
 * @brief   Check whether Lwmac timeout of type @p type is running.
 *
 * @param[in]     gnrc_netdev  gnrc_netdev structure
 * @param[in]     type         Lwmac timeout type
 *
 * @return        true, if timeout of type @p type is running.
 * @return        false, if timeout of type @p type is not running.
 */
bool lwmac_timeout_is_running(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type);

/**
 * @brief   Check whether Lwmac timeout of type @p type is expired. It will clear
 *          the timeout once it is found expired.
 *
 * @param[in,out] gnrc_netdev  gnrc_netdev structure
 * @param[in]     type         Lwmac timeout type
 *
 * @return        true, if timeout of type @p type is expired.
 * @return        false, if timeout of type @p type is not expired, or not exist.
 */
bool lwmac_timeout_is_expired(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type);

/**
 * @brief   Reset all Lwmac timeouts.
 *
 * @param[in,out] gnrc_netdev  gnrc_netdev structure
 */
void lwmac_reset_timeouts(gnrc_netdev_t *gnrc_netdev);

/**
 * @brief   Make a specific Lwmac timeout expired.
 *
 * @param[in,out] timeout   Lwmac tiemout
 */
void lwmac_timeout_make_expire(lwmac_timeout_t *timeout);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_TIMEOUT_H */
/** @} */

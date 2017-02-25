/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_lwmac
 * @{
 *
 * @file
 * @brief       Timeout handling of LWMAC
 * @internal
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef LWMAC_TIMEOUT_H
#define LWMAC_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>
#include <msg.h>
#include <xtimer.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Forward lwmac declaration
 */
typedef struct lwmac lwmac_t;

/**
 * @brief Forward gnrc_netdev2 declaration
 */
typedef struct gnrc_netdev2 gnrc_netdev2_t;

/**
 * @brief lwmac timeout types
 */
typedef enum {
    TIMEOUT_DISABLED = 0,
    TIMEOUT_WR,
    TIMEOUT_NO_RESPONSE,
    TIMEOUT_WA,
    TIMEOUT_DATA,
    TIMEOUT_WAIT_FOR_DEST_WAKEUP,
    TIMEOUT_WAKEUP_PERIOD,
    TIMEOUT_NEXT_BROADCAST,
    TIMEOUT_BROADCAST_END,
} lwmac_timeout_type_t;

 /**
 * @brief Lwmac timeout structure
 */
typedef struct {
    xtimer_t timer;
    msg_t msg;
    bool expired;              /**< If type != DISABLED, this indicates if timeout has expired */
    lwmac_timeout_type_t type;
} lwmac_timeout_t;

/**
 * @brief Static initializer for lwmac_timeout_t.
 */
#define LWMAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

/**
 * @brief Set Lwmac timeout of type @type of offset @p offset.
 *
 * @param[in,out] gnrc_netdev2  gnrc_netdev2 structure
 * @param[in]     type          Lwmac timeout type
 * @param[in]     offset        timeout offset
 */
void lwmac_set_timeout(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type, uint32_t offset);

/**
 * @brief Clear Lwmac timeout of type @type.
 *
 * @param[in,out] gnrc_netdev2  gnrc_netdev2 structure
 * @param[in]     type          Lwmac timeout type
 */
void lwmac_clear_timeout(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type);

/**
 * @brief Check whether Lwmac timeout of type @type is running.
 *
 * @param[in]     gnrc_netdev2  gnrc_netdev2 structure
 * @param[in]     type          Lwmac timeout type
 */
bool lwmac_timeout_is_running(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type);

/**
 * @brief Check whether Lwmac timeout of type @type is expired. It will clear
 *        the timeout once it is found expired.
 *
 * @param[in,out] gnrc_netdev2  gnrc_netdev2 structure
 * @param[in]     type          Lwmac timeout type
 */
bool lwmac_timeout_is_expired(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type);

/**
 * @brief Reset all Lwmac timeouts.
 *
 * @param[in,out] gnrc_netdev2  gnrc_netdev2 structure
 */
void lwmac_reset_timeouts(gnrc_netdev2_t* gnrc_netdev2);

/**
 * @brief Make a specific Lwmac timeout expired.
 *
 * @param[in,out] timeout   Lwmac tiemout
 */
void lwmac_timeout_make_expire(lwmac_timeout_t* timeout);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_TIMEOUT_H */
/** @} */

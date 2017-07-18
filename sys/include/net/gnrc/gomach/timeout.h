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
 * @brief       Timeout handling of GoMacH.
 * @internal
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef NET_GNRC_GOMACH_GOMACH_TIMEOUT_H
#define NET_GNRC_GOMACH_GOMACH_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>

#include "msg.h"
#include "xtimer.h"
#include "net/gnrc/netdev.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IQUEUEMAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

void gomach_set_timeout(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type, uint32_t offset);

void gomach_clear_timeout(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type);

bool gomach_timeout_is_running(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type);

bool gomach_timeout_is_expired(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type);

void gomach_reset_timeouts(gnrc_netdev_t *netdev);

void gomach_timeout_make_expire(gnrc_gomach_timeout_t *timeout);

static inline void _gomach_clear_timeout(gnrc_gomach_timeout_t *timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = GNRC_GOMACH_TIMEOUT_DISABLED;
}

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_GOMACH_GOMACH_TIMEOUT_H */
/** @} */

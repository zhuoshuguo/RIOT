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
 * @brief       Timeout handling of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <errno.h>
#include "xtimer.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/lwmac/lwmac.h"
#include "net/gnrc/lwmac/timeout.h"
#include "net/gnrc/lwmac/types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if ENABLE_DEBUG
char *lwmac_timeout_names[] = {
    [TIMEOUT_DISABLED]              = "DISABLED",
    [TIMEOUT_WR]                    = "WR",
    [TIMEOUT_NO_RESPONSE]           = "NO_RESPONSE",
    [TIMEOUT_WA]                    = "WA",
    [TIMEOUT_DATA]                  = "DATA",
    [TIMEOUT_WAIT_FOR_DEST_WAKEUP]  = "WAIT_FOR_DEST_WAKEUP",
    [TIMEOUT_WAKEUP_PERIOD]         = "WAKEUP_PERIOD",
};
#endif

static inline void _lwmac_clear_timeout(lwmac_timeout_t *timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/* Return index >= 0 if found, -ENONENT if not found */
static int _lwmac_find_timeout(lwmac_t *lwmac, lwmac_timeout_type_t type)
{
    assert(lwmac);

    for (unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++) {
        if (lwmac->timeouts[i].type == type) {
            return i;
        }
    }
    return -ENOENT;
}

inline bool lwmac_timeout_is_running(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev);
    return (_lwmac_find_timeout(&gnrc_netdev->lwmac, type) >= 0);
}

bool lwmac_timeout_is_expired(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev);

    int index = _lwmac_find_timeout(&gnrc_netdev->lwmac, type);
    if (index >= 0) {
        if (gnrc_netdev->lwmac.timeouts[index].expired) {
            _lwmac_clear_timeout(&gnrc_netdev->lwmac.timeouts[index]);
        }
        return gnrc_netdev->lwmac.timeouts[index].expired;
    }
    return false;
}

lwmac_timeout_t *_lwmac_acquire_timeout(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev);

    if (lwmac_timeout_is_running(gnrc_netdev, type)) {
        return NULL;
    }

    for (unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++) {
        if (gnrc_netdev->lwmac.timeouts[i].type == TIMEOUT_DISABLED) {
            gnrc_netdev->lwmac.timeouts[i].type = type;
            return &gnrc_netdev->lwmac.timeouts[i];
        }
    }
    return NULL;
}

void lwmac_timeout_make_expire(lwmac_timeout_t *timeout)
{
    assert(timeout);

    timeout->expired = true;
}

void lwmac_clear_timeout(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev);

    int index = _lwmac_find_timeout(&gnrc_netdev->lwmac, type);
    if (index >= 0) {
        _lwmac_clear_timeout(&gnrc_netdev->lwmac.timeouts[index]);
    }
}

void lwmac_set_timeout(gnrc_netdev_t *gnrc_netdev, lwmac_timeout_type_t type, uint32_t offset)
{
    assert(gnrc_netdev);

    lwmac_timeout_t *timeout;
    if ((timeout = _lwmac_acquire_timeout(gnrc_netdev, type))) {
        DEBUG("[lwmac] Set timeout %s in %" PRIu32 " us\n",
              lwmac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = LWMAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void *) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), gnrc_netdev->pid);
    }
    else {
        DEBUG("[lwmac] Cannot set timeout %s, too many concurrent timeouts\n",
              lwmac_timeout_names[type]);
    }
}

void lwmac_reset_timeouts(gnrc_netdev_t *gnrc_netdev)
{
    assert(gnrc_netdev);

    for (unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++) {
        if (gnrc_netdev->lwmac.timeouts[i].type != TIMEOUT_DISABLED) {
            _lwmac_clear_timeout(&gnrc_netdev->lwmac.timeouts[i]);
        }
    }
}

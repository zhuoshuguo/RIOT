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
 * @brief       Implementation of GoMacH's timeout module.
 *
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include "xtimer.h"
#include "net/gnrc/gomach/gomach.h"
#include "net/gnrc/gomach/timeout.h"
#include "net/gnrc/gomach/types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

static inline void _iqueuemac_clear_timeout(gnrc_gomach_timeout_t *timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = GNRC_GOMACH_TIMEOUT_DISABLED;
}

/* Return index >= 0 if found, -ENONENT if not found */
static int _iqueuemac_find_timeout(gnrc_gomach_t *gomach, gnrc_gomach_timeout_type_t type)
{
    assert(gomach);

    for (unsigned i = 0; i < GNRC_GOMACH_TIMEOUT_COUNT; i++) {
        if (gomach->timeouts[i].type == type) {
            return i;
        }
    }
    return -ENOENT;
}

inline bool gomach_timeout_is_running(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type)
{
    assert(netdev);
    return (_iqueuemac_find_timeout(&netdev->gomach, type) >= 0);
}

bool gomach_timeout_is_expired(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type)
{
    assert(netdev);

    int index = _iqueuemac_find_timeout(&netdev->gomach, type);
    if (index >= 0) {
        if (netdev->gomach.timeouts[index].expired) {
            _iqueuemac_clear_timeout(&netdev->gomach.timeouts[index]);
        }
        return netdev->gomach.timeouts[index].expired;
    }
    return false;
}

gnrc_gomach_timeout_t *_iqueuemac_acquire_timeout(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type)
{
    assert(netdev);

    if (gomach_timeout_is_running(netdev, type)) {
        return NULL;
    }

    for (unsigned i = 0; i < GNRC_GOMACH_TIMEOUT_COUNT; i++) {
        if (netdev->gomach.timeouts[i].type == GNRC_GOMACH_TIMEOUT_DISABLED) {
        	netdev->gomach.timeouts[i].type = type;
            return &netdev->gomach.timeouts[i];
        }
    }
    return NULL;
}

void gomach_timeout_make_expire(gnrc_gomach_timeout_t *timeout)
{
    assert(timeout);

    DEBUG("[gomach] Timeout %s expired\n", iqueuemac_timeout_names[timeout->type]);
    timeout->expired = true;
}

void gomach_clear_timeout(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type)
{
    assert(netdev);

    int index = _iqueuemac_find_timeout(&netdev->gomach, type);
    if (index >= 0) {
        _iqueuemac_clear_timeout(&netdev->gomach.timeouts[index]);
    }
}

void gomach_set_timeout(gnrc_netdev_t *netdev, gnrc_gomach_timeout_type_t type, uint32_t offset)
{
    assert(netdev);

    gnrc_gomach_timeout_t *timeout;
    if ((timeout = _iqueuemac_acquire_timeout(netdev, type))) {
        DEBUG("[gomach] Set timeout %s in %" PRIu32 " us\n",
              iqueuemac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = GNRC_GOMACH_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void *) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), netdev->pid);
    }
    else {

        DEBUG("[gomach] Cannot set timeout %s, too many concurrent timeouts\n",
              iqueuemac_timeout_names[type]);
    }
}

void gomach_reset_timeouts(gnrc_netdev_t *netdev)
{
    assert(netdev);

    for (unsigned i = 0; i < GNRC_GOMACH_TIMEOUT_COUNT; i++) {
        if (netdev->gomach.timeouts[i].type != GNRC_GOMACH_TIMEOUT_DISABLED) {
            _iqueuemac_clear_timeout(&netdev->gomach.timeouts[i]);
        }
    }
}

/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iqueuemac
 * @file
 * @brief       Timeout handling.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#include <xtimer.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"

#include "net/gnrc/iqueue_mac/timeout.h"
#include "net/gnrc/iqueue_mac/iqueuemac_types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

#if ENABLE_DEBUG
char *iqueuemac_timeout_names[] = {
    [TIMEOUT_DISABLED]              = "DISABLED",
    [TIMEOUT_WR]                    = "WR",
    [TIMEOUT_NO_RESPONSE]           = "NO_RESPONSE",
    [TIMEOUT_WA]                    = "WA",
    [TIMEOUT_DATA]                  = "DATA",
    [TIMEOUT_WAIT_FOR_DEST_WAKEUP]  = "WAIT_FOR_DEST_WAKEUP",
    [TIMEOUT_WAKEUP_PERIOD]         = "WAKEUP_PERIOD",
};
#endif

/******************************************************************************/

static inline void _iqueuemac_clear_timeout(gomach_timeout_t *timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/******************************************************************************/

/* Return index >= 0 if found, -ENONENT if not found */
static int _iqueuemac_find_timeout(gomach_t *gomach, gomach_timeout_type_t type)
{
    assert(gomach);

    for (unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++) {
        if (gomach->timeouts[i].type == type) {
            return i;
        }
    }
    return -ENOENT;
}

/******************************************************************************/

inline bool iqueuemac_timeout_is_running(gomach_t *gomach, gomach_timeout_type_t type)
{
    assert(gomach);
    return (_iqueuemac_find_timeout(gomach, type) >= 0);
}

/******************************************************************************/

bool iqueuemac_timeout_is_expired(gomach_t *gomach, gomach_timeout_type_t type)
{
    assert(gomach);

    int index = _iqueuemac_find_timeout(gomach, type);
    if (index >= 0) {
        if (gomach->timeouts[index].expired) {
            _iqueuemac_clear_timeout(&gomach->timeouts[index]);
        }
        return gomach->timeouts[index].expired;
    }
    return false;
}

/******************************************************************************/

gomach_timeout_t *_iqueuemac_acquire_timeout(gomach_t *gomach, gomach_timeout_type_t type)
{
    assert(gomach);

    if (iqueuemac_timeout_is_running(gomach, type)) {
        return NULL;
    }

    for (unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++) {
        if (gomach->timeouts[i].type == TIMEOUT_DISABLED) {
            gomach->timeouts[i].type = type;
            return &gomach->timeouts[i];
        }
    }
    return NULL;
}

/******************************************************************************/

void iqueuemac_timeout_make_expire(gomach_timeout_t *timeout)
{
    assert(timeout);

    DEBUG("[gomach] Timeout %s expired\n", iqueuemac_timeout_names[timeout->type]);
    timeout->expired = true;
}

/******************************************************************************/

void iqueuemac_clear_timeout(gomach_t *gomach, gomach_timeout_type_t type)
{
    assert(gomach);

    int index = _iqueuemac_find_timeout(gomach, type);
    if (index >= 0) {
        _iqueuemac_clear_timeout(&gomach->timeouts[index]);
    }
}

/******************************************************************************/

void iqueuemac_set_timeout(gomach_t *gomach, gomach_timeout_type_t type, uint32_t offset)
{
    assert(gomach);

    gomach_timeout_t *timeout;
    if ((timeout = _iqueuemac_acquire_timeout(gomach, type))) {
        DEBUG("[gomach] Set timeout %s in %" PRIu32 " us\n",
              iqueuemac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = IQUEUEMAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void *) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), gomach->pid);
    }
    else {

        DEBUG("[gomach] Cannot set timeout %s, too many concurrent timeouts\n",
              iqueuemac_timeout_names[type]);
    }
}

/******************************************************************************/

void iqueuemac_reset_timeouts(gomach_t *gomach)
{
    assert(gomach);

    for (unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++) {
        if (gomach->timeouts[i].type != TIMEOUT_DISABLED) {
            _iqueuemac_clear_timeout(&gomach->timeouts[i]);
        }
    }
}

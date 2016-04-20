/*
 * Copyright (C) 2015 Daniel Krebs
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
 * @}
 */

#include <xtimer.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"

#include "include/timeout.h"
#include "include/iqueuemac_types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

#if ENABLE_DEBUG
char* iqueuemac_timeout_names[] = {
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

static inline void _iqueuemac_clear_timeout(iqueuemac_timeout_t* timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/******************************************************************************/

/* Return index >= 0 if found, -ENONENT if not found */
static int _iqueuemac_find_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type)
{
    assert(iqueuemac);

    for(unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++)
    {
        if(iqueuemac->timeouts[i].type == type)
            return i;
    }
    return -ENOENT;
}

/******************************************************************************/

inline bool iqueuemac_timeout_is_running(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type)
{
    assert(iqueuemac);
    return (_iqueuemac_find_timeout(iqueuemac, type) >= 0);
}

/******************************************************************************/

bool iqueuemac_timeout_is_expired(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type)
{
    assert(iqueuemac);

    int index = _iqueuemac_find_timeout(iqueuemac, type);
    if(index >= 0) {
        if(iqueuemac->timeouts[index].expired)
            _iqueuemac_clear_timeout(&iqueuemac->timeouts[index]);
        return iqueuemac->timeouts[index].expired;
    }
    return false;
}

/******************************************************************************/

iqueuemac_timeout_t* _iqueuemac_acquire_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type)
{
    assert(iqueuemac);

    if(iqueuemac_timeout_is_running(iqueuemac, type))
        return NULL;

    for(unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++)
    {
        if(iqueuemac->timeouts[i].type == TIMEOUT_DISABLED)
        {
            iqueuemac->timeouts[i].type = type;
            return &iqueuemac->timeouts[i];
        }
    }
    return NULL;
}

/******************************************************************************/

void iqueuemac_timeout_make_expire(iqueuemac_timeout_t* timeout)
{
    assert(timeout);

    DEBUG("[iqueuemac] Timeout %s expired\n", iqueuemac_timeout_names[timeout->type]);
    timeout->expired = true;
}

/******************************************************************************/

void iqueuemac_clear_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type)
{
    assert(iqueuemac);

    int index = _iqueuemac_find_timeout(iqueuemac, type);
    if(index >= 0)
        _iqueuemac_clear_timeout(&iqueuemac->timeouts[index]);
}

/******************************************************************************/

void iqueuemac_set_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type, uint32_t offset)
{
    assert(iqueuemac);

    iqueuemac_timeout_t* timeout;
    if( (timeout = _iqueuemac_acquire_timeout(iqueuemac, type)) )
    {
        DEBUG("[iqueuemac] Set timeout %s in %"PRIu32" us\n",
                iqueuemac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = IQUEUEMAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void*) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), iqueuemac->pid);
    } else {
        DEBUG("[iqueuemac] Cannot set timeout %s, too many concurrent timeouts\n",
                iqueuemac_timeout_names[type]);
    }
}

/******************************************************************************/

void iqueuemac_reset_timeouts(iqueuemac_t* iqueuemac)
{
    assert(iqueuemac);

    for(unsigned i = 0; i < IQUEUEMAC_TIMEOUT_COUNT; i++)
    {
        if(iqueuemac->timeouts[i].type != TIMEOUT_DISABLED)
            _iqueuemac_clear_timeout(&iqueuemac->timeouts[i]);
    }
}

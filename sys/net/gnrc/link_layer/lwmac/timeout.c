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

#include <xtimer.h>
#include <net/gnrc/lwmac/lwmac.h>
#include <net/gnrc/lwmac/timeout.h>
#include <net/gnrc/lwmac/types.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if ENABLE_DEBUG
char* lwmac_timeout_names[] = {
    [TIMEOUT_DISABLED]              = "DISABLED",
    [TIMEOUT_WR]                    = "WR",
    [TIMEOUT_NO_RESPONSE]           = "NO_RESPONSE",
    [TIMEOUT_WA]                    = "WA",
    [TIMEOUT_DATA]                  = "DATA",
    [TIMEOUT_WAIT_FOR_DEST_WAKEUP]  = "WAIT_FOR_DEST_WAKEUP",
    [TIMEOUT_WAKEUP_PERIOD]         = "WAKEUP_PERIOD",
};
#endif

static inline void _lwmac_clear_timeout(lwmac_timeout_t* timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/* Return index >= 0 if found, -ENONENT if not found */
static int _lwmac_find_timeout(lwmac_t* lwmac, lwmac_timeout_type_t type)
{
    assert(lwmac);

    for (unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if (lwmac->timeouts[i].type == type) {
            return i;
        }
    }
    return -ENOENT;
}

inline bool lwmac_timeout_is_running(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev2);
    return (_lwmac_find_timeout(&gnrc_netdev2->lwmac, type) >= 0);
}

bool lwmac_timeout_is_expired(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev2);

    int index = _lwmac_find_timeout(&gnrc_netdev2->lwmac, type);
    if (index >= 0) {
        if (gnrc_netdev2->lwmac.timeouts[index].expired) {
            _lwmac_clear_timeout(&gnrc_netdev2->lwmac.timeouts[index]);
        }
        return gnrc_netdev2->lwmac.timeouts[index].expired;
    }
    return false;
}

lwmac_timeout_t* _lwmac_acquire_timeout(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev2);

    if (lwmac_timeout_is_running(gnrc_netdev2, type)) {
        return NULL;
    }

    for(unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if(gnrc_netdev2->lwmac.timeouts[i].type == TIMEOUT_DISABLED)
        {
        	gnrc_netdev2->lwmac.timeouts[i].type = type;
            return &gnrc_netdev2->lwmac.timeouts[i];
        }
    }
    return NULL;
}

void lwmac_timeout_make_expire(lwmac_timeout_t* timeout)
{
    assert(timeout);

    timeout->expired = true;
}

void lwmac_clear_timeout(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type)
{
    assert(gnrc_netdev2);

    int index = _lwmac_find_timeout(&gnrc_netdev2->lwmac, type);
    if (index >= 0) {
        _lwmac_clear_timeout(&gnrc_netdev2->lwmac.timeouts[index]);
    }
}

void lwmac_set_timeout(gnrc_netdev2_t* gnrc_netdev2, lwmac_timeout_type_t type, uint32_t offset)
{
    assert(gnrc_netdev2);

    lwmac_timeout_t* timeout;
    if ((timeout = _lwmac_acquire_timeout(gnrc_netdev2, type)))
    {
        DEBUG("[lwmac] Set timeout %s in %"PRIu32" us\n",
              lwmac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = LWMAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void*) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), gnrc_netdev2->pid);
    } else {
        DEBUG("[lwmac] Cannot set timeout %s, too many concurrent timeouts\n",
              lwmac_timeout_names[type]);
    }
}

void lwmac_reset_timeouts(gnrc_netdev2_t* gnrc_netdev2)
{
    assert(gnrc_netdev2);

    for (unsigned i = 0; i < LWMAC_TIMEOUT_COUNT; i++)
    {
        if (gnrc_netdev2->lwmac.timeouts[i].type != TIMEOUT_DISABLED) {
            _lwmac_clear_timeout(&gnrc_netdev2->lwmac.timeouts[i]);
        }
    }
}

/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_gnrc_mac
 * @file
 * @brief       Timeout handling.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#include <xtimer.h>
#include <net/gnrc/gnrc_mac_type/gnrc_mac.h>

#include <net/gnrc/gnrc_mac_type/timeout.h>
#include <net/gnrc/gnrc_mac_type/gnrc_mac_types.h>

#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

#if ENABLE_DEBUG
char* gnrc_mac_timeout_names[] = {
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

static inline void _gnrc_mac_clear_timeout(gnrc_mac_timeout_t* timeout)
{
    assert(timeout);

    xtimer_remove(&(timeout->timer));
    timeout->type = TIMEOUT_DISABLED;
}

/******************************************************************************/

/* Return index >= 0 if found, -ENONENT if not found */
static int _gnrc_mac_find_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type)
{
    assert(gnrc_mac);

    for(unsigned i = 0; i < GNRC_MAC_TIMEOUT_COUNT; i++)
    {
        if(gnrc_mac->timeouts[i].type == type)
            return i;
    }
    return -ENOENT;
}

/******************************************************************************/

inline bool gnrc_mac_timeout_is_running(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type)
{
    assert(gnrc_mac);
    return (_gnrc_mac_find_timeout(gnrc_mac, type) >= 0);
}

/******************************************************************************/

bool gnrc_mac_timeout_is_expired(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type)
{
    assert(gnrc_mac);

    int index = _gnrc_mac_find_timeout(gnrc_mac, type);
    if(index >= 0) {
        if(gnrc_mac->timeouts[index].expired)
            _gnrc_mac_clear_timeout(&gnrc_mac->timeouts[index]);
        return gnrc_mac->timeouts[index].expired;
    }
    return false;
}

/******************************************************************************/

gnrc_mac_timeout_t* _gnrc_mac_acquire_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type)
{
    assert(gnrc_mac);

    if(gnrc_mac_timeout_is_running(gnrc_mac, type))
        return NULL;

    for(unsigned i = 0; i < GNRC_MAC_TIMEOUT_COUNT; i++)
    {
        if(gnrc_mac->timeouts[i].type == TIMEOUT_DISABLED)
        {
            gnrc_mac->timeouts[i].type = type;
            return &gnrc_mac->timeouts[i];
        }
    }
    return NULL;
}

/******************************************************************************/

void gnrc_mac_timeout_make_expire(gnrc_mac_timeout_t* timeout)
{
    assert(timeout);

    DEBUG("[gnrc_mac] Timeout %s expired\n", gnrc_mac_timeout_names[timeout->type]);
    timeout->expired = true;
}

/******************************************************************************/

void gnrc_mac_clear_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type)
{
    assert(gnrc_mac);

    int index = _gnrc_mac_find_timeout(gnrc_mac, type);
    if(index >= 0)
        _gnrc_mac_clear_timeout(&gnrc_mac->timeouts[index]);
}

/******************************************************************************/

void gnrc_mac_set_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type, uint32_t offset)
{
    assert(gnrc_mac);

    gnrc_mac_timeout_t* timeout;
    if( (timeout = _gnrc_mac_acquire_timeout(gnrc_mac, type)) )
    {
        DEBUG("[gnrc_mac] Set timeout %s in %"PRIu32" us\n",
                gnrc_mac_timeout_names[type], offset);
        timeout->expired = false;
        timeout->msg.type = GNRC_MAC_EVENT_TIMEOUT_TYPE;
        timeout->msg.content.ptr = (void*) timeout;
        xtimer_set_msg(&(timeout->timer), offset,
                       &(timeout->msg), gnrc_mac->pid);
    } else {
        DEBUG("[gnrc_mac] Cannot set timeout %s, too many concurrent timeouts\n",
                gnrc_mac_timeout_names[type]);
    }
}

/******************************************************************************/

void gnrc_mac_reset_timeouts(gnrc_mac_t* gnrc_mac)
{
    assert(gnrc_mac);

    for(unsigned i = 0; i < GNRC_MAC_TIMEOUT_COUNT; i++)
    {
        if(gnrc_mac->timeouts[i].type != TIMEOUT_DISABLED)
            _gnrc_mac_clear_timeout(&gnrc_mac->timeouts[i]);
    }
}

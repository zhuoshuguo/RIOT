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

#ifndef GNRC_MAC_TIMEOUT_H
#define GNRC_MAC_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>
#include <msg.h>
#include <xtimer.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Foward declaration */
typedef struct gnrc_mac gnrc_mac_t;

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
} gnrc_mac_timeout_type_t;

typedef struct {
    xtimer_t timer;
    msg_t msg;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    gnrc_mac_timeout_type_t type;
} gnrc_mac_timeout_t;
#define GNRC_MAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

void gnrc_mac_set_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type, uint32_t offset);

void gnrc_mac_clear_timeout(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type);

bool gnrc_mac_timeout_is_running(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type);

bool gnrc_mac_timeout_is_expired(gnrc_mac_t* gnrc_mac, gnrc_mac_timeout_type_t type);

void gnrc_mac_reset_timeouts(gnrc_mac_t* gnrc_mac);

void gnrc_mac_timeout_make_expire(gnrc_mac_timeout_t* timeout);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_TIMEOUT_H */

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
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 * @}
 */

#ifndef IQUEUEMAC_TIMEOUT_H
#define IQUEUEMAC_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>
#include <msg.h>
#include <xtimer.h>

//#include "include/iqueue_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Foward declaration */
typedef struct iqueuemac iqueuemac_t;


typedef enum {
    TIMEOUT_DISABLED = 0,

	TIMEOUT_BROADCAST_FINISH,
	TIMEOUT_BROADCAST_INTERVAL,
	TIMEOUT_PREAMBLE,
	TIMEOUT_PREAMBLE_DURATION,
	TIMEOUT_WAIT_CP,
	TIMEOUT_WAIT_BEACON,
	TIMEOUT_WAIT_OWN_SLOTS,
	/*****************router******************/
	TIMEOUT_COLLECT_BEACON_END,
    TIMEOUT_CP_END,
    TIMEOUT_CP_MAX,
    TIMEOUT_VTDMA,
	/*****************simple-node******************/
	TIMEOUT_N_CP_DURATION,
	TIMEOUT_BEACON_END

} iqueuemac_timeout_type_t;

typedef struct {
    xtimer_t timer;
    msg_t msg;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    iqueuemac_timeout_type_t type;
} iqueuemac_timeout_t;
#define IQUEUEMAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

void iqueuemac_set_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type, uint32_t offset);

void iqueuemac_clear_timeout(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type);

bool iqueuemac_timeout_is_running(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type);

bool iqueuemac_timeout_is_expired(iqueuemac_t* iqueuemac, iqueuemac_timeout_type_t type);

void iqueuemac_reset_timeouts(iqueuemac_t* iqueuemac);

void iqueuemac_timeout_make_expire(iqueuemac_timeout_t* timeout);

#ifdef __cplusplus
}
#endif

#endif /* IQUEUEMAC_TIMEOUT_H */

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

#ifndef IQUEUEMAC_TIMEOUT_H
#define IQUEUEMAC_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>

#include "msg.h"
#include "xtimer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Foward declaration */
typedef struct gomach gomach_t;


typedef enum {
    TIMEOUT_DISABLED = 0,

    TIMEOUT_BROADCAST_FINISH,
    TIMEOUT_BROADCAST_INTERVAL,
    TIMEOUT_PREAMBLE,
    TIMEOUT_MAX_PREAM_INTERVAL,
    TIMEOUT_PREAMBLE_DURATION,
    TIMEOUT_WAIT_CP,
    TIMEOUT_WAIT_BEACON,
    TIMEOUT_WAIT_OWN_SLOTS,
    TIMEOUT_WAIT_RE_PHASE_LOCK,
    /*****************router******************/
    TIMEOUT_CP_END,
    TIMEOUT_CP_MAX,
    TIMEOUT_WAIT_RX_END,
    TIMEOUT_VTDMA,
    /*****************simple-node******************/
    TIMEOUT_N_CP_DURATION,
    TIMEOUT_BEACON_END

} gomach_timeout_type_t;

typedef struct {
    xtimer_t timer;
    msg_t msg;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    gomach_timeout_type_t type;
} gomach_timeout_t;

#define IQUEUEMAC_TIMEOUT_INIT  { {}, {}, false, TIMEOUT_DISABLED }

void iqueuemac_set_timeout(gomach_t *iqueuemac, gomach_timeout_type_t type, uint32_t offset);

void iqueuemac_clear_timeout(gomach_t *iqueuemac, gomach_timeout_type_t type);

bool iqueuemac_timeout_is_running(gomach_t *iqueuemac, gomach_timeout_type_t type);

bool iqueuemac_timeout_is_expired(gomach_t *iqueuemac, gomach_timeout_type_t type);

void iqueuemac_reset_timeouts(gomach_t *iqueuemac);

void iqueuemac_timeout_make_expire(gomach_timeout_t *timeout);

#ifdef __cplusplus
}
#endif

#endif /* IQUEUEMAC_TIMEOUT_H */
/** @} */

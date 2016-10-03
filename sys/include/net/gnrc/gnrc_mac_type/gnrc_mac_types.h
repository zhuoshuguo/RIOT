/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Internal types of LWMAC
 * @{
 *
 * @file
 * @brief       Internal types used by the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

#ifndef GNRC_MAC_TYPES_H_
#define GNRC_MAC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <kernel_types.h>
#include <xtimer.h>
#include <net/netdev2.h>
#include <net/gnrc/netdev2.h>
#include <net/gnrc.h>
//#include <net/gnrc/gnrc_mac_type/gnrc_mac.h>
//#include <net/gnrc/gnrc_mac_type/packet_queue.h>
#include "gnrc_mac.h"
#include "packet_queue.h"
#include "timeout.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************/

#define GNRC_MAC_EVENT_TIMEOUT_TYPE        (0x4400)


/******************************************************************************/
typedef struct {
    uint8_t  addr[GNRC_MAC_MAX_L2_ADDR_LEN] __attribute__ ((aligned (GNRC_MAC_MAX_L2_ADDR_LEN)));
    uint8_t  len;
} l2_addr_t;
#define GNRC_MAC_L2_ADDR_INIT      { {0}, 0 }

/******************************************************************************/

typedef enum {
    TX_FEEDBACK_UNDEF = -1,
    TX_FEEDBACK_SUCCESS,
    TX_FEEDBACK_NOACK,
    TX_FEEDBACK_BUSY
} gnrc_mac_tx_feedback_t;
#define LWMAC_TX_FEEDBACK_INIT TX_FEEDBACK_UNDEF

/******************************************************************************/
typedef struct gnrc_mac {
    /* PID of lwMAC thread */
    kernel_pid_t pid;
    /* NETDEV device used by lwMAC */
    gnrc_netdev2_t* netdev;
	const netdev2_driver_t* netdev2_driver;
	
    /* Store timeouts used for protocol */
    gnrc_mac_timeout_t timeouts[GNRC_MAC_TIMEOUT_COUNT];
    
    /* Own address */
    l2_addr_t l2_addr;   
    
    /* Track if a transmission might have corrupted a received packet */
    bool rx_started;

    /* Feedback of last packet that was sent */
    gnrc_mac_tx_feedback_t tx_feedback;

} gnrc_mac_t;

#if 0
#define GNRC_MAC_INIT { \
/* pid */                   KERNEL_PID_UNDEF,  \
/* netdev */                NULL, \
/* netdev2_driver */		NULL, \
/* state */                 UNDEF, \
/* rx_in_progress */        false, \
/* l2_addr */               LWMAC_L2_ADDR_INIT, \
/* rx */                    LWMAC_RX_INIT, \
/* tx */                    LWMAC_TX_INIT, \
/* tx_feedback */           LWMAC_TX_FEEDBACK_INIT, \
/* timeouts */              { LWMAC_TIMEOUT_INIT }, \
/* last_wakeup */           0, \
/* dutycycling_active */    false, \
/* needs_rescheduling */    false \
}
#endif



#ifdef __cplusplus
}
#endif

#endif /* GNRC_MAC_TYPES_H_ */
/** @} */

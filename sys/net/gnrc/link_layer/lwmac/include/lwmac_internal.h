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
 * @brief       Internal functions of LWMAC
 * @{
 *
 * @file
 * @brief       Interface definition for internal functions of LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 */

#ifndef GNRC_LWMAC_INTERNAL_H_
#define GNRC_LWMAC_INTERNAL_H_

#include <stdint.h>
#include "periph/rtt.h"
#include "net/gnrc/mac/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* @brief   Type to pass information about parsing */
typedef struct {
    lwmac_hdr_t* header;    /**< lwmac header of packet */
    l2_addr_t  src_addr;    /**< copied source address of packet  */
    l2_addr_t  dst_addr;    /**< copied destination address of packet */
} lwmac_packet_info_t;

/* @brief   Next RTT event must be at least this far in the future
 *
 * When setting an RTT alarm to short in the future it could be possible that
 * the counter already passed the calculated alarm before it could be set. This
 * margin will be applied when using `_next_inphase_event()`.
 */
#define LWMAC_RTT_EVENT_MARGIN_TICKS    ( RTT_MS_TO_TICKS(2) )

/* @brief Parse an incoming packet and extract important information
 *
 * Copies addresses into @p info, but header points inside @p pkt.
 *
 * @param[in]   pkt             packet that will be parsed
 * @param[out]  info            structure that will hold parsed information
 *
 * @return                      0 if correctly parsed
 * @return                      <0 on error
 */
int _parse_packet(gnrc_pktsnip_t* pkt, lwmac_packet_info_t* info);


/* @brief Shortcut to get the state of netdev
 *
 * @param[in]   lwmac           lwmac state that stores netdev pointer
 *
 * @return                      state of netdev
 */
netopt_state_t _get_netdev_state(gnrc_netdev2_t* gnrc_netdev2);

/* @brief Shortcut to set the state of netdev
 *
 * @param[in]   lwmac           lwmac state that stores netdev pointer
 * @param[in]   devstate        new state for netdev
 */
void _set_netdev_state(gnrc_netdev2_t* gnrc_netdev2, netopt_state_t devstate);

/* RTT phase calculation */
static inline uint32_t _ticks_to_phase(uint32_t ticks)
{
    return (ticks % RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US));
}

static inline uint32_t _phase_now(void)
{
    return _ticks_to_phase(rtt_get_counter());
}

static inline uint32_t _ticks_until_phase(uint32_t phase)
{
    long int tmp = phase - _phase_now();
    if(tmp < 0) {
        /* Phase in next interval */
        tmp += RTT_US_TO_TICKS(LWMAC_WAKEUP_INTERVAL_US);
    }

    return (uint32_t)tmp;
}

uint32_t _phase_to_ticks(uint32_t phase);

gnrc_mac_tx_neighbor_t* _next_tx_neighbour(gnrc_netdev2_t* gnrc_netdev2);
int _time_until_tx_us(gnrc_netdev2_t* gnrc_netdev2);
uint32_t _next_inphase_event(uint32_t last, uint32_t interval);

int _dispatch_defer(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt);

void _dispatch(gnrc_pktsnip_t* buffer[]);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_LWMAC_INTERNAL_H_ */
/** @} */

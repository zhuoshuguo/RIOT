/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_lwmac
 * @file
 * @brief       Implementation of TX state machine
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @}
 */

#ifndef LWMAC_TX_STATE_MACHINE_H_
#define LWMAC_TX_STATE_MACHINE_H_

#include "net/gnrc/pkt.h"
#include <net/gnrc/netdev2.h>
#include "net/gnrc/mac/types.h"

#ifdef __cplusplus
extern "C" {
#endif

void lwmac_tx_start(gnrc_netdev2_t* gnrc_netdev2, gnrc_pktsnip_t* pkt, gnrc_mac_tx_neighbor_t* neighbour);

void lwmac_tx_stop(gnrc_netdev2_t* gnrc_netdev2);

void lwmac_tx_update(gnrc_netdev2_t* gnrc_netdev2);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_TX_STATE_MACHINE_H_ */

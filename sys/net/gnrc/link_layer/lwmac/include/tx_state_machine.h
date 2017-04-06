/*
 * Copyright (C) 2015 Daniel Krebs
 *               2016 INRIA
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
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#ifndef LWMAC_TX_STATE_MACHINE_H_
#define LWMAC_TX_STATE_MACHINE_H_

#include "net/gnrc/pkt.h"
#include "net/gnrc/netdev2.h"
#include "net/gnrc/mac/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start Lwmac TX procedure to transmit packet @p pkt to @p neighbor
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 * @param[in]       pkt            packet to transmit
 * @param[in]       neighbor       Tx neighbor
 *
 */
void lwmac_tx_start(gnrc_netdev2_t* gnrc_netdev2, gnrc_pktsnip_t* pkt, gnrc_mac_tx_neighbor_t* neighbor);

/**
 * @brief Stop Lwmac TX procedure
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 *
 */
void lwmac_tx_stop(gnrc_netdev2_t* gnrc_netdev2);

/**
 * @brief Update Lwmac TX procedure for transmission
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 *
 */
void lwmac_tx_update(gnrc_netdev2_t* gnrc_netdev2);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_TX_STATE_MACHINE_H_ */

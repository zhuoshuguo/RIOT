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
 * @brief       Implementation of RX state machine
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */

#ifndef LWMAC_RX_STATE_MACHINE_H_
#define LWMAC_RX_STATE_MACHINE_H_

#include "net/gnrc/pkt.h"
#include "net/gnrc/netdev2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start Lwmac RX procedure to receive packet
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 *
 */
void lwmac_rx_start(gnrc_netdev2_t* gnrc_netdev2);

/**
 * @brief Stop Lwmac RX procedure
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 *
 */
void lwmac_rx_stop(gnrc_netdev2_t* gnrc_netdev2);

/**
 * @brief Update Lwmac RX procedure for packet reception
 *
 * @param[in,out]   gnrc_netdev2   gnrc_netdev2 structure
 *
 */
void lwmac_rx_update(gnrc_netdev2_t* gnrc_netdev2);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_RX_STATE_MACHINE_H_ */

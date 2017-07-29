/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_gnrc_gomach A traffic-adaptive multi-channel MAC
 * @ingroup     net
 * @brief       A traffic adaptive MAC protocol that provides high traffic adaptability,
 *              high energy efficiency and high robustness.
 * @{
 *
 * @file
 * @brief       Implementation of GoMacH protocol
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 */

#ifndef NET_GNRC_GOMACH_GOMACH_H
#define NET_GNRC_GOMACH_GOMACH_H

#include "kernel_types.h"
#include "net/gnrc/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GoMacH's superframe duration, i.e., time between consecutive wake-ups.
 *
 * This macro governs power consumption and GoMacH's reactiveness to traffic loads.
 * In GoMacH, devices adopt duty-cycle scheme to conserve power. That is,
 * time is divided into repeated cycles (superframes), and in each
 * cycle, a node only wakes up for a period of time for receiving potential
 * incoming packets for itself. This macro defines the wake-up interval, or,
 * in other words, defines the cycle duration used in GoMacH. If the wake-up interval
 * is short, nodes will wake up more frequently, which leads to quicker
 * reactiveness of the MAC protocol for handling packet reception and transmission,
 * but also results in higher power consumption due to more idle listening.
 * In GoMacH, by default, we regard the wake-up period (WP) as the beginning of a cycle.
 */
#ifndef GNRC_GOMACH_SUPERFRAME_DURATION_US
#define GNRC_GOMACH_SUPERFRAME_DURATION_US        (200U * 1000)
#endif

/**
 * @brief The default duration of GoMacH's wake-up period (WP).
 *
 * GoMacH adopts the duty-cycle scheme that, by default, a node only wakes up for a short
 * period of @ref GNRC_GOMACH_CP_DURATION_US in each cycle. In the rest of the cycle (except vTDMA),
 * the node turns off the radio to conserve power.
 * @ref GNRC_GOMACH_CP_DURATION_US should be at least longer than 6ms, thus to guarantee that that
 * receiver will not miss the preamble packet.
 */
#ifndef GNRC_GOMACH_CP_DURATION_US
#define GNRC_GOMACH_CP_DURATION_US        (10U * 1000)
#endif

/**
 * @brief The maximum duration of the random period at the end of GoMacH's wake-up period (WP).
 *
 * Currently, GoMacH's WP is actually composed of @ref GNRC_GOMACH_CP_DURATION_US and (+)
 * @ref GNRC_GOMACH_CP_RANDOM_END_US. We currently introduced this random period to avoid beacon
 * collision between neighbor nodes. This macro maybe removed in the future.
 */
#ifndef GNRC_GOMACH_CP_RANDOM_END_US
#define GNRC_GOMACH_CP_RANDOM_END_US        (1U * 1000)
#endif

/**
 * @brief The maximum duration of GoMacH's wake-up period (WP).
 *
 * @ref GNRC_GOMACH_CP_DURATION_MAX_US defines the maximum allowed duration
 * of GoMacH's WP period. A node will quit WP once it reaches this maximum
 * duration.
 * Note that, in GoMacH's WP, after each normal packet reception (except broadcast packet),
 * a receiver will automatically extends the WP period (reset WP timeout), to receiver more
 * potential incoming packets, before WP reaches this @ref GNRC_GOMACH_CP_DURATION_MAX_US
 * duration.
 */
#ifndef GNRC_GOMACH_CP_DURATION_MAX_US
#define GNRC_GOMACH_CP_DURATION_MAX_US        (5 * GNRC_GOMACH_CP_DURATION_US)
#endif

/**
 * @brief The maximum time for waiting the receiver's beacon in GoMacH.
 *
 * After transmissions in the WP, if the sender still has pending packets for the receiver
 * it will wait for the receiver's incoming beacon that alllocates dynamic transmission slots
 * to it. @ref GNRC_GOMACH_WAIT_BEACON_TIME_US defines the maximum waiting time for the beacon.
 * Once the beacon-waiting timeout expires, the sender will quit the vTMDA (slotted transmission),
 * and restarts transmissions (started with normal CSMA attempts in the receiver's WP) in the
 * next cycle.
 */
#ifndef GNRC_GOMACH_WAIT_BEACON_TIME_US
#define GNRC_GOMACH_WAIT_BEACON_TIME_US        (GNRC_GOMACH_CP_DURATION_MAX_US)
#endif

/**
 * @brief The minimum gap between neighbor nodes' wake-up phases in GoMacH.
 *
 * To reduce beacon collisions and transmission collisions, GoMacH intends to avoid neighbor
 * nodes' phases being close to each other. This macro defines the minimum gap between two nodes's
 * wake-up phases. If the sender finds its wake-up phase closed to its receiver's, it will randomly
 * select a new phase for itself.
 */
#ifndef GNRC_GOMACH_CP_MIN_GAP_US
#define GNRC_GOMACH_CP_MIN_GAP_US        (25U * 1000)
#endif

/**
 * @brief Timeout duration for waiting RX complete in GoMacH.
 *
 * Sometimes in GoMacH, if a node find RX is going on when going to the next stage,
 * a node will set up a timeout for waiting packet reception complete with this
 * @ref GNRC_GOMACH_WAIT_RX_END_US duration.
 */
#ifndef GNRC_GOMACH_WAIT_RX_END_US
#define GNRC_GOMACH_WAIT_RX_END_US        (3U * 1000)
#endif

/**
 * @brief Maximum time interval between two consecutive preamble packets in GoMacH.
 *
 * In GoMacH, a sender first uses preamble stream to track the receiver's wake-up phase (WP),
 * if the receiver's WP is unknown. This macro defines the maximum time interval between two
 * consecutive preamble packets.
 */
#ifndef GNRC_GOMACH_MAX_PREAM_INTERVAL_US
#define GNRC_GOMACH_MAX_PREAM_INTERVAL_US        (5U * 1000)
#endif

/**
 * @brief Time interval between two consecutive preamble packets in GoMacH.
 *
 * In GoMacH, after a preamble is sent, the sender sets a timeout with
 * @ref GNRC_GOMACH_PREAMBLE_INTERVAL_US duration for waiting to send the next
 * preamble. Notably, this macro is with a very small value. In GoMacH, for receiving
 * the preamble-ACK packet, the sender doesn't not wait for the whole reception of
 * the preamble-ACK. Instead, it only waits for the RX-start event which leads to shorter
 * time interval between two consecutive preamble transmissions. The shorter the preamble
 * interval is, the shorter the WP period can be, thus leading to lower power consumption.
 */
#ifndef GNRC_GOMACH_PREAMBLE_INTERVAL_US
#define GNRC_GOMACH_PREAMBLE_INTERVAL_US        (1U * 1000)
#endif

/**
 * @brief Time interval between two consecutive broadcast packets in GoMacH.
 *
 * In GoMacH, when sending a broadcast packet, the sender broadcast the same packet
 * on its two public channels simultaneously, with a total duration of
 * @ref GNRC_GOMACH_SUPERFRAME_DURATION_US. This macro defines the time interval
 * between sending two consecutive broadcast copies.
 */
#ifndef GNRC_GOMACH_BCAST_INTERVAL_US
#define GNRC_GOMACH_BCAST_INTERVAL_US        (1U * 1000)
#endif

/**
 * @brief The Maximum preamble duration time of GoMacH.
 *
 * Since GoMacH adopts duty-cycle scheme, to probe receiver's wake-up period
 * a sender sends preamble streams to notice the receiver for communication.
 * To ensure that the receiver will catch at least one preamble packet
 * in one cycle in the worst case (i.e., one public channel is jammed),
 * the sender repeatedly broadcasts a stream of preamble packets with the
 * broadcast duration (preamble duration) slightly longer period than twice
 * of @ref GNRC_GOMACH_SUPERFRAME_DURATION_US.
 */
#ifndef GNRC_GOMACH_PREAMBLE_DURATION_US
#define GNRC_GOMACH_PREAMBLE_DURATION_US        (21 * GNRC_GOMACH_SUPERFRAME_DURATION_US / 10)
#endif

/**
 * @brief The transmission slot size in GoMacH.
 *
 * GoMacH adopts dynamic slots allocation scheme to allocate transmission
 * slots to senders that have pending packets. Each slot is for one data packet
 * with ACK transmission. @ref GNRC_GOMACH_VTDMA_SLOT_SIZE_US is right sufficient
 * for the transmission of the longest packet in IEEE 802.15.4 with ACK.
 */
#ifndef GNRC_GOMACH_VTDMA_SLOT_SIZE_US
#define GNRC_GOMACH_VTDMA_SLOT_SIZE_US        (5U * 1000)
#endif

/**
 * @brief Maximum times of CSMA TX attempts under busy-indication in WP period of GoMacH.
 *
 * Senders in GoMacH adopt CSMA scheme to send data packet in the WP period of the receiver.
 * In case of having medium-busy feedback in WP and the failure count (due to busy) is below
 * @ref GNRC_GOMACH_TX_BUSY_THRESHOLD, the sender will not quit its transmission attempt
 * in the receiver's WP, with the consideration/assumption that there are multi-senders
 * simultaneously competing in WP and WP will get continuously extended.
 */
#ifndef GNRC_GOMACH_TX_BUSY_THRESHOLD
#define GNRC_GOMACH_TX_BUSY_THRESHOLD      (5U)
#endif

/**
 * @brief Maximum WP period extension time in GoMacH.
 *
 * In GoMacH, the WP period of a receiver will get extended upon each successful packet
 * reception (except receiving broadcast or preamble packet) to receive more potential
 * incoming packets. This macro defines the maximum WP period extension time in GoMacH.
 */
#ifndef GNRC_GOMACH_CP_EXTEND_THRESHOLD
#define GNRC_GOMACH_CP_EXTEND_THRESHOLD      (5U)
#endif

/**
 * @brief GoMacH's check-duplicate-packet unit life time in cycle count.
 *
 * In GoMacH, to avoid receiving duplicate-packet, we currently introduce a data type of
 * @ref gnrc_gomach_dupchk_unit_t to record the receiver's recent senders' information
 * (especially MAC TX sequence). This macro defines the check-duplicate-packet data unit's
 * life time in cycle count. Once expired, the related data unit will be reset.
 */
#ifndef GNRC_GOMACH_RX_DUPCHK_UNIT_LIFE
#define GNRC_GOMACH_RX_DUPCHK_UNIT_LIFE            (30U)
#endif

/**
 * @brief Maximum number of slots allowed to be allocated in one GoMacH cycle.
 *
 * GoMacH dynamically allocate transmission slots to senders that have pending packet in
 * the vTDMA period. This macro defines the maximum number of slots allowed to be allocated
 * in one GoMacH cycle.
 */
#ifndef GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM
#define GNRC_GOMACH_MAX_ALLOC_SLOTS_NUM           (25U)
#endif

/**
 * @brief Maximum t2k attempts before going to t2u in GoMacH.
 *
 * After phase-locked with the receiver, a sender run a t2k (transmit-to-known) procedure
 * to transmit packet to the phase-known device. However, due to factors like timer drift
 * or busy-channel, a transmission attempt may fail in t2k. If the t2k attempt count has
 * reach this @ref GNRC_GOMACH_REPHASELOCK_THRESHOLD, the sender regards pahse-locked failed
 * due to timer drifer. In this case, it will adopt t2u (transmit-to-unknown) procedure to
 * get re-phase-locked with the receiver.
 */
#ifndef GNRC_GOMACH_REPHASELOCK_THRESHOLD
#define GNRC_GOMACH_REPHASELOCK_THRESHOLD      (4U)
#endif

/**
 * @brief Maximum t2u attempts before dropping data packet in GoMacH.
 *
 * In case the receiver's phase is unknown to the sender, the sender adopts the t2u
 * (transmit-to-unknown) procedure to get phase-locked with the receiver. This macro
 * defines the maximum t2u attempts before dropping the data packet in GoMacH.
 */
#ifndef GNRC_GOMACH_T2U_RETYR_THRESHOLD
#define GNRC_GOMACH_T2U_RETYR_THRESHOLD      (2U)
#endif

/**
 * @brief   Initialize an instance of the GoMacH layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the GoMacH instance
 * @param[in] name          name of the thread housing the GoMacH instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of GoMacH thread on success
 * @return                  -EINVAL if creation of thread fails
 * @return                  -ENODEV if *dev* is invalid
 */
kernel_pid_t gnrc_gomach_init(char *stack, int stacksize, char priority,
                                 const char *name, gnrc_netdev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* NET_GNRC_GOMACH_GOMACH_H */
/** @} */

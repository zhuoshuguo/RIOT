/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @defgroup    net_gnrc_priority_pktqueue Priority packet queue
 * @ingroup     net_gnrc
 * @file
 * @brief       Wrapper for priority_queue that holds gnrc_pktsnip_t* and is
 *              aware of it's length.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 * @}
 */

#ifndef GNRC_PRIORITY_PKTQUEUE_H
#define GNRC_PRIORITY_PKTQUEUE_H

#include <stdint.h>
#include <priority_queue.h>
#include <net/gnrc/pkt.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gnrc_priority_pktqueue_node {
    struct gnrc_priority_pktqueue_node *next;   /**< next queue node */
    uint32_t priority;                          /**< queue node priority */
    gnrc_pktsnip_t *pkt;                        /**< queue node data */
} gnrc_priority_pktqueue_node_t;

/* TODO: Description */
typedef priority_queue_t gnrc_priority_pktqueue_t;

#define PRIORITY_PKTQUEUE_NODE_INIT(priority, pkt) { NULL, priority, pkt}

#define PRIORITY_PKTQUEUE_INIT { NULL }

static inline void priority_pktqueue_node_init(gnrc_priority_pktqueue_node_t *node,
                                               uint32_t priority,
                                               gnrc_pktsnip_t *pkt)
{
    node->next = NULL;
    node->priority = priority;
    node->pkt = pkt;
}

static inline void priority_pktqueue_init(gnrc_priority_pktqueue_t *q)
{
    gnrc_priority_pktqueue_t qn = PRIORITY_PKTQUEUE_INIT;
    *q = qn;
}

static inline uint32_t priority_pktqueue_length(gnrc_priority_pktqueue_t *q)
{
	uint32_t length = 0;
	priority_queue_node_t *node = q->first;
	if(!node)
		return length;

	length ++;
	while(node->next!=NULL){
		length ++;
		node = node->next;
	}
    return length;
}

void priority_pktqueue_flush(gnrc_priority_pktqueue_t* q);

/* Get first element and remove it from queue */
gnrc_pktsnip_t* priority_pktqueue_pop(gnrc_priority_pktqueue_t* q);

/* Get first element without removing */
gnrc_pktsnip_t* priority_pktqueue_head(gnrc_priority_pktqueue_t* q);

void priority_pktqueue_push(gnrc_priority_pktqueue_t* q,
                                            gnrc_priority_pktqueue_node_t *node);

#ifdef __cplusplus
}
#endif

#endif /* GNRC_PRIORITY_PKTQUEUE_H */

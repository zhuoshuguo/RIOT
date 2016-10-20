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

#include "net/gnrc/pktbuf.h"
#include <net/gnrc/priority_pktqueue.h>

/******************************************************************************/

static inline void _free_node(gnrc_priority_pktqueue_node_t *node)
{
    assert(node != NULL);

    priority_queue_node_init((priority_queue_node_t *)node);
}

/******************************************************************************/

gnrc_pktsnip_t* priority_pktqueue_pop(gnrc_priority_pktqueue_t* q)
{
    if(!q || (priority_pktqueue_length(q) == 0))
        return NULL;
    priority_queue_node_t *head = priority_queue_remove_head(q);
    gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) head->data;
    _free_node((gnrc_priority_pktqueue_node_t *)head);
    return pkt;
}

/******************************************************************************/

gnrc_pktsnip_t* priority_pktqueue_head(gnrc_priority_pktqueue_t* q)
{
    if(!q || (priority_pktqueue_length(q) == 0))
        return NULL;
    return (gnrc_pktsnip_t *)q->first->data;
}
/******************************************************************************/

void priority_pktqueue_push(gnrc_priority_pktqueue_t* q,
                            gnrc_priority_pktqueue_node_t *node)
{
    assert(q != NULL);
    assert(node != NULL);
    assert(node->pkt != NULL);
    assert(sizeof(unsigned int) == sizeof(gnrc_pktsnip_t));

    priority_queue_add(q, (priority_queue_node_t *)node);
}

/******************************************************************************/

void priority_pktqueue_flush(gnrc_priority_pktqueue_t* q)
{
    if(priority_pktqueue_length(q) == 0)
        return;

    gnrc_priority_pktqueue_node_t* node;
    while( (node = (gnrc_priority_pktqueue_node_t *)priority_queue_remove_head(q)) )
    {
        gnrc_pktbuf_release(node->pkt);
        _free_node(node);
    }
}

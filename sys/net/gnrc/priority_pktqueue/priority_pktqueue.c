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

#include <net/gnrc.h>
#include <net/gnrc/priority_pktqueue.h>

/******************************************************************************/

static packet_queue_node_t* _alloc_node(gnrc_priority_pktqueue_t* q, gnrc_pktsnip_t *pkt)
{
    assert(q != NULL);
    assert(q->buffer != NULL);
    assert(q->buffer_size > 0);
    assert(sizeof(unsigned int) == sizeof(gnrc_pktsnip_t*));

    for (size_t i = 0; i < q->buffer_size; i++) {
        if( (q->buffer[i].data == 0) &&
            (q->buffer[i].next == NULL))
        {
            q->buffer[i].data = (unsigned int) pkt;
            return &(q->buffer[i]);
        }
    }

    return NULL;
}

/******************************************************************************/

static inline void _free_node(packet_queue_node_t *node)
{
    assert(node != NULL);

    node->data = 0;
    node->next = NULL;
}

/******************************************************************************/

void priority_pktqueue_init(gnrc_priority_pktqueue_t* q, packet_queue_node_t buffer[], size_t buffer_size)
{
    assert(q != NULL);
    assert(buffer != NULL);
    assert(buffer_size > 0);

    q->buffer = buffer;
    q->buffer_size = buffer_size;
    q->queue.first = NULL;
}

/******************************************************************************/

gnrc_pktsnip_t* priority_pktqueue_pop(gnrc_priority_pktqueue_t* q)
{
    if(!q || (priority_pktqueue_length(q) == 0))
        return NULL;
    packet_queue_node_t* head = priority_queue_remove_head(&(q->queue));
    gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) head->data;
    _free_node(head);
    return pkt;
}

/******************************************************************************/

gnrc_pktsnip_t* priority_pktqueue_head(gnrc_priority_pktqueue_t* q)
{
    if(!q || (priority_pktqueue_length(q) == 0))
        return NULL;
    return (gnrc_pktsnip_t*) q->queue.first->data;
}
/******************************************************************************/

packet_queue_node_t*
priority_pktqueue_push(gnrc_priority_pktqueue_t* q, gnrc_pktsnip_t* snip, uint32_t priority)
{
    assert(q != NULL);
    assert(snip != NULL);

    packet_queue_node_t* node = _alloc_node(q, snip);

    if(node)
    {
        node->priority = priority;
        priority_queue_add(&(q->queue), node);
    }
    return node;
}

/******************************************************************************/

void priority_pktqueue_flush(gnrc_priority_pktqueue_t* q)
{
    if(priority_pktqueue_length(q) == 0)
        return;

    packet_queue_node_t* node;
    while( (node = priority_queue_remove_head(&(q->queue))) )
    {
        gnrc_pktbuf_release((gnrc_pktsnip_t*) node->data);
        _free_node(node);
    }
}

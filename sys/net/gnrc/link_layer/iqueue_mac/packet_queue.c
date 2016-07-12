/*
 * Copyright (C) 2016 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iqueuemac
 * @file
 * @brief       Wrapper for priority_queue that holds gnrc_pktsnip_t* and is
 *              aware of it's length.
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo  <shuguo.zhuo@inria.fr>
 * @}
 */
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <kernel_types.h>
#include <lpm.h>
#include "msg.h"
#include "thread.h"
#include "random.h"
#include <timex.h>
#include <periph/rtt.h>
#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev2.h"
#include "net/gnrc/netdev2.h"
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>

#include <net/gnrc.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>

/******************************************************************************/

static packet_queue_node_t* _alloc_node(packet_queue_t* q, gnrc_pktsnip_t *pkt)
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

void packet_queue_init(packet_queue_t* q, packet_queue_node_t buffer[], size_t buffer_size)
{
    assert(q != NULL);
    assert(buffer != NULL);
    assert(buffer_size > 0);

    q->buffer = buffer;
    q->buffer_size = buffer_size;
    q->queue.first = NULL;
}

/******************************************************************************/

gnrc_pktsnip_t* packet_queue_pop(packet_queue_t* q)
{
    if(!q || (q->length == 0))
        return NULL;
    packet_queue_node_t* head = priority_queue_remove_head(&(q->queue));
    gnrc_pktsnip_t* pkt = (gnrc_pktsnip_t*) head->data;
    _free_node(head);
    q->length--;
    return pkt;
}

/******************************************************************************/

gnrc_pktsnip_t* packet_queue_head(packet_queue_t* q)
{
    if(!q || (q->length == 0))
        return NULL;
    return (gnrc_pktsnip_t*) q->queue.first->data;
}
/******************************************************************************/

packet_queue_node_t*
packet_queue_push(packet_queue_t* q, gnrc_pktsnip_t* snip, uint32_t priority)
{
    assert(q != NULL);
    assert(snip != NULL);

    packet_queue_node_t* node = _alloc_node(q, snip);

    if(node)
    {
        node->priority = priority;
        priority_queue_add(&(q->queue), node);
        q->length++;
        puts("puts a pkt.");
    }
    return node;
}

/******************************************************************************/

void packet_queue_flush(packet_queue_t* q)
{
    if(q->length == 0)
        return;

    packet_queue_node_t* node;
    while( (node = priority_queue_remove_head(&(q->queue))) )
    {
        gnrc_pktbuf_release((gnrc_pktsnip_t*) node->data);
        _free_node(node);
    }
    q->length = 0;
}

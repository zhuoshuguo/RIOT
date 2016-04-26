/*
 * Copyright (C) 2015 Daniel Krebs
 * Copyright (C) 2016 Shuguo Zhuo
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iQueueMAC
 * @file
 * @brief       Internal functions of iQueueMAC
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      Shuguo Zhuo <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>
#include <periph/rtt.h>
#include <net/gnrc.h>
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>

#include "include/iqueuemac_internal.h"
#include "include/iqueuemac_types.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/******************************************************************************/

int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[])
{
    int res;
    gnrc_netif_hdr_t* netif_hdr;

    if(!pkt)
        return -ENODEV;

    netif_hdr = (gnrc_netif_hdr_t*) pkt->data;
    if( (res = netif_hdr->dst_l2addr_len) <= 0)
        return -ENOENT;

    *pointer_to_addr = gnrc_netif_hdr_get_dst_addr(netif_hdr);
    return res;
}

/******************************************************************************/

/* Find a payload based on it's protocol type */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type)
{
    while(pkt != NULL)
    {
        if(pkt->type == type) {
            return pkt->data;
        }
        pkt = pkt->next;
    }
    return NULL;
}

/******************************************************************************/

int _find_neighbour(iqueuemac_t* iqueuemac, uint8_t* dst_addr, int addr_len)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->tx.neighbours;

    for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr.len == addr_len) {
            if(memcmp(&(neighbours[i].l2_addr.addr), dst_addr, addr_len) == 0) {
                return i;
            }
        }
    }
    return -1;
}

/******************************************************************************/

/* Free first empty queue that is not active */
int _free_neighbour(iqueuemac_t* iqueuemac)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->tx.neighbours;

    for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if( packet_queue_length(&(neighbours[i].queue)) == 0) {
            /* Mark as free */
            neighbours[i].l2_addr.len = 0;
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

int _alloc_neighbour(iqueuemac_t* iqueuemac)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->tx.neighbours;

    for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr.len == 0) {
            packet_queue_init(&(neighbours[i].queue),
                              iqueuemac->tx._queue_nodes,
                              (sizeof(iqueuemac->tx._queue_nodes) / sizeof(packet_queue_node_t)));
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

void _init_neighbour(iqueuemac_tx_neighbour_t* neighbour, uint8_t* addr, int len)
{
    assert(neighbour != NULL);
    assert(addr  != NULL);
    assert(len > 0);

    neighbour->l2_addr.len = len;
    neighbour->cp_phase = IQUEUEMAC_PHASE_UNINITIALIZED;
    memcpy(&(neighbour->l2_addr.addr), addr, len);
}

/******************************************************************************/

uint32_t _ticks_to_phase(uint32_t ticks)
{
    return (ticks % RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US));
}
/******************************************************************************/

bool _queue_tx_packet(iqueuemac_t* iqueuemac,  gnrc_pktsnip_t* pkt)
{

    iqueuemac_tx_neighbour_t* neighbour;
    int neighbour_id;

    if(_packet_is_broadcast(pkt)) {
        /* Broadcast queue is neighbour 0 by definition */
        neighbour_id = 0;
        neighbour = _get_neighbour(iqueuemac, neighbour_id);

    } else {
        uint8_t* addr;
        int addr_len;
        bool neighbour_known = true;

        /* Get destination address of packet */
        addr_len = _get_dest_address(pkt, &addr);
        if(addr_len <= 0) {
            DEBUG("[iqueuemac-int] Packet has no destination address\n");
            gnrc_pktbuf_release(pkt);
            return false;
        }

        /* Search for existing queue for destination */
        neighbour_id = _find_neighbour(iqueuemac, addr, addr_len);

        /* Neighbour node doesn't have a queue yet */
        if(neighbour_id < 0) {
            neighbour_known = false;

            /* Try to allocate neighbour entry */
            neighbour_id = _alloc_neighbour(iqueuemac);

            /* No neighbour entries left */
            if(neighbour_id < 0) {
                DEBUG("[iqueuemac-int] No neighbour entries left, maybe increase "
                      "iqueuemac_NEIGHBOUR_COUNT for better performance\n");

                /* Try to free an unused queue */
                neighbour_id = _free_neighbour(iqueuemac);

                /* All queues are in use, so reject */
                if(neighbour_id < 0) {
                    DEBUG("[iqueuemac-int] Couldn't allocate tx queue for packet\n");
                    puts("Shuguo: there is no free neighbor for caching packet! ");
                    gnrc_pktbuf_release(pkt);
                    return false;
                }
            }
        }

        neighbour = _get_neighbour(iqueuemac, neighbour_id);

        if(!neighbour_known) {
            _init_neighbour(neighbour, addr, addr_len);
        }

    }

    printf("Shuguo: the current find neighbour_id is %d \n", neighbour_id);
    printf("Shuguo: the inited addr in the neighbor-list is %d %d \n", iqueuemac->tx.neighbours[neighbour_id].l2_addr.addr[1], iqueuemac->tx.neighbours[neighbour_id].l2_addr.addr[0]);

    if(packet_queue_push(&(neighbour->queue), pkt, 0) == NULL) {
    	puts("Shuguo: Cann't push packet into queue, queue is perhaps full! ");
        gnrc_pktbuf_release(pkt);
        return false;
    }

    printf("Shuguo: the current find neighbour %d 's queue-length is %d. \n", neighbour_id, (int)iqueuemac->tx.neighbours[neighbour_id].queue.length);

    DEBUG("[iqueuemac-int] Queuing pkt to neighbour #%d\n", neighbour_id);

    return true;
}


void iqueuemac_trun_on_radio(iqueuemac_t* iqueuemac)
{
	netopt_state_t devstate;
	devstate = NETOPT_STATE_IDLE;
	iqueuemac->netdev2_driver->set(iqueuemac->netdev->dev,
	                              NETOPT_STATE,
	                              &devstate,
	                              sizeof(devstate));
}

void iqueuemac_trun_off_radio(iqueuemac_t* iqueuemac)
{
	netopt_state_t devstate;
	devstate = NETOPT_STATE_SLEEP;
	iqueuemac->netdev2_driver->set(iqueuemac->netdev->dev,
	                              NETOPT_STATE,
	                              &devstate,
	                              sizeof(devstate));
}


int iqueuemac_send(iqueuemac_t* iqueuemac, gnrc_pktsnip_t *pkt, netopt_enable_t* csma_enable)
{
	//netopt_enable_t csma_enable_send;
	//csma_enable_send = csma_enable;
	iqueuemac->netdev2_driver->set(iqueuemac->netdev->dev, NETOPT_CSMA, csma_enable, sizeof(netopt_enable_t));

	iqueuemac->netdev->send(iqueuemac->netdev, pkt);
	return 1;

}


void iqueue_send_preamble_ack(iqueuemac_t* iqueuemac, iqueuemac_packet_info_t* info)
{
	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt;
	gnrc_netif_hdr_t* nethdr_preamble_ack;
	uint32_t next_cp_timing_ticks;

	next_cp_timing_ticks = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US) - rtt_get_counter();

	/* Assemble preamble_ack packet */
	iqueuemac_frame_preamble_ack_t iqueuemac_preamble_ack_hdr;
	iqueuemac_preamble_ack_hdr.header.type = FRAMETYPE_PREAMBLE_ACK;
	iqueuemac_preamble_ack_hdr.dst_addr = info->src_addr;
	iqueuemac_preamble_ack_hdr.device_type = ROUTER;
	iqueuemac_preamble_ack_hdr.father_router = iqueuemac->own_addr;
	iqueuemac_preamble_ack_hdr.next_cp_time = next_cp_timing_ticks;

	pkt = gnrc_pktbuf_add(NULL, &iqueuemac_preamble_ack_hdr, sizeof(iqueuemac_preamble_ack_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
		    ;
	}

	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
	      ;
	}
	/* We wouldn't get here if add the NETIF header had failed, so no
		sanity checks needed */
	nethdr_preamble_ack = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

	/* Construct NETIF header and insert address for WA packet */
	gnrc_netif_hdr_init(nethdr_preamble_ack, 0, 0);
	//gnrc_netif_hdr_set_dst_addr(nethdr_wa, lwmac->rx.l2_addr.addr, lwmac->rx.l2_addr.len);

	/* Send WA as broadcast*/
	nethdr_preamble_ack->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

	netopt_enable_t csma_enable;
	csma_enable = NETOPT_DISABLE;
	iqueuemac_send(iqueuemac, pkt, &csma_enable);

}

int iqueuemac_assemble_and_send_beacon(iqueuemac_t* iqueuemac)
{
	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt;
	gnrc_netif_hdr_t* nethdr_beacon;

	//assert(lwmac->rx.l2_addr.len != 0);

	/* Assemble Beacon packet */
	iqueuemac_frame_beacon_t iqueuemac_hdr;
	iqueuemac_hdr.header.type = FRAMETYPE_BEACON;
    iqueuemac_hdr.next_cp_time = IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_DURATION_US;
	iqueuemac_hdr.sub_channel_seq = 12;
	iqueuemac_hdr.schedulelist_size = 0;
	//lwmac_hdr.dst_addr = lwmac->rx.l2_addr;

	pkt = gnrc_pktbuf_add(NULL, &iqueuemac_hdr, sizeof(iqueuemac_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
	      ;
	}

	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
	      ;
	}

	/* We wouldn't get here if add the NETIF header had failed, so no
	sanity checks needed */
	nethdr_beacon = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

   /* Construct NETIF header and insert address for WA packet */
	gnrc_netif_hdr_init(nethdr_beacon, 0, 0);
	//gnrc_netif_hdr_set_dst_addr(nethdr_wa, lwmac->rx.l2_addr.addr, lwmac->rx.l2_addr.len);

    /* Send WA as broadcast*/
    nethdr_beacon->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

    /* Disable Auto ACK */
    //netopt_enable_t autoack = NETOPT_DISABLE;
 	//lwmac->netdev2_driver->set(lwmac->netdev->dev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

    netopt_enable_t csma_enable;
    csma_enable = NETOPT_DISABLE;
    iqueuemac_send(iqueuemac, pkt, &csma_enable);

	return 1;

}


/******************************************************************************/

int _parse_packet(gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* info)
{
    gnrc_netif_hdr_t* netif_hdr;
    gnrc_pktsnip_t* iqueuemac_snip;
    iqueuemac_hdr_t* iqueuemac_hdr;

    assert(info != NULL);
    assert(pkt != NULL);

    netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
    if(netif_hdr == NULL) {
        return -1;
    }

    if(netif_hdr->dst_l2addr_len > sizeof(info->dst_addr)) {
        return -3;
    }

    if(netif_hdr->src_l2addr_len > sizeof(info->src_addr)) {
        return -4;
    }


    /* Dissect iQueue-MAC header */

    /* every frame has header as first member */
    iqueuemac_hdr = (iqueuemac_hdr_t*) pkt->data;

    switch(iqueuemac_hdr->type) {
    case FRAMETYPE_BEACON:{
    	iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_beacon_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

    case FRAMETYPE_PREAMBLE:{
       	iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_preamble_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

    case FRAMETYPE_PREAMBLE_ACK:{
        iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_preamble_ack_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

        /*
    case FRAMETYPE_WA:
        lwmac_snip = gnrc_pktbuf_mark(pkt, sizeof(lwmac_frame_wa_t), GNRC_NETTYPE_LWMAC);
        break;
    case FRAMETYPE_DATA:
        lwmac_snip = gnrc_pktbuf_mark(pkt, sizeof(lwmac_frame_data_t), GNRC_NETTYPE_LWMAC);
        break;
    case FRAMETYPE_BROADCAST:
        lwmac_snip = gnrc_pktbuf_mark(pkt, sizeof(lwmac_frame_broadcast_t), GNRC_NETTYPE_LWMAC);
        break;  */
    default:
        return -2;
    }

    /* Memory location may have changed while marking */
    iqueuemac_hdr = iqueuemac_snip->data;


    /****** get the destination address *************/
    switch(iqueuemac_hdr->type){

        case FRAMETYPE_BEACON:{
        	info->dst_addr.len = 2;
        	info->dst_addr.addr[0] = 0xff;
        	info->dst_addr.addr[1] = 0xff;
        }break;

        case FRAMETYPE_PREAMBLE:{
        	info->dst_addr = ((iqueuemac_frame_preamble_t*)iqueuemac_hdr)->dst_addr;
        }break;

        case FRAMETYPE_PREAMBLE_ACK:{
        	info->dst_addr = ((iqueuemac_frame_preamble_ack_t*)iqueuemac_hdr)->dst_addr;
        }break;

        default:break;

    }
    /****** get the source address *************/
    if(netif_hdr->src_l2addr_len) {
        info->src_addr.len = netif_hdr->src_l2addr_len;
        memcpy(info->src_addr.addr,
               gnrc_netif_hdr_get_src_addr(netif_hdr),
               netif_hdr->src_l2addr_len);
    }

    info->header = iqueuemac_hdr;

    return 0;
}
/******************************************************************************/
int iqueue_push_packet_to_dispatch_queue(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt)
{
	for(unsigned i = 0; i < IQUEUEMAC_DISPATCH_BUFFER_SIZE; i++) {
	   /* Buffer will be filled bottom-up and emptied completely so no holes */
	   if(buffer[i] == NULL) {
	     buffer[i] = pkt;
	     return 0;
	    }
	}
	return -1;
}

void iqueue_cp_receive_packet_process(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	int res = _parse_packet(pkt, &receive_packet_info);

    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
          case FRAMETYPE_BEACON:{

          }break;

          case FRAMETYPE_PREAMBLE:{
        	  if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr)){
        		  iqueue_send_preamble_ack(iqueuemac, &receive_packet_info);
        	  }else{
        		  /****** this means that there is a long preamble period, so quit this cycle and go to sleep.****/
        		  iqueuemac->quit_current_cycle = true;
        	  }
           }break;

          case FRAMETYPE_PREAMBLE_ACK:{

          }break;

          case FRAMETYPE_DATA:{
        	  iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt);
          }break;

          default:break;
  	    }
    }


}

/******************************************************************************/



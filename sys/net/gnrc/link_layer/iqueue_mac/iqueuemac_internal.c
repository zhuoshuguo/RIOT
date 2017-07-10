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
 * @brief       Internal functions of iqueue_mac
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 * @author      shuguo Zhuo <shuguo.zhuo@inria.fr>
 * @}
 */

#include <stdbool.h>
#include <periph/rtt.h>
#include <net/gnrc.h>
#include "random.h"
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>
#include <net/gnrc/iqueue_mac/hdr.h>

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
            neighbours[i].mac_type = UNKNOWN;
            return i;
        }
    }
    return -1;
}

/******************************************************************************/

int _alloc_neighbour(iqueuemac_t* iqueuemac)
{
    iqueuemac_tx_neighbour_t* neighbours = iqueuemac->tx.neighbours;

    for(int i = 0; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {
        if(neighbours[i].l2_addr.len == 0) {
        	neighbours[i].mac_type = UNKNOWN;
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

    neighbour->mac_type = UNKNOWN;
    neighbour->l2_addr.len = len;
    neighbour->cp_phase = IQUEUEMAC_PHASE_UNINITIALIZED;
    memcpy(&(neighbour->l2_addr.addr), addr, len);
}

/******************************************************************************/

uint32_t _ticks_to_phase(uint32_t ticks)
{
    return (ticks % RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US));
}


uint32_t _phase_now(iqueuemac_t* iqueuemac){

	uint32_t phase_now;
	phase_now = rtt_get_counter();

	/* in case that rtt overflows */
	if(phase_now < iqueuemac->last_wakeup){
		uint32_t gap_to_full;
		gap_to_full = IQUEUEMAC_PHASE_MAX - iqueuemac->last_wakeup;
		phase_now += gap_to_full;
	}else{
		phase_now = phase_now - iqueuemac->last_wakeup;
	}

	return phase_now;
}

uint32_t _ticks_until_phase(iqueuemac_t* iqueuemac, uint32_t phase)  //
{
	/*
	uint32_t phase_now;
	uint32_t wait_phase_duration;

	phase_now = rtt_get_counter();

	if(phase >= phase_now){
		wait_phase_duration = phase - phase_now;
	}else{
		wait_phase_duration = phase_now - phase;
		wait_phase_duration += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
	}*/


    long int tmp = phase - _phase_now(iqueuemac); //rtt_get_counter();
    if(tmp < 0) {
        tmp += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
    }

    return (uint32_t)tmp;
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
                    //puts("iqueuemac: there is no free neighbor for caching packet! ");
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

    //printf("iqueuemac: the current find neighbour_id is %d \n", neighbour_id);
    //printf("iqueuemac: the inited addr in the neighbor-list is %d %d \n", iqueuemac->tx.neighbours[neighbour_id].l2_addr.addr[1], iqueuemac->tx.neighbours[neighbour_id].l2_addr.addr[0]);

    if(packet_queue_push(&(neighbour->queue), pkt, 0) == NULL) {
    	puts("tx-queue: full, drop data");
        gnrc_pktbuf_release(pkt);
        return false;
    }

    //printf("iqueuemac: the current find neighbour %d 's queue-length is %d. \n", neighbour_id, (int)iqueuemac->tx.neighbours[neighbour_id].queue.length);

    //DEBUG("[iqueuemac-int] Queuing pkt to neighbour #%d\n", neighbour_id);

    return true;
}


void iqueuemac_trun_on_radio(iqueuemac_t* iqueuemac)
{
	netopt_state_t devstate;
	devstate = NETOPT_STATE_IDLE;
	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
	                              NETOPT_STATE,
	                              &devstate,
	                              sizeof(devstate));
}

void iqueuemac_trun_off_radio(iqueuemac_t* iqueuemac)
{
	netopt_state_t devstate;
	devstate = NETOPT_STATE_SLEEP;
	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
	                              NETOPT_STATE,
	                              &devstate,
	                              sizeof(devstate));
}


void iqueuemac_set_autoack(iqueuemac_t* iqueuemac, netopt_enable_t autoack)
{
	netopt_enable_t setautoack = autoack;

	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
								  NETOPT_AUTOACK,
	                              &setautoack,
	                              sizeof(setautoack));
}

void iqueuemac_set_ack_req(iqueuemac_t* iqueuemac, netopt_enable_t ack_req)
{
    netopt_enable_t set_ack_req = ack_req;

    iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
                                  NETOPT_ACK_REQ,
                                  &set_ack_req,
                                  sizeof(set_ack_req));
}

netopt_state_t _get_netdev_state(iqueuemac_t* iqueuemac)
{
    netopt_state_t state;

	if (0 < iqueuemac->netdev_driver->get(iqueuemac->netdev->dev,
                                       NETOPT_STATE,
                                       &state,
                                       sizeof(state)))
	{
        return state;
    }
    return -1;
}

void iqueuemac_set_promiscuousmode(iqueuemac_t* iqueuemac, netopt_enable_t enable)
{
	netopt_enable_t set_enable = enable;

	//iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_PROMISCUOUSMODE, &enable, sizeof(enable));

	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
								  NETOPT_PROMISCUOUSMODE,
	                              &set_enable,
	                              sizeof(set_enable));
}


void iqueuemac_turn_radio_channel(iqueuemac_t* iqueuemac, uint16_t channel_num)
{
	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev, NETOPT_CHANNEL, &channel_num, sizeof(channel_num));
}

void iqueuemac_set_raddio_to_listen_mode(iqueuemac_t* iqueuemac){

	iqueuemac_trun_on_radio(iqueuemac);
}

int iqueuemac_send(iqueuemac_t* iqueuemac, gnrc_pktsnip_t *pkt, netopt_enable_t csma_enable)
{
	netopt_enable_t csma_enable_send;
	int res;
	csma_enable_send = csma_enable;
	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev, NETOPT_CSMA, &csma_enable_send, sizeof(netopt_enable_t));

	iqueuemac->tx.tx_finished = false;
	gnrc_netdev_set_tx_feedback(iqueuemac->netdev,TX_FEEDBACK_UNDEF);
	res = iqueuemac->netdev->send(iqueuemac->netdev, pkt);

	/*
	netopt_state_t devstate;
	devstate = NETOPT_STATE_TX;
	iqueuemac->netdev_driver->set(iqueuemac->netdev->dev,
	                              NETOPT_STATE,
	                              &devstate,
	                              sizeof(devstate));
	                              */

	return res;

}


int iqueue_send_preamble_ack(iqueuemac_t* iqueuemac, iqueuemac_packet_info_t* info)
{
	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt_iqmac;
	gnrc_pktsnip_t* pkt;
	gnrc_netif_hdr_t* nethdr_preamble_ack;
	uint32_t phase_now_ticks;  //next_cp_timing_ticks;  //next_cp_timing_us; //


	//next_cp_timing_ticks = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US) - rtt_get_counter();
	//next_cp_timing_us = IQUEUEMAC_SUPERFRAME_DURATION_US - RTT_TICKS_TO_US(rtt_get_counter());

	/* Assemble preamble_ack packet */
	iqueuemac_frame_preamble_ack_t iqueuemac_preamble_ack_hdr;
	iqueuemac_preamble_ack_hdr.header.type = FRAMETYPE_PREAMBLE_ACK;
	iqueuemac_preamble_ack_hdr.dst_addr = info->src_addr;
	iqueuemac_preamble_ack_hdr.device_type = iqueuemac->mac_type;
	//maybe we don't need this "father_router_addr" parameter anymore
	iqueuemac_preamble_ack_hdr.father_router = iqueuemac->father_router_addr;

	phase_now_ticks = _phase_now(iqueuemac); //rtt_get_counter();

	iqueuemac_preamble_ack_hdr.phase_in_ticks = phase_now_ticks; // next_cp_timing_ticks; //  next_cp_timing_us; //

	pkt = gnrc_pktbuf_add(NULL, &iqueuemac_preamble_ack_hdr, sizeof(iqueuemac_preamble_ack_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
		puts("iqueuemac: payload buf add failed in iqueue_send_preamble_ack().");
		return -ENOBUFS;
	}
	pkt_iqmac = pkt;

	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
		puts("iqueuemac: netif_hdr add failed in iqueue_send_preamble_ack().");
		gnrc_pktbuf_release(pkt_iqmac);
		return -ENOBUFS;
	}
	pkt_iqmac = pkt;

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
	int res;
	res = iqueuemac_send(iqueuemac, pkt, csma_enable);
    if(res < 0){
		puts("iqueuemac: send preamble-ack failed in iqueue_send_preamble_ack().");
    	gnrc_pktbuf_release(pkt_iqmac);
    }
	return res;

}

int iqueuemac_assemble_and_send_beacon(iqueuemac_t* iqueuemac)
{
	int i;
	int j=0;
	uint8_t total_tdma_node_num = 0;
	uint8_t total_tdma_slot_num = 0;
	bool slots_full = false;

	iqueuemac->rx.router_vtdma_mana.total_slots_num = 0;

	for(i=0;i<IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT;i++)
	{
		if(iqueuemac->rx.rx_register_list[i].queue_indicator >0){
			total_tdma_slot_num = iqueuemac->rx.rx_register_list[i].queue_indicator;
			break;
		}
	}
	if(total_tdma_slot_num == 0) {
		return 0;
	}

	total_tdma_slot_num = 0;

	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt;
	gnrc_pktsnip_t* pkt_iqmac;
	gnrc_netif_hdr_t* nethdr_beacon;

	//assert(lwmac->rx.l2_addr.len != 0);

	/* Assemble Beacon packet */
	iqueuemac_frame_beacon_t iqueuemac_hdr;
	iqueuemac_hdr.header.type = FRAMETYPE_BEACON;
	if(iqueuemac->phase_changed == false){
		iqueuemac_hdr.current_phase = _phase_now(iqueuemac);
	}else{
		iqueuemac_hdr.current_phase = 0;
	}
	iqueuemac_hdr.sub_channel_seq = iqueuemac->sub_channel_num; //iqueuemac->rx.router_vtdma_mana.sub_channel_seq;
	//iqueuemac_hdr.schedulelist_size = 0;

	/********* Add the slots schedule list functionality here!!!  *********/

	iqueuemac->rx.router_vtdma_mana.total_slots_num = 0;

	l2_id_t  id_list[IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT];
	uint8_t  slots_list[IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT];

	/**** search router-type first ****/
	for(i=0;i<IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT;i++)
	{
		if((iqueuemac->rx.rx_register_list[i].mac_type == ROUTER)&&(iqueuemac->rx.rx_register_list[i].queue_indicator > 0))
		{
			//id_list[j].addr = iqueuemac->rx.rx_register_list[i].node_addr.addr;
			memcpy(id_list[j].addr,
					iqueuemac->rx.rx_register_list[i].node_addr.addr,
					iqueuemac->rx.rx_register_list[i].node_addr.len);

			slots_list[j] = iqueuemac->rx.rx_register_list[i].queue_indicator;

			total_tdma_node_num ++;
			total_tdma_slot_num += slots_list[j];

			if(total_tdma_slot_num >= IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM){
				uint8_t redueced_slots_num;
				redueced_slots_num = total_tdma_slot_num - IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM;
				slots_list[j] -= redueced_slots_num;
				total_tdma_slot_num -= redueced_slots_num;
				//j++;
				slots_full = true;
				break;
			}
			j++;
		}

	}
	/**** search node-type then ****/
	for(i=0;i<IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT;i++)
	{
		if(slots_full == true){
			break;
		}

		if((iqueuemac->rx.rx_register_list[i].mac_type == NODE)&&(iqueuemac->rx.rx_register_list[i].queue_indicator > 0))
		{
			//id_list[j].addr = iqueuemac->rx.rx_register_list[i].node_addr.addr;
			memcpy(id_list[j].addr,
			     	iqueuemac->rx.rx_register_list[i].node_addr.addr,
					iqueuemac->rx.rx_register_list[i].node_addr.len);
			slots_list[j] = iqueuemac->rx.rx_register_list[i].queue_indicator;

			total_tdma_node_num ++;
			total_tdma_slot_num += slots_list[j];

			if(total_tdma_slot_num >= IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM){
				uint8_t redueced_slots_num;
				redueced_slots_num = total_tdma_slot_num - IQUEUEMAC_MAX_SCHEDULE_SLOTS_NUM;
				slots_list[j] -= redueced_slots_num;
				total_tdma_slot_num -= redueced_slots_num;
				slots_full = true;
				break;
			}
			j++;
		}
	}

	iqueuemac_hdr.schedulelist_size = total_tdma_node_num;

	if(total_tdma_node_num > 0){

		iqueuemac->rx.router_vtdma_mana.total_slots_num = total_tdma_slot_num;
		//printf("iqueuemac: the total slots-number this cycle is %d . \n", iqueuemac->rx.router_vtdma_mana.total_slots_num);

		//puts("iqueuemac: schedule slots-list");
	    /**** add the slots list ****/
	    pkt = gnrc_pktbuf_add(NULL, slots_list, total_tdma_node_num * sizeof(uint8_t), GNRC_NETTYPE_IQUEUEMAC);
	    if(pkt == NULL) {
	    	puts("iqueuemac: pktbuf add failed in iqueuemac_assemble_and_send_beacon().");
	    	return -ENOBUFS;
	    }
	    pkt_iqmac = pkt;

	    /**** add the ID list ****/
	    pkt = gnrc_pktbuf_add(pkt, id_list, total_tdma_node_num * sizeof(l2_id_t), GNRC_NETTYPE_IQUEUEMAC);
	    if(pkt == NULL) {
	    	puts("iqueuemac: pktbuf add failed in iqueuemac_assemble_and_send_beacon().");
	    	gnrc_pktbuf_release(pkt_iqmac);
	    	return -ENOBUFS;
	    }
	    pkt_iqmac = pkt;

	    /**** add the beacon header ****/
	    pkt = gnrc_pktbuf_add(pkt, &iqueuemac_hdr, sizeof(iqueuemac_hdr), GNRC_NETTYPE_IQUEUEMAC);
	    if(pkt == NULL) {
	    	puts("iqueuemac: pktbuf add failed in iqueuemac_assemble_and_send_beacon().");
	    	gnrc_pktbuf_release(pkt_iqmac);
	    	return -ENOBUFS;
	    }
	    pkt_iqmac = pkt;

	}else{
		/* if there is no slots to allocate, quit sending beacon! */
		return 0;

        /*
		pkt = gnrc_pktbuf_add(NULL, &iqueuemac_hdr, sizeof(iqueuemac_hdr), GNRC_NETTYPE_IQUEUEMAC);
		if(pkt == NULL) {
			puts("iqueuemac: pktbuf add failed in iqueuemac_assemble_and_send_beacon().");
			return -ENOBUFS;
		}
		pkt_iqmac = pkt;
		*/
	}

	/********* Add the Netif header  *********/
	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
		puts("iqueuemac: pktbuf add failed in iqueuemac_assemble_and_send_beacon().");
    	gnrc_pktbuf_release(pkt_iqmac);
    	return -ENOBUFS;
	}
	pkt_iqmac = pkt;

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
 	//lwmac->netdev_driver->set(lwmac->netdev->dev, NETOPT_AUTOACK, &autoack, sizeof(autoack));

    netopt_enable_t csma_enable;
    if(iqueuemac->get_other_preamble == true){
    	/* use csma for collision avoidance of other preamble */
    	csma_enable = NETOPT_ENABLE;  // NETOPT_ENABLE
    }else{
    	/* it is important to disable csma here for CP phase overlap detect! */
    	csma_enable = NETOPT_DISABLE;  // NETOPT_ENABLE
    }
    int res;
    res = iqueuemac_send(iqueuemac, pkt, csma_enable);
    if(res < 0){
		puts("iqueuemac: send beacon failed, release it.");
    	gnrc_pktbuf_release(pkt);
    }
	return res;

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

    case FRAMETYPE_IQUEUE_DATA:{
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_data_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

    case FRAMETYPE_ANNOUNCE:{
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_announce_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

    case FRAMETYPE_BROADCAST:{
            iqueuemac_snip = gnrc_pktbuf_mark(pkt, sizeof(iqueuemac_frame_broadcast_t), GNRC_NETTYPE_IQUEUEMAC);
    }break;

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

        case FRAMETYPE_IQUEUE_DATA:{
             if(netif_hdr->dst_l2addr_len) {
                info->dst_addr.len = netif_hdr->dst_l2addr_len;
                memcpy(info->dst_addr.addr,
                       gnrc_netif_hdr_get_dst_addr(netif_hdr),
                       netif_hdr->dst_l2addr_len);
             }
        }break;

        case FRAMETYPE_BROADCAST:{
           	info->dst_addr.len = 2;
           	info->dst_addr.addr[0] = 0xff;
           	info->dst_addr.addr[1] = 0xff;
        }break;

        case FRAMETYPE_ANNOUNCE:{
          	info->dst_addr.len = 2;
          	info->dst_addr.addr[0] = 0xff;
           	info->dst_addr.addr[1] = 0xff;
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

    info->seq = netif_hdr->seq;

    return 0;
}


/******************************************************************************/
int iqueue_push_packet_to_dispatch_queue(gnrc_pktsnip_t* buffer[], gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info, iqueuemac_t* iqueuemac)
{
	for(unsigned i = 0; i < IQUEUEMAC_DISPATCH_BUFFER_SIZE; i++) {
	   /* Buffer will be filled bottom-up and emptied completely so no holes */
	   if(buffer[i] == NULL) {
	     buffer[i] = pkt;
	     return 0;
	    }
	}
	gnrc_pktbuf_release(pkt);
	puts("dispatch buffer full, drop pkt");

	return -1;
}

void iqueuemac_router_queue_indicator_update(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info)
{

	iqueuemac_frame_data_t* iqueuemac_data_hdr;

	iqueuemac_data_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);

	if(iqueuemac_data_hdr == NULL) {
	  puts("iqueuemac_data_hdr is null");
	  return;
	}

	int i;
	 /* check whether the node has registered or not  */
	for(i=0;i<IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT;i++)
	{
		if(_addr_match(&iqueuemac->rx.rx_register_list[i].node_addr, &pa_info->src_addr)){

			//iqueuemac_data_hdr->queue_indicator = iqueuemac_data_hdr->queue_indicator & 0x7F;

			iqueuemac->rx.rx_register_list[i].queue_indicator = iqueuemac_data_hdr->queue_indicator & 0x3F;
			//printf("iqueuemac: the registered queue-indicator is %d. \n", iqueuemac_data_hdr->queue_indicator);
			return;
		}
	}

	/********* the sender has not registered yet   **********/
	for(i=0;i<IQUEUEMAC_MAX_RX_SLOTS_SCHEDULE_UNIT;i++)
	{
		if((iqueuemac->rx.rx_register_list[i].node_addr.len == 0)||(iqueuemac->rx.rx_register_list[i].queue_indicator ==0))
		{
			iqueuemac->rx.rx_register_list[i].node_addr.len = pa_info->src_addr.len;
			memcpy(iqueuemac->rx.rx_register_list[i].node_addr.addr,
			       pa_info->src_addr.addr,
				   pa_info->src_addr.len);

			/****  extra mac type process ***/
			uint8_t extra_mac_type;
			extra_mac_type = iqueuemac_data_hdr->queue_indicator & 0x80;
			if(extra_mac_type == 0x80){
			    iqueuemac->rx.rx_register_list[i].mac_type = NODE;
			    //puts("iqueuemac: the registered device is node type.");
			}else{
				iqueuemac->rx.rx_register_list[i].mac_type = ROUTER;
				//puts("iqueuemac: the registered device is router type.");
			}

			iqueuemac->rx.rx_register_list[i].queue_indicator = iqueuemac_data_hdr->queue_indicator & 0x3F;
			//printf("iqueuemac: the registered queue-indicator is %d. \n", iqueuemac_data_hdr->queue_indicator);
			return;
		}
	}

}

bool iqueuemac_check_duplicate(iqueuemac_t* iqueuemac, iqueuemac_packet_info_t* pa_info)
{
	int i;
	for(i=0;i<IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE;i++){
		if(_addr_match(&iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr, &pa_info->src_addr)){
			iqueuemac->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
			if(iqueuemac->rx.check_dup_pkt.last_nodes[i].seq == pa_info->seq){
				return true;
			}else{
				iqueuemac->rx.check_dup_pkt.last_nodes[i].seq = pa_info->seq;
				return false;
			}
		}
	}

	/* look for a free unit */
	for(i=0;i<IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE;i++){
		if(iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr.len == 0){
			iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr.len = pa_info->src_addr.len;
			memcpy(iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr.addr,
					pa_info->src_addr.addr,
					pa_info->src_addr.len);
			iqueuemac->rx.check_dup_pkt.last_nodes[i].seq = pa_info->seq;
			iqueuemac->rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
			return false;
		}
	}
	puts("no free unit");
	return false;

#if 0
	uint8_t head;
	head = iqueuemac->rx.check_dup_pkt.queue_head;

	/* check with last_1 (head) address */
	if(_addr_match(&iqueuemac->rx.check_dup_pkt.last_nodes[head].node_addr, &pa_info->src_addr)){
		iqueuemac->rx.check_dup_pkt.last_nodes[head].life_cycle = 0;

		if(iqueuemac->rx.check_dup_pkt.last_nodes[head].seq == pa_info->seq){
			return true;
		}else{
			iqueuemac->rx.check_dup_pkt.last_nodes[head].seq = pa_info->seq;
			return false;
		}
	}

	bool duplicate;
	duplicate = false;
	/* search other units.*/
	for(int i=0;i<IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE;i++){

		if(_addr_match(&iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr, &pa_info->src_addr)){
			if(iqueuemac->rx.check_dup_pkt.last_nodes[i].seq == pa_info->seq){
				duplicate = true;
			}
			iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr.addr[0]=0;
			iqueuemac->rx.check_dup_pkt.last_nodes[i].node_addr.addr[1]=0;
			iqueuemac->rx.check_dup_pkt.last_nodes[i].seq=0;

			break;
		}
	}

	/* update the head */
	iqueuemac->rx.check_dup_pkt.queue_head ++;
	if(iqueuemac->rx.check_dup_pkt.queue_head >= IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE){
		iqueuemac->rx.check_dup_pkt.queue_head = 0;
	}
	head = iqueuemac->rx.check_dup_pkt.queue_head;

	iqueuemac->rx.check_dup_pkt.last_nodes[head].node_addr.len = pa_info->src_addr.len;
	memcpy(iqueuemac->rx.check_dup_pkt.last_nodes[head].node_addr.addr,
			pa_info->src_addr.addr,
			pa_info->src_addr.len);

	iqueuemac->rx.check_dup_pkt.last_nodes[head].seq = pa_info->seq;
	iqueuemac->rx.check_dup_pkt.last_nodes[head].life_cycle = 0;

	return duplicate;
#endif

#if 0
	/* check with last_1 address */
	if(_addr_match(&iqueuemac->rx.check_dup_pkt.last_1.node_addr, &pa_info->src_addr)){
		if(iqueuemac->rx.check_dup_pkt.last_1.seq == pa_info->seq){
			return true;
		}else{
			iqueuemac->rx.check_dup_pkt.last_1.seq = pa_info->seq;
			return false;
		}
	}

	/* if reach here, means the sender is not last-1, check with last_2 address */

	bool duplicate;
	duplicate = false;

	if(_addr_match(&iqueuemac->rx.check_dup_pkt.last_2.node_addr, &pa_info->src_addr)){

		if(iqueuemac->rx.check_dup_pkt.last_2.seq == pa_info->seq){
			duplicate = true;
		}
	}

	/* push last_1 to last_2 */
	iqueuemac->rx.check_dup_pkt.last_2.node_addr.len = iqueuemac->rx.check_dup_pkt.last_1.node_addr.len;
	memcpy(iqueuemac->rx.check_dup_pkt.last_2.node_addr.addr,
			iqueuemac->rx.check_dup_pkt.last_1.node_addr.addr,
			iqueuemac->rx.check_dup_pkt.last_1.node_addr.len);
	iqueuemac->rx.check_dup_pkt.last_2.seq = iqueuemac->rx.check_dup_pkt.last_1.seq;

	/* update the last_1 address */
	iqueuemac->rx.check_dup_pkt.last_1.node_addr.len = pa_info->src_addr.len;
	memcpy(iqueuemac->rx.check_dup_pkt.last_1.node_addr.addr,
			pa_info->src_addr.addr,
			pa_info->src_addr.len);

	iqueuemac->rx.check_dup_pkt.last_1.seq = pa_info->seq;

	return duplicate;
#endif

}

void iqueue_router_cp_receive_packet_process(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{

            	gnrc_pktbuf_release(pkt);
#if 0
            	uint32_t own_phase;
            	uint32_t sender_phase;

            	own_phase = _phase_now(iqueuemac);

            	iqueuemac_frame_beacon_t* iqueuemac_beacon_hdr;
            	iqueuemac_beacon_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);

            	/* this means that the beacon sender will change its phase, so meaningless to deal with this received beacon */
            	if(iqueuemac_beacon_hdr->current_phase == 0){
            		gnrc_pktbuf_release(pkt);
            		continue;
            	}

            	sender_phase = iqueuemac_beacon_hdr->current_phase + RTT_US_TO_TICKS(IQUEUEMAC_RECEPTION_MAGIN_US);

            	/* in case the sender's phase is larger */
            	if(sender_phase >= own_phase){

            		uint32_t phase_gap_ticks;

            		/* calculate the gap in ticks */
            		phase_gap_ticks = sender_phase - own_phase;

            		if(phase_gap_ticks < RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US)){
            			uint32_t backoff_ticks;
            			uint32_t random_backoff;

            			iqueuemac->phase_backoff = true;
            			backoff_ticks = RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US + IQUEUEMAC_RECEPTION_MAGIN_US) - phase_gap_ticks;

            			random_backoff = RTT_US_TO_TICKS((IQUEUEMAC_SUPERFRAME_DURATION_US/2));
            			random_backoff = random_uint32_range(0, random_backoff);
            			iqueuemac->backoff_phase_ticks =  backoff_ticks + random_backoff;
            		}
            	}else{
            		;/* currently, we don't deal with the case the sender's phase is smaller */
            	}

            	gnrc_pktbuf_release(pkt);
            	/* in the future, take CP overlape collision measurements after receive ohter's beacon!! */
#endif

            }break;

            case FRAMETYPE_PREAMBLE:{
        	    if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr)){

        	    	iqueuemac->got_preamble = true;
        	    	/** if reception is not going on, reply preamble-ack,
        	    	 * also, don't send preamble-ACK if CP ends. **/
        	    	if(_get_netdev_state(iqueuemac) == NETOPT_STATE_IDLE){
        	    		/***  disable auto-ack ***/
        	    		iqueuemac_set_autoack(iqueuemac, NETOPT_DISABLE);

        	    		int res;
        	    		res = iqueue_send_preamble_ack(iqueuemac, &receive_packet_info);
        	    		if(res < 0){
        	    			printf("preamble-ack: res %d\n",res);
        	    		}

        	    		/* Enable Auto ACK again for data reception */
        	    		iqueuemac_set_autoack(iqueuemac, NETOPT_ENABLE);
        	    	}
        	    }else{
        		    //iqueuemac->quit_current_cycle = true;
        	    	/* if receives unintended preamble, don't send beacon and quit the following vTDMA period. */
        	    	iqueuemac->get_other_preamble = true;
        	    }
        	    gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	gnrc_pktbuf_release(pkt);

            }break;


            // iqueuemac.rx.last_seq_info.seq = netif_hdr->seq;
            case FRAMETYPE_IQUEUE_DATA:{

            	if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr)) {
            		iqueuemac_router_queue_indicator_update(iqueuemac, pkt, &receive_packet_info);

                	if((iqueuemac_check_duplicate(iqueuemac, &receive_packet_info))){
                		gnrc_pktbuf_release(pkt);
                		puts("dup pkt.");
                		return;
                	}

            		iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
            		_dispatch(iqueuemac->rx.dispatch_buffer);
            	}else{/* if the data is not for the node, release it.  */
            		/* it is very unlikely that we will receive not-intended data here, since CP will not overlape! */
            		gnrc_pktbuf_release(pkt);
            	}
            	//gnrc_pktbuf_release(pkt);
            	//printf("%lu. \n", RTT_TICKS_TO_US(_phase_now(iqueuemac)));
        	    //puts("iqueuemac: router receives a data !!");
            }break;

            case FRAMETYPE_BROADCAST:{
            	iqueuemac->quit_current_cycle = true;
                iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
                _dispatch(iqueuemac->rx.dispatch_buffer);
                //puts("iqueuemac: router receives a broadcast data !!");
           }break;

            default:{
            	gnrc_pktbuf_release(pkt);break;
            }
  	    }

    }/* end of while loop */
}

void iqueuemac_update_subchannel_occu_flags(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info){

	uint16_t subchannel_seq,flag;

	subchannel_seq = 0;

	switch(pa_info->header->type){
      case FRAMETYPE_BEACON:{
    	  iqueuemac_frame_beacon_t* iqueuemac_beacon_hdr;
    	  iqueuemac_beacon_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);
    	  if(iqueuemac_beacon_hdr == NULL){
    	      puts("iqueuemac_beacon_hdr is null");
    	      return;
    	  }
    	  subchannel_seq = (uint16_t)iqueuemac_beacon_hdr->sub_channel_seq;
    	  //printf("iqueuemac: received beacon's subchannel is %d .\n", subchannel_seq);

      }break;
      case FRAMETYPE_ANNOUNCE:{
    	  iqueuemac_frame_announce_t* iqueuemac_announce_hdr;
    	  iqueuemac_announce_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);
    	  if(iqueuemac_announce_hdr == NULL){
    	      puts("iqueuemac_announce_hdr is null");
    	      return;
    	  }
    	  subchannel_seq = (uint16_t)iqueuemac_announce_hdr->subchannel_seq;
    	  //printf("iqueuemac: received announce's subchannel is %d .\n", subchannel_seq);

      }break;
      default:break;
	}

	subchannel_seq = subchannel_seq - 11;

	flag = (1 << subchannel_seq);

	iqueuemac->router_states.subchannel_occu_flags = iqueuemac->router_states.subchannel_occu_flags | flag;
	//printf("iqueuemac: subchannel flag is %d .\n", iqueuemac->router_states.subchannel_occu_flags);

}


void iqueuemac_packet_process_in_init(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	iqueuemac_update_subchannel_occu_flags(iqueuemac,pkt,&receive_packet_info);
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
            	iqueuemac->quit_current_cycle = true;
        	    gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_BROADCAST:{
            	iqueuemac->quit_current_cycle = true;
                iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
                //puts("iqueuemac: router receives a broadcast data !!");
           }break;

            case FRAMETYPE_ANNOUNCE:{
            	iqueuemac_update_subchannel_occu_flags(iqueuemac,pkt,&receive_packet_info);

            	/*** it seems that this "init_retry" procedure is unnecessary here!! maybe delete it in the future ***/
            	//iqueuemac->router_states.init_retry = true;
               	gnrc_pktbuf_release(pkt);
            }break;


            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */
}

void iqueuemac_init_choose_subchannel(iqueuemac_t* iqueuemac){

	uint16_t subchannel_seq, check_seq, own_id;

	memcpy(&own_id,
	       iqueuemac->own_addr.addr,
		   iqueuemac->own_addr.len);

	/* range from 12 to 25 */
	//own_id = 12;
	subchannel_seq = 12 + (own_id % 14);
	//printf("iqueuemac: the random selected subchannel is %d .\n", subchannel_seq);
	//printf("iqueuemac: subchannel flag is %d .\n", iqueuemac->router_states.subchannel_occu_flags);

	int i=0;
	for(i=0;i<14;i++){
		/* range from 1 to 14, lead to 2nd to 15th bit */
		check_seq = subchannel_seq - 11;
		check_seq = (1<<check_seq);

		if(check_seq & iqueuemac->router_states.subchannel_occu_flags){
			//puts("iqueuemac: subchannel exist, find next subchannel.");
			own_id += 1;
			subchannel_seq = 12 + (own_id % 14);
		}else{
			break;
		}
	}

	iqueuemac->sub_channel_num = subchannel_seq;
	//printf("iqueuemac: the final selected subchannel is %d .\n", subchannel_seq);
}

int iqueue_mac_send_preamble(iqueuemac_t* iqueuemac, netopt_enable_t use_csma)
{
	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt;
	gnrc_netif_hdr_t* nethdr_preamble;
	gnrc_pktsnip_t* pkt_iqmac;

	/* Assemble preamble packet */
	iqueuemac_frame_preamble_t iqueuemac_preamble_hdr;
	iqueuemac_preamble_hdr.header.type = FRAMETYPE_PREAMBLE;
	iqueuemac_preamble_hdr.dst_addr = iqueuemac->tx.current_neighbour->l2_addr;

	pkt = gnrc_pktbuf_add(NULL, &iqueuemac_preamble_hdr, sizeof(iqueuemac_preamble_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
		puts("iqueuemac: preamble_hdr buf add failed in iqueue_mac_send_preamble().");
		return -ENOBUFS;
	}
	pkt_iqmac = pkt;

	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
		puts("iqueuemac: preamble_hdr netif add failed in iqueue_mac_send_preamble().");
		gnrc_pktbuf_release(pkt_iqmac);
		return -ENOBUFS;
	}
	pkt_iqmac = pkt;

	/* We wouldn't get here if add the NETIF header had failed, so no
		sanity checks needed */
	nethdr_preamble = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

	/* Construct NETIF header and initiate address fields */
	gnrc_netif_hdr_init(nethdr_preamble, 0, 0);
	//gnrc_netif_hdr_set_dst_addr(nethdr_wa, lwmac->rx.l2_addr.addr, lwmac->rx.l2_addr.len);

	/* Send WA as broadcast*/
	nethdr_preamble->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

	netopt_enable_t csma_enable;
	csma_enable = use_csma;
	int res;

	res = iqueuemac_send(iqueuemac, pkt, csma_enable);
    if(res < 0){
		puts("iqueuemac: send preamble failed in iqueue_mac_send_preamble().");
    	gnrc_pktbuf_release(pkt_iqmac);
    }
	return res;
}


void iqueuemac_send_announce(iqueuemac_t* iqueuemac, netopt_enable_t use_csma)
{
	/****** assemble and send the beacon ******/
	gnrc_pktsnip_t* pkt;
	gnrc_netif_hdr_t* nethdr_announce;

	/* Assemble announce packet */
	iqueuemac_frame_announce_t iqueuemac_announce_hdr;
	iqueuemac_announce_hdr.header.type = FRAMETYPE_ANNOUNCE;
	iqueuemac_announce_hdr.subchannel_seq = iqueuemac->sub_channel_num;

	pkt = gnrc_pktbuf_add(NULL, &iqueuemac_announce_hdr, sizeof(iqueuemac_announce_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
		puts("iqueuemac: pktbuf add failed in iqueuemac_send_announce().");
	}

	pkt = gnrc_pktbuf_add(pkt, NULL, sizeof(gnrc_netif_hdr_t), GNRC_NETTYPE_NETIF);
	if(pkt == NULL) {
		puts("iqueuemac: pktbuf add failed in iqueuemac_send_announce().");
	}
	/* We wouldn't get here if add the NETIF header had failed, so no
		sanity checks needed */
	nethdr_announce = (gnrc_netif_hdr_t*) _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);

	/* Construct NETIF header and initiate address fields */
	gnrc_netif_hdr_init(nethdr_announce, 0, 0);

	/* Send WA as broadcast*/
	nethdr_announce->flags |= GNRC_NETIF_HDR_FLAGS_BROADCAST;

	netopt_enable_t csma_enable;
	csma_enable = use_csma;
	iqueuemac_send(iqueuemac, pkt, csma_enable);
}


void iqueuemac_add_in_cluster_neighbor(iqueuemac_t* iqueuemac, l2_addr_t* addr){

	int i;
	/*** first check whether it has been recorded ***/
	for(i=0;i<IQUEUEMAC_MAX_IN_CLUSTER_NEIGH_INFO_NUM;i++){
		if(iqueuemac->in_cluster_node_list[i].node_addr.len != 0){
			if(_addr_match(&iqueuemac->in_cluster_node_list[i].node_addr, addr)){
				return;
			}
		}
	}

	/*** Then, if not recorded yet, add it into the list ***/
	for(i=0;i<IQUEUEMAC_MAX_IN_CLUSTER_NEIGH_INFO_NUM;i++){
		if(iqueuemac->in_cluster_node_list[i].node_addr.len == 0){
			iqueuemac->in_cluster_node_list[i].node_addr.len = addr->len;
			memcpy(iqueuemac->in_cluster_node_list[i].node_addr.addr,
					addr->addr,
					addr->len);
			return;
		}
	}
}

void iqueuemac_remove_in_cluster_neighbor(iqueuemac_t* iqueuemac, l2_addr_t* addr){

	int i;
	/*** first check whether it has been recorded ***/
	for(i=0;i<IQUEUEMAC_MAX_IN_CLUSTER_NEIGH_INFO_NUM;i++){
		if(_addr_match(&iqueuemac->in_cluster_node_list[i].node_addr, addr)){
			iqueuemac->in_cluster_node_list[i].node_addr.len = 0;
			return;
		}
	}
}

void iqueuemac_device_process_preamble_ack(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt, iqueuemac_packet_info_t* pa_info){

	 iqueuemac_frame_preamble_ack_t* iqueuemac_preamble_ack_hdr;

	 iqueuemac_preamble_ack_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);

	 if(iqueuemac_preamble_ack_hdr == NULL) {
		 puts("iqueuemac_preamble_ack_hdr is null");
		 return;
	 }

	 /**** check if this node has a father router yet, if no, check whether this destination is a router.**/

	 if((iqueuemac->father_router_addr.len == 0)&&(iqueuemac->mac_type == NODE)){
		 /*** this node doesn't has a father router yet ***/

		 /*** check whether the receiver is a router type ***/
		 if(iqueuemac_preamble_ack_hdr->device_type == ROUTER){
			 /*** the first heard router is selected as father router ***/
			 iqueuemac->father_router_addr.len = pa_info->src_addr.len;
			 memcpy(iqueuemac->father_router_addr.addr,
					pa_info->src_addr.addr,
					pa_info->src_addr.len);
		 }
	 }

	 /***** update all the necessary information to marked as a known neighbor ****/
	 iqueuemac->tx.current_neighbour->mac_type = iqueuemac_preamble_ack_hdr->device_type;

	 iqueuemac->tx.current_neighbour->in_same_cluster = false;


	 /*** remember to reduce a bit the phase for locking, since there is a hand-shake procedure before ***/
	 //uint32_t  phase_ticks;
	 /*** adjust the phase of the receiver ***/

	 long int phase_ticks;

	 if((iqueuemac->phase_changed == true)&&(iqueuemac->router_states.router_new_cycle == true)){
		 /* this means that the node is already in a new cycle when doing phase changed.
		  * So, give some compensation for later phase adjust */
		 phase_ticks = _phase_now(iqueuemac) + iqueuemac->backoff_phase_ticks - iqueuemac_preamble_ack_hdr->phase_in_ticks;
		 phase_ticks += (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)/4);
	 }else{
		 phase_ticks = _phase_now(iqueuemac) - iqueuemac_preamble_ack_hdr->phase_in_ticks + (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)/4);
	 }

	 if(phase_ticks < 0) {
		 phase_ticks += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
	 }

	 /* check if phase is too close */
	 long int future_neighbor_phase;
	 if(iqueuemac->phase_changed == true){
		 future_neighbor_phase = phase_ticks - iqueuemac->backoff_phase_ticks;

		 if(future_neighbor_phase < 0) {
			 future_neighbor_phase += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
		 }
	 }else{
		 future_neighbor_phase = phase_ticks;
	 }

	 uint32_t neighbor_phase;
	 neighbor_phase = (uint32_t)future_neighbor_phase;

	 /* if the sender's phase is too close to the receiver */
	 if((RTT_TICKS_TO_US(neighbor_phase) > (IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_MIN_GAP_US)) ||
			 (RTT_TICKS_TO_US(neighbor_phase) < IQUEUEMAC_CP_MIN_GAP_US))
	 {
		 puts("p close");
		 iqueuemac->phase_backoff = true;
		 iqueuemac->backoff_phase_ticks =
				 random_uint32_range(RTT_US_TO_TICKS(IQUEUEMAC_CP_MIN_GAP_US), RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_MIN_GAP_US));
	 }

     /*** move 1/3 CP duration to give some time redundancy for sender the has forward timer-drift!!! ***/
	 //phase_ticks = (phase_ticks + (RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)/3)) % RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);

	 iqueuemac->tx.current_neighbour->cp_phase = (uint32_t)phase_ticks;//_phase_now(iqueuemac); //- RTT_US_TO_TICKS(IQUEUEMAC_WAIT_CP_SECUR_GAP_US); rtt_get_counter();


#if 0
	 if((iqueuemac->father_router_addr.len != 0)&&(_addr_match(&iqueuemac->father_router_addr,&iqueuemac_preamble_ack_hdr->father_router))){
	     //iqueuemac->tx.current_neighbour->in_same_cluster = true;
	     //iqueuemac->tx.current_neighbour->cp_phase = 0;

		 iqueuemac->tx.current_neighbour->in_same_cluster = false;
		 iqueuemac->tx.current_neighbour->cp_phase = rtt_get_counter();

		 puts("iqueuemac: node got phase-locked with father.");

	     /***  add the node type into the in-cluster list if the receiver and the sender share the same father ***/
	     if(iqueuemac_preamble_ack_hdr->device_type == NODE){
	    	 iqueuemac_add_in_cluster_neighbor(iqueuemac, &pa_info->src_addr);
	     }
	     //puts("iqueuemac: get phased-locked, in the same cluster.");
	 }else{//for router type, it will automatically enter here, since father-router are different
		 iqueuemac->tx.current_neighbour->in_same_cluster = false;
		 iqueuemac->tx.current_neighbour->cp_phase = rtt_get_counter();

		 if(iqueuemac_preamble_ack_hdr->device_type == NODE){
			 iqueuemac_remove_in_cluster_neighbor(iqueuemac, &pa_info->src_addr);
	     }
		 //puts("iqueuemac: get phased-locked, not in the same cluster.");
	 }


	 /* if this is the father router, get phase-locked!!!!  */
	 if((_addr_match(&iqueuemac->father_router_addr, &pa_info->src_addr))&&(iqueuemac->mac_type == NODE)){
		 rtt_clear_alarm();

		 uint32_t  phase_ticks;
		 uint32_t alarm;
		 //uint32_t  current_timing_us;

		 /**set the rtt counter to be the same as father router*/
		 phase_ticks = iqueuemac_preamble_ack_hdr->phase_in_ticks;
		 rtt_set_counter(phase_ticks);

		 /** set set next rtt alarm*/
		 if(phase_ticks >= RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US)){
			 alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
			 iqueuemac_set_rtt_alarm(alarm, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_CP);
			 iqueuemac->node_states.in_cp_period = false;
			 //puts("iqueuemac: node got phase-locked with father in sleep.");
		 }else{
			 alarm = RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
			 iqueuemac_set_rtt_alarm(alarm, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP);
			 iqueuemac->node_states.in_cp_period = true;
			 puts("iqueuemac: node got phase-locked with father in CP.");
		 }

	 }
#endif

}

void iqueuemac_packet_process_in_wait_preamble_ack(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {
    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	/* it is very unlikely that we will receive beacon here.  */
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
            	/* Due to non-overlap CP rule, it is very unlikely that we will receive preamble here.
            	 * But, in case it happens, quit this t-2-u for collision avoidance.
            	 * Release all received preamle here to reduce complexity. Only reply preamble in CP.*/
            	gnrc_pktbuf_release(pkt);
            	puts("q-u-1");
            	iqueuemac->quit_current_cycle = true;
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	//puts("iqueuemac: nodes receives a preamble_ack");
            	if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr)){
            		if(_addr_match(&iqueuemac->tx.current_neighbour->l2_addr, &receive_packet_info.src_addr)){
            			iqueuemac->tx.got_preamble_ack = true;

            			iqueuemac_device_process_preamble_ack(iqueuemac, pkt, &receive_packet_info);

            			/**got preamble-ack, flush the rx queue***/
            			gnrc_pktbuf_release(pkt);
            			packet_queue_flush(&iqueuemac->rx.queue);
            			return;
            		}
            	}
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr))
            	{
            		iqueuemac_router_queue_indicator_update(iqueuemac, pkt, &receive_packet_info);

                	if((iqueuemac_check_duplicate(iqueuemac, &receive_packet_info))){
                		gnrc_pktbuf_release(pkt);
                		puts("dup pkt.");
                		return;
                	}

            		iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
            		_dispatch(iqueuemac->rx.dispatch_buffer);
            	}else {/* if the data is not for the node, release it.  */

            		gnrc_pktbuf_release(pkt);
            	}
            }break;

            case FRAMETYPE_BROADCAST:{
            	/* Due to non-overlap CP rule, it is very unlikely that we will receive broadcast here.
            	 * But, in case it happens, quit this t-2-u for collision avoidance.
            	 * Release the broadcast pkt, and receive it in CP, thus to reduce complexity.*/
            	iqueuemac->quit_current_cycle = true;
            	gnrc_pktbuf_release(pkt);
            	puts("q-u-2");
            }break;

            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */

}

int iqueuemac_send_data_packet(iqueuemac_t* iqueuemac, netopt_enable_t csma_enable)
{
	gnrc_pktsnip_t* pkt;
	pkt = iqueuemac->tx.tx_packet;

    if (pkt == NULL) {
        puts("iqsend: pkt was NULL\n");
        return -EINVAL;
    }

	/*** enable auto-ACK ??? ***/

	/* Insert iqueue-mac header above NETIF header */
	iqueuemac_frame_data_t* iqueuemac_data_hdr_pointer;
	iqueuemac_data_hdr_pointer = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);

	if(iqueuemac_data_hdr_pointer == NULL){

		iqueuemac_frame_data_t iqueuemac_data_hdr;
		iqueuemac_data_hdr.header.type = FRAMETYPE_IQUEUE_DATA;
		iqueuemac_data_hdr.queue_indicator = iqueuemac->tx.current_neighbour->queue.length;

		/* save payload pointer */
		gnrc_pktsnip_t* payload = iqueuemac->tx.tx_packet->next;

		pkt->next = gnrc_pktbuf_add(pkt->next, &iqueuemac_data_hdr, sizeof(iqueuemac_data_hdr), GNRC_NETTYPE_IQUEUEMAC);
		if(pkt->next == NULL) {
			puts("iqueuemac: pktbuf add failed in iqueuemac_send_data_packet().");
			/* make append payload after netif header again */
			iqueuemac->tx.tx_packet->next = payload;
			return -ENOBUFS;
		}

	}else{
		/*** update queue-indicator ***/
		iqueuemac_data_hdr_pointer->queue_indicator = iqueuemac->tx.current_neighbour->queue.length;
	}

	gnrc_pktbuf_hold(iqueuemac->tx.tx_packet,1);

	int res;
	res = iqueuemac_send(iqueuemac, iqueuemac->tx.tx_packet, csma_enable);
    if(res < 0){
        /* If res is < 0, then, the old pkt will not be released in send(). so need to release old data once */
        gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
        iqueuemac->tx.tx_packet = NULL;
		puts("iqueuemac: tx-res < 0 in iqueuemac_send_data_packet().");
    }
	return res;

}


bool iqueue_mac_find_next_tx_neighbor(iqueuemac_t* iqueuemac){

    //////////////
    int next = -1;
    //uint32_t phase_check;
    //uint32_t phase_nearest = IQUEUEMAC_PHASE_MAX;

    /*** If current_neighbour is not NULL, means last t-2-r or t-2-u failed, will continue try t-2-r/t-2-u
     * again for the same neighbor, which has not been released in last t-2-r/t-2-u. ***/
    if(iqueuemac->tx.current_neighbour != NULL)
    {
       return true;
    }

    if(iqueuemac->tx.neighbours[0].queue.length > 0){
    	next = 0;
    }else{
    	/*** find the next neighbor ***/
    	uint32_t j;
    	j = iqueuemac->tx.last_tx_neighbor_id + 1;

    	if(j >= IQUEUEMAC_NEIGHBOUR_COUNT) {
    		j= 1;
    	}

    	for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++) {

    		if(iqueuemac->tx.neighbours[j].queue.length > 0) {
    			iqueuemac->tx.last_tx_neighbor_id = j;
    			next = (int)j;
    			break;
    		} else {
    			j ++;
    	    	if(j >= IQUEUEMAC_NEIGHBOUR_COUNT) {
    	    		j= 1;
    	    	}
    		}

#if 0
        	if(iqueuemac->tx.neighbours[i].queue.length > 0) {
            	/* Unknown destinations are initialized with their phase at the end
             	* of the local interval, so known destinations that still wakeup
       	        * in this interval will be preferred. */
        	    phase_check = _ticks_until_phase(iqueuemac, iqueuemac->tx.neighbours[i].cp_phase);

       	        if(phase_check <= phase_nearest) {
        	        next = i;
        	        phase_nearest = phase_check;
        	    }
       	 	}
#endif
    	}
    }
    ////////
    if(next >= 0){
       	gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.neighbours[next].queue));
      	if(pkt != NULL){
       		iqueuemac->tx.tx_packet = pkt;
       		iqueuemac->tx.current_neighbour = &iqueuemac->tx.neighbours[next];
       		iqueuemac->tx.tx_seq = 0;
       		iqueuemac->tx.t2u_retry_contuer = 0;

       		//printf("iqueuemac: the find nearest neighbor is %d. \n", next);
       		return true;
      	}else{
      		return false;
      	}
    }
    return false;
}

/*
bool iqueuemac_check_has_pending_packet(packet_queue_t* q)
{
	gnrc_pktsnip_t* pkt;
	pkt = packet_queue_head(q);

	if(pkt != NULL){
		return true;
	}

	return false;
}*/


void iqueuemac_beacon_process(iqueuemac_t* iqueuemac, gnrc_pktsnip_t* pkt){
	iqueuemac_frame_beacon_t* iqueuemac_beacon_hdr;
	gnrc_pktsnip_t* iqueuemac_snip;

	l2_id_t* id_list;
	uint8_t* slots_list;
	uint8_t schedulelist_size = 0;
	bool got_allocated_slots;
	uint8_t id_position;
	uint8_t slots_position;

	iqueuemac_beacon_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_IQUEUEMAC);

	if(iqueuemac_beacon_hdr == NULL) {
	   puts("iqueuemac_beacon_hdr is null");
	   return;
	}

	schedulelist_size = iqueuemac_beacon_hdr->schedulelist_size;
	iqueuemac->tx.vtdma_para.sub_channel_seq = iqueuemac_beacon_hdr->sub_channel_seq;

	if(schedulelist_size == 0){
		iqueuemac->tx.vtdma_para.slots_num = 0;
		iqueuemac->tx.vtdma_para.slots_position = 0;
		return;
	}

	/**** take the ID-list out ****/
	iqueuemac_snip = gnrc_pktbuf_mark(pkt, schedulelist_size * sizeof(l2_id_t), GNRC_NETTYPE_IQUEUEMAC);
	id_list = iqueuemac_snip->data;
	/**** take the slots-list out ****/
	slots_list = pkt->data;

	/**** check whether has been allocated slots ****/
	int i=0;
	got_allocated_slots = false;
	id_position = 0;

	for(i=0;i<schedulelist_size;i++){
		if(memcmp(iqueuemac->own_addr.addr, id_list[i].addr, iqueuemac->own_addr.len) == 0){
			got_allocated_slots = true;
			id_position = i;
		}
	}

	/**** find the slots number and position ****/
	if(got_allocated_slots == true){
		iqueuemac->tx.vtdma_para.slots_num = slots_list[id_position];

		slots_position = 0;
		for(i=0;i<id_position;i++){
			slots_position += slots_list[i];
		}
		iqueuemac->tx.vtdma_para.slots_position = slots_position;

		//printf("iqueuemac: the allocated slots-num is %d, id-position is %d .\n", iqueuemac->tx.vtdma_para.slots_num, id_position);
	}else{
		iqueuemac->tx.vtdma_para.slots_num = 0;
		iqueuemac->tx.vtdma_para.slots_position = 0;
	}
}

/****** check whether this function can be merged with router-wait-beacon-packet-process!!! ******/
void iqueuemac_wait_beacon_packet_process(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	if(_addr_match(&iqueuemac->tx.current_neighbour->l2_addr, &receive_packet_info.src_addr)){
            		iqueuemac->tx.vtdma_para.get_beacon = true;
            		iqueuemac_beacon_process(iqueuemac, pkt);
            	}
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
            	/* Release preamble pkt no matter the preamble is for it or not, and quit the t-2-r. */
            	iqueuemac->quit_current_cycle = true;
        	    gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	// should we quit this period also??!
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	/* It is unlikely that we will received a data for us here. This means the nodes' CP is close with its
            	 * destination's. */
            	if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr))
            	{
            		iqueuemac_router_queue_indicator_update(iqueuemac, pkt, &receive_packet_info);

                	if((iqueuemac_check_duplicate(iqueuemac, &receive_packet_info))){
                		gnrc_pktbuf_release(pkt);
                		puts("dup pkt.");
                		return;
                	}

            		iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
            		_dispatch(iqueuemac->rx.dispatch_buffer);
            	}else/* if the data is not for the node, release it.  */
            	{
            		/* it is very unlikely that we will receive not-intended data here, since CP will not overlape! */
            		gnrc_pktbuf_release(pkt);
            	}
            }break;

            case FRAMETYPE_BROADCAST:{
            	iqueuemac->quit_current_cycle = true;
            	gnrc_pktbuf_release(pkt);
            }break;

            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */
}

void iqueue_node_cp_receive_packet_process(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
        	    if(_addr_match(&iqueuemac->own_addr, &receive_packet_info.dst_addr)){
        	  	  iqueue_send_preamble_ack(iqueuemac, &receive_packet_info);
        	    }else{
        		  //this means that there is a long preamble period, so quit this cycle and go to sleep.
        		  iqueuemac->quit_current_cycle = true;
        	    }
        	    gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	gnrc_pktbuf_release(pkt);

            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	//printf("%lu. \n", RTT_TICKS_TO_US(_phase_now(iqueuemac)));
            	//iqueuemac_router_queue_indicator_update(iqueuemac, pkt, &receive_packet_info);
        	    iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
            	//gnrc_pktbuf_release(pkt);


        	    //puts("iqueuemac: node receives a data !!");
            }break;

            case FRAMETYPE_BROADCAST:{
               	iqueuemac->quit_current_cycle = true;
                iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);
            }break;

            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */
}



void iqueuemac_router_vtdma_receive_packet_process(iqueuemac_t* iqueuemac){
	gnrc_pktsnip_t* pkt;

	iqueuemac_packet_info_t receive_packet_info;

    while( (pkt = packet_queue_pop(&iqueuemac->rx.queue)) != NULL ) {

    	/* parse the packet */
    	int res = _parse_packet(pkt, &receive_packet_info);
    	if(res != 0) {
            //LOG_DEBUG("Packet could not be parsed: %i\n", ret);
            gnrc_pktbuf_release(pkt);
            continue;
        }

    	switch(receive_packet_info.header->type){
            case FRAMETYPE_BEACON:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE:{
        	    gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_PREAMBLE_ACK:{
            	gnrc_pktbuf_release(pkt);
            }break;

            case FRAMETYPE_IQUEUE_DATA:{
            	iqueuemac_router_queue_indicator_update(iqueuemac, pkt, &receive_packet_info);

            	if((iqueuemac_check_duplicate(iqueuemac, &receive_packet_info))){
            		gnrc_pktbuf_release(pkt);
            		puts("dup pkt.");
            		return;
            	}
        	    iqueue_push_packet_to_dispatch_queue(iqueuemac->rx.dispatch_buffer, pkt, &receive_packet_info, iqueuemac);

        	    _dispatch(iqueuemac->rx.dispatch_buffer);
        	    //puts("iqueuemac: router receives a data in vtdma!!");
            }break;

            default:gnrc_pktbuf_release(pkt);break;
  	    }

    }/* end of while loop */
}

void iqueuemac_figure_tx_neighbor_phase(iqueuemac_t* iqueuemac){

	if(iqueuemac->phase_changed == true){

		iqueuemac->phase_changed = false;

    	for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++){
    		if(iqueuemac->tx.neighbours[i].mac_type == ROUTER){
    			long int tmp = iqueuemac->tx.neighbours[i].cp_phase - iqueuemac->backoff_phase_ticks;
    		    if(tmp < 0) {
    		        tmp += RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);

    		        /* update the neighbor's cur_pub_channel if tmp < 0 */
        	        if(iqueuemac->tx.neighbours[i].cur_pub_channel == iqueuemac->pub_channel_1) {
        		        iqueuemac->tx.neighbours[i].cur_pub_channel = iqueuemac->pub_channel_2;
        	        }else{
        		        iqueuemac->tx.neighbours[i].cur_pub_channel = iqueuemac->pub_channel_1;
        	        }
    		    }
    		    iqueuemac->tx.neighbours[i].cp_phase = (uint32_t)tmp;
    		}
    	}
	}
}

void _dispatch(gnrc_pktsnip_t** buffer)
{
    assert(buffer != NULL);

    for(unsigned i = 0; i < IQUEUEMAC_DISPATCH_BUFFER_SIZE; i++) {
        if(buffer[i]) {

            /* save pointer to netif header */
            gnrc_pktsnip_t* netif = buffer[i]->next->next;

            /* remove iqueuemac header */
            buffer[i]->next->next = NULL;
            gnrc_pktbuf_release(buffer[i]->next);

            /* make append netif header after payload again */
            buffer[i]->next = netif;

            if (!gnrc_netapi_dispatch_receive(buffer[i]->type, GNRC_NETREG_DEMUX_CTX_ALL, buffer[i])) {
                DEBUG("Unable to forward packet of type %i\n", buffer[i]->type);
                gnrc_pktbuf_release(buffer[i]);
                puts("dispatch pkt fail, drop it");
            }
            buffer[i] = NULL;
        }
    }
}

void update_neighbor_pubchan(iqueuemac_t* iqueuemac)
{
	if(iqueuemac->cur_pub_channel == iqueuemac->pub_channel_1) {
		iqueuemac->cur_pub_channel = iqueuemac->pub_channel_2;
	}else{
		iqueuemac->cur_pub_channel = iqueuemac->pub_channel_1;
	}

	/* update tx-nighbors' current channel */
	for(int i = 1; i < IQUEUEMAC_NEIGHBOUR_COUNT; i++){
		if(iqueuemac->tx.neighbours[i].mac_type == ROUTER){
			/* switch public channel */
			if(iqueuemac->tx.neighbours[i].cur_pub_channel == iqueuemac->pub_channel_1) {
				iqueuemac->tx.neighbours[i].cur_pub_channel = iqueuemac->pub_channel_2;
			}else{
				iqueuemac->tx.neighbours[i].cur_pub_channel = iqueuemac->pub_channel_1;
			}
		}
	}
}



/******************************************************************************/



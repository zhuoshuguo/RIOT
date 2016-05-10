/*
 * Copyright (C) 2016 Shuguo Zhuo
 *               
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 * @ingroup     net_iqueue_mac
 * @file
 * @brief       implementation of iqueue_mac
 *
 * @author      Shuguo Zhuo <shuguo.zhuo@iniria.fr>
 * @}
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <kernel_types.h>
#include <lpm.h>
#include "msg.h"
#include "thread.h"
#include <timex.h>
#include <periph/rtt.h>
#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev2.h"
#include "net/gnrc/netdev2.h"
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include <net/gnrc/iqueue_mac/packet_queue.h>

#include "include/iqueuemac_internal.h"
#include "include/iqueuemac_types.h"
#include "include/timeout.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif

#define NETDEV2_NETAPI_MSG_QUEUE_SIZE 8


static iqueuemac_t iqueuemac;

void iqueuemac_init(iqueuemac_t* iqueuemac)
{
	if(iqueuemac->mac_type == ROUTER)
	{
		iqueuemac->router_states.router_basic_state = R_LISTENNING;
		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_INIT;

		iqueuemac->router_states.router_new_cycle = false;

		iqueuemac->rx.router_vtdma_mana.sub_channel_seq = 26;

	}else{
		iqueuemac->node_states.node_basic_state = N_LISTENNING;
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_INIT;
		iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;
		iqueuemac->node_states.node_t2u_state = N_T2U_SEND_PREAMBLE_INIT;
		iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_CP_INIT;

		iqueuemac->father_router_addr.len = 0;
		iqueuemac->father_router_addr.addr[0] = 0;
		iqueuemac->father_router_addr.addr[1] = 0;

		iqueuemac->node_states.in_cp_period = false;
	}

	iqueuemac->own_addr.len = iqueuemac->netdev->dev->driver->get(iqueuemac->netdev->dev, NETOPT_ADDRESS, iqueuemac->own_addr.addr, sizeof(iqueuemac->own_addr.addr));
	//assert(lwmac.l2_addr.len > 0);

	printf("shuguo: iqueuemac's own addrs is: %d, %d . \n ", iqueuemac->own_addr.addr[1], iqueuemac->own_addr.addr[0]);

	/* Enable RX- and TX-started interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    //iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    /* Initialize receive packet queue */
    packet_queue_init(&(iqueuemac->rx.queue),
    		          iqueuemac->rx._queue_nodes,
                      (sizeof(iqueuemac->rx._queue_nodes) / sizeof(packet_queue_node_t)));

    /* Reset all timeouts just to be sure */
    iqueuemac_reset_timeouts(iqueuemac);

    iqueuemac->packet_received = false;
	iqueuemac->need_update = false;
	iqueuemac->duty_cycle_started = false;
	iqueuemac->quit_current_cycle = false;

}

static void rtt_cb(void* arg)
{
    msg_t msg;
    msg.content.value = ((uint32_t) arg ) & 0xffff;
    msg.type = IQUEUEMAC_EVENT_RTT_TYPE;
    msg_send(&msg, iqueuemac.pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}


void iqueuemac_set_rtt_alarm(uint32_t alarm, void *arg){

	rtt_set_alarm(alarm, rtt_cb, arg);
}

void rtt_handler(uint32_t event)
{
    uint32_t alarm;
    switch(event & 0xffff)
    {
      case IQUEUEMAC_EVENT_RTT_R_ENTER_CP:{
        iqueuemac.router_state = R_CP;
        alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
        rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_ENTER_BEACON);
      }break;
      
      case IQUEUEMAC_EVENT_RTT_ENTER_BEACON:{
        iqueuemac.router_state = R_BEACON;
        alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
        rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_ENTER_VTDMA);
      }break;
      
      case IQUEUEMAC_EVENT_RTT_ENTER_VTDMA:{
    	iqueuemac.router_state = R_VTDMA;

    	puts("Shuguo: setting vTDMA period timeout!");

    	//iqueuemac_set_timeout(&iqueuemac, TIMEOUT_VTDMA, IQUEUEMAC_VTDMA_DURATION_US);
    	//iqueuemac_set_timeout(&iqueuemac, TIMEOUT_VTDMA_LONG, IQUEUEMAC_VTDMA_LONG_DURATION_US);
    	//iqueuemac_set_timeout(&iqueuemac, TIMEOUT_VTDMA_LONG_LONG, IQUEUEMAC_VTDMA_LONG_LONG_DURATION_US);

        //alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
        //rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_ENTER_SLEEP);
      }break;
      
      case IQUEUEMAC_EVENT_RTT_ENTER_SLEEP:{
        iqueuemac.router_state = R_SLEEPING;
        alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_SLEEP_DURATION_US);
        rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_ENTER_CP);
      }break;      

      /*******************************Router RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE:{

          if(iqueuemac.duty_cycle_started == false){
        	  iqueuemac.duty_cycle_started = true;
          }else{
        	  iqueuemac.router_states.router_new_cycle = true;
          }

          /// Shuguo: 以后每次进这里把RTT的计时器清零？！ 方便于管理和计算！！？？
          rtt_set_counter(0);

          //alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);

          iqueuemac.need_update = true;

      }break;

      /*******************************Node RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_N_ENTER_CP:{

    	  rtt_set_counter(0);

    	  /// Shuguo: 以后每次进这里把RTT的计时器清零？！ 方便于管理和计算！！？？
    	  //alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	  alarm = RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	  rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP);

    	  if(iqueuemac.duty_cycle_started == false){
    	     iqueuemac.duty_cycle_started = true;
    	  }

    	  iqueuemac.node_states.in_cp_period = true;

    	  iqueuemac.need_update = true;
      }break;

      case IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP:{
    	  //alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_CP_DURATION_US);
    	  alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
    	  rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_CP);
    	  iqueuemac.node_states.in_cp_period = false;

    	  iqueuemac.need_update = true;
      }break;

      /********************************************************/
      case IQUEUEMAC_EVENT_RTT_START:{
    	  puts("shuguo: starting duty cycling.");
    	  if(iqueuemac.mac_type == ROUTER)
    	  {
    	     alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	     rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);
    	  }else{
     	     alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
     	     rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_CP);
    	  }
      }break;

      default: break;

      }

}

/******************************router state machinies**********************************/
void iqueue_mac_router_update_old(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_state)
	{
	  case R_CP:{
		  puts("Shuguo: we are now in CP period!");
	  }break;

	  case R_BEACON:{
		  puts("Shuguo: we are now in BEACON period!");

		  if(iqueuemac->tx.neighbours[1].queue.length>0)
		  {
			  gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.neighbours[1].queue));
			  if(pkt != NULL){
				  //iqueuemac_send(iqueuemac, pkt, true);
			  	puts("Shuguo: we are now sending data in beacon period!");
			  }
			  //printf("Shuguo: neighbor-1's queue-length is %d .\n", (int)iqueuemac.neighbours[1].queue.length);
		  }

		  /*
		  gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac.iqueue_mac_tx_queue));

		  if(pkt != NULL){  txtsnd 4 1234 1111
		    gnrc_netdev2->send(gnrc_netdev2, pkt);
		    puts("Shuguo: we are now sending data in beacon period!");
		  }else{
		    puts("Shuguo: we are now in beacon period with no data to send!");
		  }
		  printf("Shuguo: the current queue-length is %d .\n", (int)iqueuemac.iqueue_mac_tx_queue.length);
          */
	  }break;

	  case R_VTDMA:{
		  if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_VTDMA)){
			 // iqueuemac_clear_timeout(iqueuemac, TIMEOUT_VTDMA);
			  puts("Shuguo: vTDMA timeout!!  ");

			  uint32_t alarm;
			  alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
			  rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_ENTER_SLEEP);
		  }

		  /*
		  if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_VTDMA_LONG)){
			  //puts("Shuguo: vTDMA_LONG TIMEOUT!!!");
		  }

		  if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_VTDMA_LONG_LONG)){
		 	  puts("Shuguo: vTDMA_LONG_LONG TIMEOUT!!!");
		  }*/

	  }break;

	  case R_SLEEPING:{
		  puts("Shuguo: we are now in SLEEP period!");
	  }break;
	  default: break;

	}

}

/******************new router state machines*****/

void iqueue_mac_router_listen_cp_init(iqueuemac_t* iqueuemac){

	//puts("Shuguo: router is now entering CP");

	iqueuemac->router_states.router_new_cycle = false;

	iqueuemac_trun_on_radio(iqueuemac);
	iqueuemac->packet_received = false;

	/******set cp timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);

	iqueuemac->router_states.router_listen_state = R_LISTEN_CP_LISTEN;
	iqueuemac->need_update = true;

	packet_queue_flush(&iqueuemac->rx.queue);
}

void iqueue_mac_router_listen_cp_listen(iqueuemac_t* iqueuemac){
/* In CP, a router can receive preamble, beacon, data packet, */

    if(iqueuemac->packet_received == true){
    	iqueuemac->packet_received = false;
    	iqueue_router_cp_receive_packet_process(iqueuemac);
    }

    /****** insert codes here for handling quit this cycle when receiving unexpected preamble***/
    // if(iqueuemac->quit_current_cycle == true)
    // clear timeout and switch to sleep period

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_CP_END)){
		//puts("Shuguo: Router CP ends!!");
		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_router_cp_end(iqueuemac_t* iqueuemac){

	packet_queue_flush(&iqueuemac->rx.queue);
	iqueuemac->router_states.router_listen_state = R_LISTEN_SEND_BEACON;
	iqueuemac->need_update = true;
}

void iqueue_mac_router_send_beacon(iqueuemac_t* iqueuemac){

	/****** assemble and send the beacon ******/
	iqueuemac_assemble_and_send_beacon(iqueuemac);

	//puts("Shuguo: router is now sending the beacon!!!");
	/****** router switch to sleep period or vTDMA period ******/
	if(iqueuemac->rx.router_vtdma_mana.total_slots_num > 0){
		iqueuemac->router_states.router_listen_state = R_LISTEN_VTDMA_INIT;
		iqueuemac->need_update = true;
	}else{
	    iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
	    iqueuemac->need_update = true;
	}
}

void iqueue_mac_router_vtdma_init(iqueuemac_t* iqueuemac){

	/*** switch the radio to the subchannel ***/

	/*** set the vTDMA period timeout!!! ***/
	uint32_t vtdma_duration;
	vtdma_duration = iqueuemac->rx.router_vtdma_mana.total_slots_num * IQUEUEMAC_VTDMA_SLOT_SIZE_US;

	iqueuemac_set_timeout(iqueuemac, TIMEOUT_VTDMA, vtdma_duration);

	iqueuemac->router_states.router_listen_state = R_LISTEN_VTDMA;
	iqueuemac->need_update = true;

}

void iqueue_mac_router_vtdma(iqueuemac_t* iqueuemac){

	if(iqueuemac->packet_received == true){
	    iqueuemac->packet_received = false;

	    /*** check whether this packet-process func. can be merged with cp-packet-process func. due to similar functionalities!!! ***/
	    //iqueue_router_cp_receive_packet_process(iqueuemac);
	    iqueuemac_router_vtdma_receive_packet_process(iqueuemac);
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_VTDMA)){
		//puts("Shuguo: Router vTDMA ends!!");
		iqueuemac->router_states.router_listen_state = R_LISTEN_VTDMA_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_router_vtdma_end(iqueuemac_t* iqueuemac){

	/*** switch the radio to the public-channel!!! ***/

	/*** ensure that the channel-switching is finished before go to sleep to turn it off !!! ***/

	iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
	iqueuemac->need_update = true;

}

void iqueue_mac_router_sleep_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_off_radio(iqueuemac);
	iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING;
	iqueuemac->need_update = true;

	//puts("Shuguo: router is now entering sleeping period");
}

void iqueue_mac_router_sleep(iqueuemac_t* iqueuemac){

	if(iqueuemac->router_states.router_new_cycle == true){

		iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_router_sleep_end(iqueuemac_t* iqueuemac){

	iqueuemac->router_states.router_listen_state = R_LISTEN_CP_INIT;
	iqueuemac->need_update = true;
}

void iqueue_mac_router_listen_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_states.router_listen_state)
   {
	case R_LISTEN_CP_INIT: iqueue_mac_router_listen_cp_init(iqueuemac); break;
	case R_LISTEN_CP_LISTEN: iqueue_mac_router_listen_cp_listen(iqueuemac); break;
	case R_LISTEN_CP_END: iqueue_mac_node_router_cp_end(iqueuemac); break;
	//case R_LISTEN_CREATE_BEACON: iqueue_mac_router_create_beacon(iqueuemac); break;
	case R_LISTEN_SEND_BEACON: iqueue_mac_router_send_beacon(iqueuemac); break;
	case R_LISTEN_VTDMA_INIT: iqueue_mac_router_vtdma_init(iqueuemac); break;
	case R_LISTEN_VTDMA: iqueue_mac_router_vtdma(iqueuemac); break;
	case R_LISTEN_VTDMA_END: iqueue_mac_router_vtdma_end(iqueuemac); break;
	case R_LISTEN_SLEEPING_INIT: iqueue_mac_router_sleep_init(iqueuemac); break;
	case R_LISTEN_SLEEPING: iqueue_mac_router_sleep(iqueuemac); break;
	case R_LISTEN_SLEEPING_END: iqueue_mac_router_sleep_end(iqueuemac); break;
	default: break;
   }
}

void iqueue_mac_router_transmit_update(iqueuemac_t* iqueuemac){
;
}

void iqueue_mac_router_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_states.router_basic_state)
   {
	case R_LISTENNING: iqueue_mac_router_listen_update(iqueuemac); break;
	case R_TRANSMITTING: iqueue_mac_router_transmit_update(iqueuemac); break;
	default: break;
   }
}

/******************************node state machinies**********************************/
void iqueue_mac_node_listen_cp_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_on_radio(iqueuemac);

	//iqueue_mac_send_preamble(iqueuemac, NETOPT_ENABLE);

	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
	iqueuemac->need_update = true;

	packet_queue_flush(&iqueuemac->rx.queue);
	//puts("Shuguo: node is now entering CP");
}

void iqueue_mac_node_listen_cp_listen(iqueuemac_t* iqueuemac){

	/**** add packet received process function here to flush the rx_queue, otherwise may overflow!!!  ****/
	if(iqueuemac->packet_received == true){
	    	iqueuemac->packet_received = false;
	    	iqueue_node_cp_receive_packet_process(iqueuemac);
	}

	if(iqueuemac->node_states.in_cp_period == false)
	{
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_listen_cp_end(iqueuemac_t* iqueuemac){

	packet_queue_flush(&iqueuemac->rx.queue);

	if(iqueue_mac_find_next_tx_neighbor(iqueuemac)){
		iqueuemac->node_states.node_basic_state = N_TRANSMITTING;
		switch(iqueuemac->tx.current_neighbour->mac_type){
		  case UNKNOWN: iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;break;
		  case ROUTER: iqueuemac->node_states.node_trans_state = N_TRANS_TO_ROUTER;break;
		  case NODE: iqueuemac->node_states.node_trans_state = N_TRANS_TO_NODE;break;
		  default:break;
		}
		iqueuemac->need_update = true;
	}else{
	    iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING_INIT;
	    iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_sleep_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_off_radio(iqueuemac);
	iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
	iqueuemac->need_update = true;

	//puts("Shuguo: node is now entering sleeping period");

}

void iqueue_mac_node_sleep(iqueuemac_t* iqueuemac){

	if(iqueuemac->node_states.in_cp_period == true){
		iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_sleep_end(iqueuemac_t* iqueuemac){

	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_INIT;
	iqueuemac->need_update = true;
}

void iqueue_mac_node_listen_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_listen_state)
   {
	case N_LISTEN_CP_INIT: iqueue_mac_node_listen_cp_init(iqueuemac); break;
	case N_LISTEN_CP_LISTEN: iqueue_mac_node_listen_cp_listen(iqueuemac); break;
	case N_LISTEN_CP_END: iqueue_mac_node_listen_cp_end(iqueuemac); break;
	case N_LISTEN_SLEEPING_INIT: iqueue_mac_node_sleep_init(iqueuemac); break;
	case N_LISTEN_SLEEPING: iqueue_mac_node_sleep(iqueuemac); break;
	case N_LISTEN_SLEEPING_END: iqueue_mac_node_sleep_end(iqueuemac); break;
	default: break;
   }
}
/**************************transmission handling of simple node *************************************/

void iqueue_mac_node_t2u_send_preamble_init(iqueuemac_t* iqueuemac)
{
	//iqueuemac_trun_on_radio(iqueuemac);
	iqueuemac_trun_on_radio(iqueuemac);
	iqueuemac->packet_received = false;
	iqueuemac->tx.preamble_sent = 0;
	iqueuemac->tx.got_preamble_ack = false;

	iqueuemac->node_states.node_t2u_state = N_T2U_SEND_PREAMBLE;
	iqueuemac->need_update = true;

	packet_queue_flush(&iqueuemac->rx.queue);
}

void iqueue_mac_node_t2u_send_preamble(iqueuemac_t* iqueuemac)
{

	if(iqueuemac->tx.preamble_sent == 0){
		iqueue_mac_send_preamble(iqueuemac, NETOPT_ENABLE);
		iqueuemac_set_timeout(iqueuemac, TIMEOUT_PREAMBLE_DURATION, IQUEUEMAC_SUPERFRAME_DURATION_US); //IQUEUEMAC_PREAMBLE_DURATION_US
	}else{
		iqueue_mac_send_preamble(iqueuemac, NETOPT_DISABLE);
	}

	//iqueuemac_trun_on_radio(iqueuemac);

	iqueuemac->tx.preamble_sent ++;

	/******set preamble timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_PREAMBLE, IQUEUEMAC_PREAMBLE_INTERVAL_US);

	iqueuemac->node_states.node_t2u_state = N_T2U_WAIT_PREAMBLE_ACK;
	iqueuemac->need_update = false;
}

void iqueue_mac_node_t2u_wait_preamble_ack(iqueuemac_t* iqueuemac)
{
	//puts("shuguo: node wait preamble ack");

	if(iqueuemac->packet_received == true){
    	iqueuemac->packet_received = false;
    	iqueuemac_packet_process_in_wait_preamble_ack(iqueuemac);
    }

    /****** insert codes here for handling quit this cycle when receiving unexpected preamble***/
    // if(iqueuemac->quit_current_cycle == true)
    // clear timeout and switch to sleep period

	if(iqueuemac->tx.got_preamble_ack == true){
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE_DURATION);
		iqueuemac->node_states.node_t2u_state = N_T2U_SEND_DATA;
		iqueuemac->need_update = true;
		return;
	}
/*
	if(iqueuemac->rx_started == true){
		iqueuemac->need_update = false;
		return;
	}*/

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_PREAMBLE_DURATION)){
		iqueuemac->node_states.node_t2u_state = N_T2U_END;
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac->need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_PREAMBLE)){
		iqueuemac->node_states.node_t2u_state = N_T2U_SEND_PREAMBLE;
		iqueuemac->need_update = true;
		return;
	}
}

void iqueue_mac_node_t2u_send_data(iqueuemac_t* iqueuemac)
{
	iqueuemac_send_data_packet(iqueuemac, NETOPT_DISABLE);

	iqueuemac->tx.tx_packet = NULL;

	iqueuemac->node_states.node_t2u_state = N_T2U_END;
	iqueuemac->need_update = true;


	/**** add a "wait-for-tx feedback" period to wait for the ending of the tx operation ***/
}

void iqueue_mac_node_t2u_end(iqueuemac_t* iqueuemac)
{
 /***** !!!!!!!!!!!!!!!!!!!!!!!!!!! *****************/
	/* this line is very important! only switch to sleep after packet transmission has a feedback!
	namely, after transmission is finished. */
	if(iqueuemac->tx.tx_finished == false){
		return;
	}

	puts("shuguo: node T2U ends");
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE_DURATION);

	if(iqueuemac->tx.tx_packet){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}
	iqueuemac->tx.current_neighbour = NULL;

	iqueuemac->node_states.node_t2u_state = N_T2U_SEND_PREAMBLE_INIT;


	iqueuemac->node_states.node_basic_state = N_LISTENNING;
	/*********** judge and update the states before switch back to CP listening period   ***********/
	if(iqueuemac->node_states.in_cp_period == true){
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
		//puts("Shuguo: node is in t2u send preamble-end and switch to listen's CP");
	}else{
		iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
		iqueuemac_trun_off_radio(iqueuemac);
		//puts("Shuguo: node is in t2u send preamble-end and switch to listen's sleep");
	}

	iqueuemac->need_update = true;
}


void iqueue_mac_node_t2u_update(iqueuemac_t* iqueuemac)
{
	switch(iqueuemac->node_states.node_t2u_state)
	{
	 case N_T2U_SEND_PREAMBLE_INIT: iqueue_mac_node_t2u_send_preamble_init(iqueuemac);break;
     case N_T2U_SEND_PREAMBLE: iqueue_mac_node_t2u_send_preamble(iqueuemac); break;
	 case N_T2U_WAIT_PREAMBLE_ACK: iqueue_mac_node_t2u_wait_preamble_ack(iqueuemac); break;
	 case N_T2U_SEND_DATA: iqueue_mac_node_t2u_send_data(iqueuemac); break;
	 case N_T2U_END: iqueue_mac_node_t2u_end(iqueuemac);break;
	 default: break;
	}
}

/************************** transmission to router handling of simple node *************************/
void iqueue_mac_node_t2r_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_off_radio(iqueuemac);

	if(iqueuemac->tx.current_neighbour->in_same_cluster == false){
		// set timer for the targetted router!
		;
	}

	iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_CP;
	iqueuemac->need_update = true;
}

void iqueue_mac_node_t2r_wait_cp(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.current_neighbour->in_same_cluster == true){
		/***** wait until it is in the cluster's CP ****/
		if(iqueuemac->node_states.in_cp_period == true){
			iqueuemac_trun_on_radio(iqueuemac);
			iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_IN_CP;
			iqueuemac->need_update = true;
		}

	}else{
		;////if(is_timerout_expired())
	}

}

void iqueue_mac_node_t2r_trans_in_cp(iqueuemac_t* iqueuemac){

	/******Use CSMA here, and send_packet() will release the pkt itself !!!!******/
	iqueuemac_send_data_packet(iqueuemac, NETOPT_ENABLE);

	iqueuemac->tx.tx_packet = NULL;

	iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_CPTRANS_FEEDBACK;
	iqueuemac->need_update = true;

}

void iqueue_mac_node_t2r_wait_cp_transfeedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/
		if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_SUCCESS){
			puts("Shuguo: node success sends a data to father router!!");
		}

		if(iqueuemac->tx.current_neighbour->queue.length > 0){
			iqueuemac->tx.vtdma_para.get_beacon = false;
			iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_BEACON, IQUEUEMAC_SUPERFRAME_DURATION_US);
			// need to flush the rx-queue ??
			packet_queue_flush(&iqueuemac->rx.queue);

			iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_BEACON;
			iqueuemac->need_update = true;
		}else{
			iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
			iqueuemac->need_update = true;
		}

		//iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
		//iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_t2r_wait_beacon(iqueuemac_t* iqueuemac){

    if(iqueuemac->packet_received == true){
    	iqueuemac->packet_received = false;
    	iqueuemac_node_wait_beacon_packet_process(iqueuemac);
    }

    if(iqueuemac->tx.vtdma_para.get_beacon == true){

    	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_BEACON);

    	if(iqueuemac->tx.vtdma_para.slots_num > 0){

    		/*** switch the radio to the sub-channel ***/
    		//iqueuemac_switch_channel(iqueuemac, sub_channel);

    		if(iqueuemac->tx.vtdma_para.slots_position > 0){
    			/*** wait for the finish of switching channel !!! and then turn off the radio to save power ***/
    			iqueuemac_trun_off_radio(iqueuemac);

    			uint32_t wait_slots_duration;
    			wait_slots_duration = iqueuemac->tx.vtdma_para.slots_position * IQUEUEMAC_VTDMA_SLOT_SIZE_US;
    			iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_OWN_SLOTS, wait_slots_duration);

    			iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_OWN_SLOTS;
    			iqueuemac->need_update = true;
    		}else{// be the first sender in vtdma
    			iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_IN_VTDMA;
    			iqueuemac->need_update = true;
    		}
    	}else{
    		iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
    		iqueuemac->need_update = true;
    	}
    	/*
    	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_BEACON);
    	iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
    	iqueuemac->need_update = true;
    	*/

    }

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_BEACON)){
		puts("Shuguo: No beacon.");
		iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_t2r_wait_own_slots(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_OWN_SLOTS)){
		iqueuemac_trun_on_radio(iqueuemac);
		iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_IN_VTDMA;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_t2r_trans_in_slots(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.vtdma_para.slots_num > 0){

		/**** Delete the pkt no matter the transmission is success or not !!! ****/
		gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.current_neighbour->queue));

		/**** Or, only delete the pkt when the feedback shows good !!! ****/
		//gnrc_pktsnip_t *pkt = packet_queue_head(&(iqueuemac->tx.current_neighbour.queue));

		if(pkt != NULL){
			iqueuemac->tx.tx_packet = pkt;
			/******disable CSMA here, and send_packet() will release the pkt itself !!!!******/
			iqueuemac_send_data_packet(iqueuemac, NETOPT_DISABLE);
			iqueuemac->tx.vtdma_para.slots_num --;
			iqueuemac->tx.tx_packet = NULL;

			iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_VTDMATRANS_FEEDBACK;
			iqueuemac->need_update = true;
		}
	}else{/*** here means the slots have been used up !!! ***/
		iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
		iqueuemac->need_update = true;
	}
}

void iqueue_mac_node_t2r_wait_vtdma_transfeedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/
		if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_SUCCESS){
			puts("Shuguo: node success sends a data to father router in vtdma !!");

			/****  if use packt_head previously ****/
			//gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.current_neighbour.queue));
			//gnrc_pktbuf_release(pkt);
		}

		if(iqueuemac->tx.vtdma_para.slots_num > 0){
			iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_IN_VTDMA;
			iqueuemac->need_update = true;
		}else{
			iqueuemac->node_states.node_t2r_state = N_T2R_TRANS_END;
			iqueuemac->need_update = true;
		}
	}
}

void iqueue_mac_node_t2r_end(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_packet){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}
	iqueuemac->tx.current_neighbour = NULL;

	/*** clear all timeouts ***/
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_BEACON);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_OWN_SLOTS);

	iqueuemac->node_states.node_t2r_state = N_T2R_WAIT_CP_INIT;

	iqueuemac->node_states.node_basic_state = N_LISTENNING;
	/*********** judge and update the states before switch back to CP listening period   ***********/
	if(iqueuemac->node_states.in_cp_period == true){
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
		//puts("Shuguo: node is in t2u send preamble-end and switch to listen's CP");
	}else{
		iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
		iqueuemac_trun_off_radio(iqueuemac);
		//puts("Shuguo: node is in t2u send preamble-end and switch to listen's sleep");
	}

	iqueuemac->need_update = true;

}


void iqueue_mac_node_t2r_update(iqueuemac_t* iqueuemac)
{
	switch(iqueuemac->node_states.node_t2r_state)
	{
	 case N_T2R_WAIT_CP_INIT: iqueue_mac_node_t2r_init(iqueuemac);break;
     case N_T2R_WAIT_CP: iqueue_mac_node_t2r_wait_cp(iqueuemac); break;
	 case N_T2R_TRANS_IN_CP: iqueue_mac_node_t2r_trans_in_cp(iqueuemac); break;
	 case N_T2R_WAIT_CPTRANS_FEEDBACK: iqueue_mac_node_t2r_wait_cp_transfeedback(iqueuemac); break;
	 case N_T2R_WAIT_BEACON: iqueue_mac_node_t2r_wait_beacon(iqueuemac);break;
	 case N_T2R_WAIT_OWN_SLOTS:iqueue_mac_node_t2r_wait_own_slots(iqueuemac); break;
	 case N_T2R_TRANS_IN_VTDMA:iqueue_mac_node_t2r_trans_in_slots(iqueuemac); break;
	 case N_T2R_WAIT_VTDMATRANS_FEEDBACK:iqueue_mac_node_t2r_wait_vtdma_transfeedback(iqueuemac);break;
	 case N_T2R_TRANS_END:iqueue_mac_node_t2r_end(iqueuemac);break;
	 default: break;
	}
}


void iqueue_mac_node_transmit_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_trans_state)
   {
	case N_TRANS_TO_UNKOWN: iqueue_mac_node_t2u_update(iqueuemac); break;
	case N_TRANS_TO_ROUTER: iqueue_mac_node_t2r_update(iqueuemac); break;
	//case N_TRANS_TO_NODE: iqueue_mac_node_t2n_update(iqueuemac); break;
	default: break;
   }

}

void iqueue_mac_node_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_basic_state)
   {
	case N_LISTENNING: iqueue_mac_node_listen_update(iqueuemac); break;
	case N_TRANSMITTING: iqueue_mac_node_transmit_update(iqueuemac); break;
	default: break;
   }
}

void iqueue_mac_update(iqueuemac_t* iqueuemac){

	if(iqueuemac->mac_type == ROUTER){
	  //iqueue_mac_router_update_old(iqueuemac);
	  iqueue_mac_router_update(iqueuemac);
	}else{
	  iqueue_mac_node_update(iqueuemac);
	}
}

///static void _pass_on_packet(gnrc_pktsnip_t *pkt);

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 * @param[in] data      optional parameter
 */
static void _event_cb(netdev2_t *dev, netdev2_event_t event, void *data)
{
    (void) data;
    gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) dev->isr_arg;

    if (event == NETDEV2_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV2_MSG_TYPE_EVENT;
        msg.content.ptr = (void*) gnrc_netdev2;

        if (msg_send(&msg, gnrc_netdev2->pid) <= 0) {
            puts("gnrc_netdev2: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev2: event triggered -> %i\n", event);
        switch(event) {

            case NETDEV2_EVENT_RX_STARTED:
            	iqueuemac.rx_started = true;
            	break;

            case NETDEV2_EVENT_RX_COMPLETE:
                {
                    gnrc_pktsnip_t *pkt = gnrc_netdev2->recv(gnrc_netdev2);

                    if(!iqueuemac.rx_started) {
       				   //LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
       				   gnrc_pktbuf_release(pkt);
       				   iqueuemac.rx_started = false;
                       break;
                    }
                    /*
                    gnrc_netif_hdr_t* netif_hdr;
                    netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
                    printf("shuguo: the received packet rssi is: %d .\n", netif_hdr->rssi);
                    */

                    iqueuemac.rx_started = false;
                    iqueuemac.packet_received = true;

                    if(!packet_queue_push(&iqueuemac.rx.queue, pkt, 0))
                   	{
                    	//LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
                    	gnrc_pktbuf_release(pkt);
                    	break;
                    }
                    iqueuemac.need_update = true;
                    /*
                    if (pkt) {
                        _pass_on_packet(pkt);
                    }*/
                }break;

            case NETDEV2_EVENT_TX_COMPLETE:{
            	iqueuemac.tx.tx_feedback = TX_FEEDBACK_SUCCESS;
            	iqueuemac.tx.tx_finished = true;
            	iqueuemac_set_raddio_to_listen_mode(&iqueuemac);
            	iqueuemac.need_update = true;
            }break;

           	case NETDEV2_EVENT_TX_NOACK:{
           		iqueuemac.tx.tx_feedback = TX_FEEDBACK_NOACK;
           		iqueuemac.tx.tx_finished = true;
           		iqueuemac_set_raddio_to_listen_mode(&iqueuemac);
            	iqueuemac.need_update = true;
           	}break;

           	case NETDEV2_EVENT_TX_MEDIUM_BUSY:{
           		iqueuemac.tx.tx_feedback = TX_FEEDBACK_BUSY;
           		iqueuemac.tx.tx_finished = true;
           		iqueuemac_set_raddio_to_listen_mode(&iqueuemac);
           		iqueuemac.need_update = true;
           	}break;
/*
        	case NETDEV2_EVENT_TX_STARTED:{
        		if(iqueuemac.tx.got_preamble_ack == true){
        		  puts("Shuguo: data packet transmission tx started!");
        		 }
        	}break;*/

#ifdef MODULE_NETSTATS_L2
            case NETDEV2_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV2_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default: break;

                DEBUG("gnrc_netdev2: warning: unhandled event %u.\n", event);
        }
    }
}

//static void _pass_on_packet(gnrc_pktsnip_t *pkt)
//{
    /* throw away packet if no one is interested */
   // if (!gnrc_netapi_dispatch_receive(pkt->type, GNRC_NETREG_DEMUX_CTX_ALL, pkt)) {
   //     DEBUG("gnrc_netdev2: unable to forward packet of type %i\n", pkt->type);
    //    gnrc_pktbuf_release(pkt);
    //    return;
   // }
//}

/**
 * @brief   Startup code and event loop of the gnrc_netdev2 layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_iqueuemac_thread(void *args)
{

    gnrc_netdev2_t* gnrc_netdev2 = iqueuemac.netdev = (gnrc_netdev2_t *)args;
    netdev2_t* dev = gnrc_netdev2->dev;
    iqueuemac.netdev2_driver = dev->driver;
    
        
    /**************************************origin*************************************/
    DEBUG("gnrc_netdev2: starting thread\n");
    //gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) args;
    //netdev2_t *dev = gnrc_netdev2->dev;

    gnrc_netdev2->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV2_NETAPI_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV2_NETAPI_MSG_QUEUE_SIZE);
    /***************************************origin************************************/
    
    /*************************************iqueue-mac**************************************/
    /* RTT is used for scheduling wakeup */
    rtt_init();    
    
    /* Store pid globally, so that IRQ can use it to send msg */
    iqueuemac.pid = thread_getpid();
    /*************************************iqueue-mac**************************************/
    
    /***************************************************************************/
    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->isr_arg = (void*) gnrc_netdev2;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);
    /***************************************************************************/

    iqueuemac.mac_type = MAC_TYPE;

    xtimer_sleep(3);

    iqueuemac_init(&iqueuemac);

    rtt_handler(IQUEUEMAC_EVENT_RTT_START);


    /* start the event loop */
    while (1) {
        DEBUG("gnrc_netdev2: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV2_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev2: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev2: response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev2: response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;

            /**************************************iqueue-mac********************************************/
            case IQUEUEMAC_EVENT_RTT_TYPE:{
                rtt_handler(msg.content.value);
            }break;

            case IQUEUEMAC_EVENT_TIMEOUT_TYPE:{
              //printf("Shuguo: Hitting a timeout event.\n");
              iqueuemac_timeout_make_expire((iqueuemac_timeout_t*) msg.content.ptr);
              iqueuemac.need_update = true;
            }break;

            case GNRC_NETAPI_MSG_TYPE_SND:{
              DEBUG("gnrc_netdev2: GNRC_NETAPI_MSG_TYPE_SND received\n");

              gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)msg.content.ptr;
              //gnrc_netdev2->send(gnrc_netdev2, pkt);

              _queue_tx_packet(&iqueuemac,  pkt);
              iqueuemac.need_update = true;

            }break;
            /**************************************iqueue-mac********************************************/

            default:
                DEBUG("gnrc_netdev2: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }

        while((iqueuemac.need_update == true)&&(iqueuemac.duty_cycle_started == true))
        {
        	iqueuemac.need_update = false;
            iqueue_mac_update(&iqueuemac);
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_iqueuemac_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev2_t *gnrc_netdev2)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev2 == NULL || gnrc_netdev2->dev == NULL) {
        return -ENODEV;
    }

    /* create new gnrc_netdev2 thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _gnrc_iqueuemac_thread, (void *)gnrc_netdev2, name);
    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}

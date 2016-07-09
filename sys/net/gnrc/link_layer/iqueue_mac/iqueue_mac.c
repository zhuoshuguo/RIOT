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
#include "random.h"
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

#define SHUGUO_SAY(value) printf("Shuguo: the value " #value " is %d. \n", value)

static iqueuemac_t iqueuemac;

void iqueuemac_init(iqueuemac_t* iqueuemac)
{

	iqueuemac->own_addr.len = iqueuemac->netdev->dev->driver->get(iqueuemac->netdev->dev, NETOPT_ADDRESS, iqueuemac->own_addr.addr, sizeof(iqueuemac->own_addr.addr));
	//assert(lwmac.l2_addr.len > 0);
	printf("shuguo: iqueuemac's own addrs is: %d, %d . \n ", iqueuemac->own_addr.addr[1], iqueuemac->own_addr.addr[0]);

	if(iqueuemac->mac_type == ROUTER)
	{
		iqueuemac->router_states.router_basic_state = R_INIT;  //R_LISTENNING;
		iqueuemac->router_states.router_init_state = R_INIT_PREPARE;

		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_INIT;
		iqueuemac->router_states.router_trans_state = R_TRANS_TO_UNKOWN;

		iqueuemac->router_states.router_new_cycle = false;

		iqueuemac->rx.router_vtdma_mana.sub_channel_seq = 26;

		iqueuemac->router_states.subchannel_occu_flags = 0;

		/*** set the father-router as itself ***/
		iqueuemac->father_router_addr.len = iqueuemac->own_addr.len;
		memcpy(iqueuemac->father_router_addr.addr,
		       iqueuemac->own_addr.addr,
			   iqueuemac->own_addr.len);

		/*** initiate the sub_channel_num  ***/
		//uint16_t random_channel = iqueuemac->own_addr.addr[0] % 15;
		//iqueuemac->sub_channel_num = 11 + random_channel;
		iqueuemac->sub_channel_num = 13;

	}else{
		iqueuemac->node_states.node_basic_state = N_INIT; //N_LISTENNING;
		iqueuemac->node_states.node_init_state = N_INIT_PREPARE;

		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_INIT;
		iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;

		iqueuemac->father_router_addr.len = 0;
		iqueuemac->father_router_addr.addr[0] = 0;
		iqueuemac->father_router_addr.addr[1] = 0;

		iqueuemac->node_states.in_cp_period = false;

		iqueuemac->node_states.node_new_cycle = false;
	}

	iqueuemac->public_channel_num = 26;

	iqueuemac->device_states.device_broadcast_state = DEVICE_BROADCAST_INIT;
	iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CP_INIT;
	iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;
	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;

	iqueuemac->tx.no_ack_contuer = 0;

	/* Enable RX-start and TX-started and TX-END interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    //iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    /* Initialize broadcast sequence number. This at least differs from board
         * to board */
    iqueuemac->tx.broadcast_seq = iqueuemac->own_addr.addr[0];

    /* First neighbour queue is supposed to be broadcast queue */
    int broadcast_queue_id = _alloc_neighbour(iqueuemac);
    assert(broadcast_queue_id == 0);

    /* Setup broadcast tx queue */
    uint8_t broadcast_addr[] = {0xff, 0xff};
    _init_neighbour(_get_neighbour(iqueuemac, 0), broadcast_addr, sizeof(broadcast_addr));

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


void rtt_handler(uint32_t event)
{
    uint32_t alarm;
    switch(event & 0xffff)
    {
      /*******************************Router RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE:{

          if(iqueuemac.duty_cycle_started == false){
        	  iqueuemac.duty_cycle_started = true;

        	  /*** record the starting phase of iQueuemac ***/
        	  iqueuemac.last_wakeup = rtt_get_counter();
          }else{
        	  iqueuemac.router_states.router_new_cycle = true;
        	  iqueuemac.last_wakeup = rtt_get_alarm(); //rtt_get_counter();
        	  // iqueuemac_stop_lpm();
          }

          lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;

          /// Shuguo: 以后每次进这里把RTT的计时器清零？！ 方便于管理和计算！！？？
          ///rtt_set_counter(0);

          //alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          alarm = iqueuemac.last_wakeup + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);

          iqueuemac.need_update = true;

      }break;

      /******************************* Node new RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE:{

         if(iqueuemac.duty_cycle_started == false){
          	 iqueuemac.duty_cycle_started = true;

             /*** record the starting phase of iQueuemac ***/
             iqueuemac.last_wakeup = rtt_get_counter();
         }else{
             iqueuemac.node_states.node_new_cycle = true;
             iqueuemac.last_wakeup = rtt_get_alarm(); //rtt_get_counter();
             // iqueuemac_stop_lpm();
         }

         lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;

         alarm = iqueuemac.last_wakeup + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
         rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE);

         iqueuemac.need_update = true;

      }break;
      /*******************************Node RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_N_ENTER_CP:{

    	  if(iqueuemac.duty_cycle_started == false){
    	   	  iqueuemac.duty_cycle_started = true;
    	   	  rtt_set_counter(0);
    	  }

    	  rtt_set_counter(0);

    	  /// Shuguo: 以后每次进这里把RTT的计时器清零？！ 方便于管理和计算！！？？
    	  //alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	  alarm = RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	  rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_ENTER_SLEEP);

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

    	  if(iqueuemac.mac_type == ROUTER)
    	  {
    		  iqueuemac.duty_cycle_started = true;
    		  iqueuemac.need_update = true;
    		  /*** set a random starting time here in the future, thus to avoid the same phase for neighbor devices.
    		  puts("shuguo: router starting duty cycling.");

    	      alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	      rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);  ***/
    	  }else{
    		  puts("shuguo: node starting duty cycling.");
    		  /*** set a random starting time here in the future, thus to avoid the same phase for neighbor devices. ***/
     	      alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
     	      rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE);
    	  }
      }break;

      default: break;

      }

}


/****************** Device (both router and node) broadcast state machines*****/

void iqueuemac_device_broadcast_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_on_radio(iqueuemac);

	/*** assemble broadcast packet ***/
	gnrc_pktsnip_t* pkt = iqueuemac->tx.tx_packet;

	iqueuemac_frame_broadcast_t iqueuemac_broadcast_hdr;
	iqueuemac_broadcast_hdr.header.type = FRAMETYPE_BROADCAST;
	iqueuemac_broadcast_hdr.seq_nr = iqueuemac->tx.broadcast_seq;

	pkt->next = gnrc_pktbuf_add(pkt->next, &iqueuemac_broadcast_hdr, sizeof(iqueuemac_broadcast_hdr), GNRC_NETTYPE_IQUEUEMAC);

	iqueuemac_set_timeout(iqueuemac, TIMEOUT_BROADCAST_FINISH, IQUEUEMAC_SUPERFRAME_DURATION_US);

	iqueuemac->device_states.device_broadcast_state = DEVICE_SEND_BROADCAST;
	iqueuemac->need_update = true;

	packet_queue_flush(&iqueuemac->rx.queue);
}

void iqueuemac_device_send_broadcast(iqueuemac_t* iqueuemac){

	gnrc_pktbuf_hold(iqueuemac->tx.tx_packet,1);

	iqueuemac_send(iqueuemac, iqueuemac->tx.tx_packet, NETOPT_DISABLE);

	//iqueue_mac_send_preamble(iqueuemac, NETOPT_ENABLE);

	iqueuemac_set_timeout(iqueuemac, TIMEOUT_BROADCAST_INTERVAL, IQUEUEMAC_BROADCAST_INTERVAL_US);

	iqueuemac->device_states.device_broadcast_state = DEVICE_WAIT_BROADCAST_FEEDBACK;
	iqueuemac->need_update = true;
}

void iqueuemac_device_wait_broadcast_feedback(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_BROADCAST_FINISH)){

		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_BROADCAST_INTERVAL);
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;

		iqueuemac->device_states.device_broadcast_state = DEVICE_BROADCAST_END;
		iqueuemac->need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_BROADCAST_INTERVAL)){

		iqueuemac->device_states.device_broadcast_state = DEVICE_SEND_BROADCAST;
		iqueuemac->need_update = true;
	}
}


void iqueuemac_device_broadcast_end(iqueuemac_t* iqueuemac){

	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_BROADCAST_INTERVAL);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_BROADCAST_FINISH);

	if(iqueuemac->tx.tx_packet){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}
	iqueuemac->tx.current_neighbour = NULL;

	iqueuemac->device_states.device_broadcast_state = DEVICE_BROADCAST_INIT;

	if(iqueuemac->mac_type == ROUTER){
	    /*********** judge and update the states before switch back to CP listening period   ***********/
	    iqueuemac->router_states.router_basic_state = R_LISTENNING;
	    iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING;
	    iqueuemac->router_states.router_new_cycle = false;

	    iqueuemac_trun_off_radio(iqueuemac);

	    puts("Shuguo: router is in broadcast end, switching back to sleeping period");
	}else{
		iqueuemac->node_states.node_basic_state = N_LISTENNING;

		iqueuemac->node_states.node_new_cycle = false;

		/*********** judge and update the states before switch back to CP listening period   ***********/
		uint32_t current_phase;
		current_phase = _phase_now(iqueuemac);

		if(current_phase < RTT_US_TO_TICKS(IQUEUEMAC_QUIT_CP_MARGIN_US)){   ///iqueuemac->node_states.in_cp_period == true
			/** if there is no buffered packet to send, then we are sure to go to listen state!! **/
			if(iqueue_mac_find_next_tx_neighbor(iqueuemac) == false){
			    iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			    //puts("Shuguo: node (device) is in t-2-u end, turn to listen.");
			}else{/** if there is buffered packet to send, then judge the receiver's phase.
                      if the phase is too far from now, then still go to listen state, otherwise, go to transmit state **/
				if(iqueuemac->tx.current_neighbour->mac_type == UNKNOWN){
					iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				}else{/** if the receiver is not unknown type, check how far its phase from now! **/
				    uint32_t wait_phase_duration;
				    wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);
				    wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;

				    if(wait_phase_duration > (2*IQUEUEMAC_CP_DURATION_US)){
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				    }else{
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
				    }
				}
			}
			//iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
		    //puts("Shuguo: node is in broadcast end and switch to listen's CP");
	    }else{
		    iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
		    iqueuemac_trun_off_radio(iqueuemac);
		    puts("Shuguo: node is in broadcast end and switch to listen's sleep");
	    }
	}
	iqueuemac->need_update = true;
}


void iqueuemac_device_broadcast_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->device_states.device_broadcast_state)
	{
	 case DEVICE_BROADCAST_INIT: iqueuemac_device_broadcast_init(iqueuemac);break;
	 case DEVICE_SEND_BROADCAST: iqueuemac_device_send_broadcast(iqueuemac); break;
	 case DEVICE_WAIT_BROADCAST_FEEDBACK: iqueuemac_device_wait_broadcast_feedback(iqueuemac); break;
	 case DEVICE_BROADCAST_END: iqueuemac_device_broadcast_end(iqueuemac);break;
	 default: break;
	}
}


/****************** iQueue-MAC transmission to node state machines *****/

void iqueuemac_init_prepare(iqueuemac_t* iqueuemac){

	rtt_clear_alarm();

	uint32_t listen_period;

	listen_period = random_uint32_range(0, IQUEUEMAC_SUPERFRAME_DURATION_US);
	listen_period = (IQUEUEMAC_SUPERFRAME_DURATION_US*11/10) + listen_period;

	iqueuemac->quit_current_cycle = false;
	iqueuemac->router_states.init_retry = false;
	iqueuemac->router_states.subchannel_occu_flags = 0;

	packet_queue_flush(&iqueuemac->rx.queue);

	/******set TIMEOUT_COLLECT_BEACON_END timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_COLLECT_BEACON_END, listen_period);

	iqueuemac->router_states.router_init_state = R_INIT_COLLECT_BEACONS;
	iqueuemac->need_update = true;

}

void iqueuemac_init_collec_beacons(iqueuemac_t* iqueuemac){

	if(iqueuemac->packet_received == true){
	   	iqueuemac->packet_received = false;
	   	iqueuemac_packet_process_in_init(iqueuemac);
	}

	if(iqueuemac->quit_current_cycle == true){
		iqueuemac_trun_off_radio(iqueuemac);
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_COLLECT_BEACON_END);
		iqueuemac_set_timeout(iqueuemac, TIMEOUT_BROADCAST_FINISH, IQUEUEMAC_SUPERFRAME_DURATION_US);
		iqueuemac->router_states.router_init_state = R_INIT_WAIT_BUSY_END;
		iqueuemac->need_update = true;
		return;
	}

	/*** it seems that this "init_retry" procedure is unnecessary here!! maybe delete it in the future ***/
	if(iqueuemac->router_states.init_retry == true){
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_COLLECT_BEACON_END);
		packet_queue_flush(&iqueuemac->rx.queue);
		iqueuemac->router_states.router_init_state = R_INIT_PREPARE;
		iqueuemac->need_update = true;
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_COLLECT_BEACON_END)){
		iqueuemac_init_choose_subchannel(iqueuemac);
		iqueuemac->router_states.router_init_state = R_INIT_ANNOUNCE_SUBCHANNEL;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_init_wait_busy_end(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_BROADCAST_FINISH)){
		iqueuemac_trun_on_radio(iqueuemac);
		iqueuemac->router_states.router_init_state = R_INIT_PREPARE;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_init_announce_subchannel(iqueuemac_t* iqueuemac){

	//set csma retry number here??
	iqueuemac_send_announce(iqueuemac,NETOPT_ENABLE);

	iqueuemac->router_states.router_init_state = R_INIT_WAIT_ANNOUNCE_FEEDBACK;
	iqueuemac->need_update = true;
}

void iqueuemac_init_wait_announce_feedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){

		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/
		if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_SUCCESS){
			packet_queue_flush(&iqueuemac->rx.queue);
			iqueuemac->router_states.router_init_state = R_INIT_END;
			iqueuemac->need_update = true;
			return;
		}else{ //if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_BUSY)
			iqueuemac->router_states.router_init_state = R_INIT_PREPARE;
			iqueuemac->need_update = true;
		}
	}
}

void iqueuemac_init_end(iqueuemac_t* iqueuemac){

	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_COLLECT_BEACON_END);

	iqueuemac->router_states.router_init_state = R_INIT_PREPARE;
	/*** switch to duty-cycle operation ***/
	iqueuemac->router_states.router_basic_state = R_LISTENNING;
	iqueuemac->router_states.router_listen_state = R_LISTEN_CP_INIT;

	puts("router random ends.");
	/*** start duty-cycle ***/
	iqueuemac->duty_cycle_started = false;
	rtt_handler(IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);
	iqueuemac->need_update = true;
}

void iqueuemac_init_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_states.router_init_state)
	{
		case R_INIT_PREPARE: iqueuemac_init_prepare(iqueuemac);break;
		case R_INIT_COLLECT_BEACONS: iqueuemac_init_collec_beacons(iqueuemac);break;
		case R_INIT_WAIT_BUSY_END: iqueuemac_init_wait_busy_end(iqueuemac);break;
		case R_INIT_ANNOUNCE_SUBCHANNEL: iqueuemac_init_announce_subchannel(iqueuemac);break;
		case R_INIT_WAIT_ANNOUNCE_FEEDBACK: iqueuemac_init_wait_announce_feedback(iqueuemac);break;
		case R_INIT_END: iqueuemac_init_end(iqueuemac);break;
		default: break;
	}
}


void iqueuemac_node_init_prepare(iqueuemac_t* iqueuemac){

	rtt_clear_alarm();

	uint32_t random_wait_period;

	random_wait_period = random_uint32_range(0, IQUEUEMAC_SUPERFRAME_DURATION_US);

	iqueuemac_trun_off_radio(iqueuemac);

	packet_queue_flush(&iqueuemac->rx.queue);

	/******set TIMEOUT_COLLECT_BEACON_END timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_COLLECT_BEACON_END, random_wait_period);

	iqueuemac->node_states.node_init_state = N_INIT_WAIT_TIMEOUT;
	iqueuemac->need_update = true;

}

void iqueuemac_node_init_wait_timeout(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_COLLECT_BEACON_END)){
		puts("shuguo: random ends.");
		iqueuemac->node_states.node_init_state = N_INIT_END;
		iqueuemac->need_update = true;
	}
}


void iqueuemac_node_init_end(iqueuemac_t* iqueuemac){

	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_COLLECT_BEACON_END);

	iqueuemac->node_states.node_init_state = N_INIT_PREPARE;
	/*** switch to duty-cycle operation ***/
	iqueuemac->node_states.node_basic_state = N_LISTENNING;
	iqueuemac->node_states.node_init_state = N_LISTEN_CP_INIT;

	/*** start duty-cycle ***/
	iqueuemac->duty_cycle_started = false;
	rtt_handler(IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE);
	iqueuemac->need_update = true;
}

void iqueuemac_node_init_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_init_state)
	{
		case N_INIT_PREPARE: iqueuemac_node_init_prepare(iqueuemac);break;
		case N_INIT_WAIT_TIMEOUT: iqueuemac_node_init_wait_timeout(iqueuemac);break;
		case N_INIT_END: iqueuemac_node_init_end(iqueuemac);break;
		default: break;
	}
}


/****************** iQueue-MAC transmission to node state machines *****/

void iqueuemac_t2n_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_off_radio(iqueuemac);

	/*** set timer for the targeted node! ***/
	uint32_t wait_phase_duration;
	wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);

	wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_CP, wait_phase_duration);
/*
	printf("shuguo: the wait phase time is %lu us .\n" , wait_phase_duration);
	wait_phase_duration = RTT_TICKS_TO_US(iqueuemac->tx.current_neighbour->cp_phase);
	printf("shuguo: the dest's phase is %lu us .\n" , wait_phase_duration);
*/
	/*** flush the rx-queue here to reduce possible buffered packet in RIOT!! ***/
	packet_queue_flush(&iqueuemac->rx.queue);

	iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CP;
	iqueuemac->need_update = true;
}

void iqueuemac_t2n_wait_cp(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_CP)){
		iqueuemac->tx.cp_retries = 0;

		iqueuemac_trun_on_radio(iqueuemac);
		iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_TRANS_IN_CP;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2n_trans_in_cp(iqueuemac_t* iqueuemac){

	/******Use CSMA here, and send_packet() will release the pkt itself !!!!******/
	iqueuemac_send_data_packet(iqueuemac, NETOPT_ENABLE);

	/*** here, must set to NULL!! don't know why!!  ***/
	//iqueuemac->tx.tx_packet = NULL;

	iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CPTRANS_FEEDBACK;
	iqueuemac->need_update = true;

}

void iqueuemac_t2n_wait_cp_transfeedback(iqueuemac_t* iqueuemac){

	/*** add another condition here in the future: the tx-feedback must be ACK-got,
			 * namely, completed, to ensure router gets the data correctly***/
	if(iqueuemac->tx.tx_finished == true){

		switch(iqueuemac->tx.tx_feedback){
			case TX_FEEDBACK_SUCCESS:{
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_TRANS_END;
			}break;

			/*** if NOACK, regards it as phase-lock failed ***/
			case TX_FEEDBACK_NOACK:{
				puts("phase-lock failed.");
				iqueuemac->tx.current_neighbour->mac_type = UNKNOWN;

				iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_RE_PHASE_LOCK, (IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_RE_PHASE_LOCK_ADVANCE_US));
				iqueuemac_trun_off_radio(iqueuemac);
				iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_RE_PHASE_LOCK_PREPARE;
			}break;

			/*** if BUSY, regards it as channel busy ***/
			case TX_FEEDBACK_BUSY:{
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_TRANS_END;
			}break;

			default:{
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;
				iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_TRANS_END;
			}break;
		}
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2n_re_phase_lock_prepare(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_RE_PHASE_LOCK)){
		/*** leaving t-2-n, so initiate the state ***/
		iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CP_INIT;

		if(iqueuemac->mac_type == ROUTER){
			iqueuemac->router_states.router_trans_state = R_TRANS_TO_UNKOWN;
		}else{
			iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;
		}

		iqueuemac->need_update = true;
	}
}


void iqueuemac_t2n_end(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_packet){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}

	iqueuemac->tx.current_neighbour = NULL;

	/*** clear all timeouts ***/
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_CP);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_RE_PHASE_LOCK);

	iqueuemac->device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CP_INIT;

	if(iqueuemac->mac_type == ROUTER){
		/*********** judge and update the states before switch back to CP listening period   ***********/
		iqueuemac->router_states.router_basic_state = R_LISTENNING;
	    iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING;
	    iqueuemac->router_states.router_new_cycle = false;

	    iqueuemac_trun_off_radio(iqueuemac);
	    //puts("Shuguo: router (device) is in t-2-n end.");
	}else{
		iqueuemac->node_states.node_basic_state = N_LISTENNING;

		iqueuemac->node_states.node_new_cycle = false;

		/*********** judge and update the states before switch back to CP listening period ***********/
		uint32_t current_phase;
		current_phase = _phase_now(iqueuemac);

		if(current_phase < RTT_US_TO_TICKS(IQUEUEMAC_QUIT_CP_MARGIN_US)){   ///iqueuemac->node_states.in_cp_period == true
			/** if there is no buffered packet to send, then we are sure to go to listen state!! **/
			if(iqueue_mac_find_next_tx_neighbor(iqueuemac) == false){
			    iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			    //puts("Shuguo: node (device) is in t-2-u end, turn to listen.");
			}else{/** if there is buffered packet to send, then judge the receiver's phase.
                      if the phase is too far from now, then still go to listen state, otherwise, go to transmit state **/
				if(iqueuemac->tx.current_neighbour->mac_type == UNKNOWN){
					iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				}else{/** if the receiver is not unknown type, check how far its phase from now! **/
				    uint32_t wait_phase_duration;
				    wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);
				    wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;

				    if(wait_phase_duration > (2*IQUEUEMAC_CP_DURATION_US)){
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				    }else{
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
				    }
				}
			}
			//iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			//puts("Shuguo: node (device) is in t2n end, switch to listen's CP");
		}else{
			iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
			iqueuemac_trun_off_radio(iqueuemac);
			//puts("Shuguo: node (device) is in t2u end, switch to listen's sleep");
		}
	}

	iqueuemac->need_update = true;
}


void iqueuemac_t2n_update(iqueuemac_t* iqueuemac)
{
	switch(iqueuemac->device_states.iqueuemac_device_t2n_state)
	{
	 case DEVICE_T2N_WAIT_CP_INIT: iqueuemac_t2n_init(iqueuemac);break;
     case DEVICE_T2N_WAIT_CP: iqueuemac_t2n_wait_cp(iqueuemac); break;
	 case DEVICE_T2N_TRANS_IN_CP: iqueuemac_t2n_trans_in_cp(iqueuemac); break;
	 case DEVICE_T2N_WAIT_CPTRANS_FEEDBACK: iqueuemac_t2n_wait_cp_transfeedback(iqueuemac); break;
	 case DEVICE_T2N_RE_PHASE_LOCK_PREPARE: iqueuemac_t2n_re_phase_lock_prepare(iqueuemac); break;
	 case DEVICE_T2N_TRANS_END:iqueuemac_t2n_end(iqueuemac);break;
	 default: break;
	}
}


/****************** iQueuemac: Transmit to router *****/
void iqueuemac_t2r_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_off_radio(iqueuemac);

	/* set timer for the targeted router! */
	uint32_t wait_phase_duration;

	wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);
	wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_CP, wait_phase_duration);

	/*** flush the rx-queue here to reduce possible buffered packet in RIOT!! ***/
	packet_queue_flush(&iqueuemac->rx.queue);

	iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP;
	iqueuemac->need_update = true;
}

void iqueuemac_t2r_wait_cp(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_CP)){
		iqueuemac_trun_on_radio(iqueuemac);
		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_CP;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2r_trans_in_cp(iqueuemac_t* iqueuemac){

	/******Use CSMA here, and send_packet() will release the pkt itself !!!!******/
	iqueuemac_send_data_packet(iqueuemac, NETOPT_ENABLE);

	//iqueuemac->tx.tx_packet = NULL;

	iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CPTRANS_FEEDBACK;
	iqueuemac->need_update = true;

}

void iqueuemac_t2r_wait_cp_transfeedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
				 * namely, completed, to ensure router gets the data correctly***/
		switch(iqueuemac->tx.tx_feedback){

			case TX_FEEDBACK_SUCCESS:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				/*** if has pending pkt, join the vTDMA period, first wait receiver's beacon ***/
				if(iqueuemac->tx.current_neighbour->queue.length > 0){
					iqueuemac->tx.vtdma_para.get_beacon = false;
					iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_BEACON, IQUEUEMAC_SUPERFRAME_DURATION_US);
					// need to flush the rx-queue ??
					packet_queue_flush(&iqueuemac->rx.queue);

					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_BEACON;
				}else{
					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				iqueuemac->need_update = true;
			}break;

			/*** if NOACK, regards it as phase-lock failed ***/
			case TX_FEEDBACK_NOACK:{

				iqueuemac->tx.no_ack_contuer ++;

				if(iqueuemac->tx.no_ack_contuer >= IQUEUEMAC_REPHASELOCK_THRESHOLD){
					iqueuemac->tx.no_ack_contuer = 0;

					//puts("phase-lock failed.");
					iqueuemac->tx.current_neighbour->mac_type = UNKNOWN;

					iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_RE_PHASE_LOCK, (IQUEUEMAC_SUPERFRAME_DURATION_US - IQUEUEMAC_RE_PHASE_LOCK_ADVANCE_US));
					iqueuemac_trun_off_radio(iqueuemac);
					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_RE_PHASE_LOCK_PREPARE;

				}else{
					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2N_TRANS_END;
				}
				iqueuemac->need_update = true;
				/*** pkt trans failed, don't release the pkt here, retry to transmit in t-2-u procedure. ***/

			}break;

			/*** if BUSY, regards it as channel busy ***/
			case TX_FEEDBACK_BUSY:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2N_TRANS_END;
				iqueuemac->need_update = true;

			}break;

			default:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2N_TRANS_END;
				iqueuemac->need_update = true;
			}break;

		}
	}
}

void iqueuemac_t2r_re_phase_lock_prepare(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_RE_PHASE_LOCK)){
		/*** leaving t-2-r, so initiate the state ***/
		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;

		if(iqueuemac->mac_type == ROUTER){
			iqueuemac->router_states.router_trans_state = R_TRANS_TO_UNKOWN;
		}else{
			iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;
		}

		iqueuemac->need_update = true;
	}

}

void iqueuemac_t2r_wait_beacon(iqueuemac_t* iqueuemac){

    if(iqueuemac->packet_received == true){
    	iqueuemac->packet_received = false;
    	iqueuemac_wait_beacon_packet_process(iqueuemac);
    }

    if(iqueuemac->tx.vtdma_para.get_beacon == true){

    	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_BEACON);

    	if(iqueuemac->tx.vtdma_para.slots_num > 0){

    		/*** switch the radio to the sub-channel ***/
    		uint16_t tartget_sub_channel_seq;
    		tartget_sub_channel_seq = (uint16_t)iqueuemac->tx.vtdma_para.sub_channel_seq;
    		iqueuemac_turn_radio_channel(iqueuemac, tartget_sub_channel_seq);

    		if(iqueuemac->tx.vtdma_para.slots_position > 0){
    			/*** wait for the finish of switching channel !!! and then turn off the radio to save power ***/
    			iqueuemac_trun_off_radio(iqueuemac);

    			uint32_t wait_slots_duration;
    			wait_slots_duration = iqueuemac->tx.vtdma_para.slots_position * IQUEUEMAC_VTDMA_SLOT_SIZE_US;
    			iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_OWN_SLOTS, wait_slots_duration);

    			iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_OWN_SLOTS;
    			iqueuemac->need_update = true;
    		}else{// be the first sender in vtdma

    			gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.current_neighbour->queue));

    			if(pkt != NULL){
    				iqueuemac->tx.tx_packet = pkt;
    				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
    			}else{
    				puts("iqueueMAC-Error: NUll pktbuf!");
    				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
    			}

    			iqueuemac->need_update = true;
    		}
    	}else{/*** no slots get allocated, go to t-2-r end ***/
    		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
    		iqueuemac->need_update = true;
    	}
    	return;
    }

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_BEACON)){
		puts("Shuguo: No beacon.");
		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2r_wait_own_slots(iqueuemac_t* iqueuemac){

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_WAIT_OWN_SLOTS)){

		iqueuemac_trun_on_radio(iqueuemac);

		gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.current_neighbour->queue));

		if(pkt != NULL){
			iqueuemac->tx.tx_packet = pkt;
			iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
		}else{
			puts("iqueueMAC-Error: NUll pktbuf!");
			iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		}
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2r_trans_in_slots(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.vtdma_para.slots_num > 0){

		/**** Delete the pkt no matter the transmission is success or not !!! ****/
		/**** Or, only delete the pkt when the feedback shows good !!! ****/
		//gnrc_pktsnip_t *pkt = packet_queue_head(&(iqueuemac->tx.current_neighbour.queue));

		/******disable CSMA here, and iqueuemac_send_data_packet() will release the pkt itself !!!!******/
		iqueuemac_send_data_packet(iqueuemac, NETOPT_DISABLE);
		iqueuemac->tx.vtdma_para.slots_num --;
		//iqueuemac->tx.tx_packet = NULL;

		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_VTDMATRANS_FEEDBACK;
		iqueuemac->need_update = true;

	}else{/*** here means the slots have been used up !!! ***/
		/****  switch back to the public channel ****/
		iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->public_channel_num);

		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		iqueuemac->need_update = true;
	}
}

void iqueuemac_t2r_wait_vtdma_transfeedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/

		switch(iqueuemac->tx.tx_feedback){

			case TX_FEEDBACK_SUCCESS:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				/*** if the sender has pending pkt, continue vTDMA transmission ***/
				if((iqueuemac->tx.vtdma_para.slots_num > 0)&&(iqueuemac->tx.current_neighbour->queue.length>0)){
					gnrc_pktsnip_t *pkt = packet_queue_pop(&(iqueuemac->tx.current_neighbour->queue));

					if(pkt != NULL){
						iqueuemac->tx.tx_packet = pkt;
						iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
					}else{
						puts("iqueueMAC-Error: NUll pktbuf!");
						iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
					}
				}else{
					/****  vtdma period ends, switch back to the public channel ****/
					iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->public_channel_num);

					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				iqueuemac->need_update = true;
			}break;

			/*** if BUSY and NOACK, regards it as busy channel ***/
			case TX_FEEDBACK_BUSY:
			case TX_FEEDBACK_NOACK:{
				/*** do not release the pkt here, continue sending the same pkt ***/
				if(iqueuemac->tx.vtdma_para.slots_num > 0){

					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;

				}else{ /* if no slots for sending, queue the pkt for retry in next cycle */

					//puts("failed in vTDMA, re-queue pkt.");
		           /* save payload pointer */
		            gnrc_pktsnip_t* payload = iqueuemac->tx.tx_packet->next->next;

		            /* remove iqueuemac header */
		            iqueuemac->tx.tx_packet->next->next = NULL;
		            gnrc_pktbuf_release(iqueuemac->tx.tx_packet->next);

		            /* make append payload after netif header again */
		            iqueuemac->tx.tx_packet->next = payload;

		            /* queue the pkt for transmission in next cycle */
		            _queue_tx_packet(iqueuemac, iqueuemac->tx.tx_packet);
		            iqueuemac->tx.tx_packet = NULL;

		            /****  vtdma period ends, switch back to the public channel ****/
					iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->public_channel_num);
					iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				iqueuemac->need_update = true;

			}break;

			default:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
				iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2N_TRANS_END;
				iqueuemac->need_update = true;
			}break;
		}
	}
}

void iqueuemac_t2r_end(iqueuemac_t* iqueuemac){

	if((iqueuemac->tx.tx_packet)&&(iqueuemac->tx.no_ack_contuer == 0)){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}
	if(iqueuemac->tx.no_ack_contuer == 0){
		iqueuemac->tx.current_neighbour = NULL;
	}

	/*** clear all timeouts ***/
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_CP);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_BEACON);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_WAIT_OWN_SLOTS);

	iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;

	if(iqueuemac->mac_type == ROUTER){
		/*********** judge and update the states before switch back to CP listening period   ***********/
	    iqueuemac->router_states.router_basic_state = R_LISTENNING;
		iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING;
		iqueuemac->router_states.router_new_cycle = false;

		iqueuemac_trun_off_radio(iqueuemac);
		//puts("Shuguo: router (device) is in t-2-r end.");
	}else{
		iqueuemac->node_states.node_basic_state = N_LISTENNING;

		iqueuemac->node_states.node_new_cycle = false;

		/*********** judge and update the states before switch back to CP listening period ***********/
		uint32_t current_phase;
		current_phase = _phase_now(iqueuemac);

		if(current_phase < RTT_US_TO_TICKS(IQUEUEMAC_QUIT_CP_MARGIN_US)){   ///iqueuemac->node_states.in_cp_period == true
			/** if there is no buffered packet to send, then we are sure to go to listen state!! **/
			if(iqueue_mac_find_next_tx_neighbor(iqueuemac) == false){
			    iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			    //puts("Shuguo: node (device) is in t-2-u end, turn to listen.");
			}else{/** if there is buffered packet to send, then judge the receiver's phase.
                      if the phase is too far from now, then still go to listen state, otherwise, go to transmit state **/
				if(iqueuemac->tx.current_neighbour->mac_type == UNKNOWN){
					iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				}else{/** if the receiver is not unknown type, check how far its phase from now! **/
				    uint32_t wait_phase_duration;
				    wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);
				    wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;

				    if(wait_phase_duration > (2*IQUEUEMAC_CP_DURATION_US)){
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				    }else{
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
				    }
				}
			}
			//iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			//puts("Shuguo: node (device) is in t-2-r end, turn to listen.");
		}else{
			iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
			iqueuemac_trun_off_radio(iqueuemac);
			//puts("Shuguo: node (device) is in t-2-r end, turn to sleep.");
		}
	}
	iqueuemac->need_update = true;
}


void iqueuemac_t2r_update(iqueuemac_t* iqueuemac)
{
	switch(iqueuemac->device_states.iqueuemac_device_t2r_state)
	{
	 case DEVICE_T2R_WAIT_CP_INIT: iqueuemac_t2r_init(iqueuemac);break;
     case DEVICE_T2R_WAIT_CP: iqueuemac_t2r_wait_cp(iqueuemac); break;
	 case DEVICE_T2R_TRANS_IN_CP: iqueuemac_t2r_trans_in_cp(iqueuemac); break;
	 case DEVICE_T2R_WAIT_CPTRANS_FEEDBACK: iqueuemac_t2r_wait_cp_transfeedback(iqueuemac); break;
	 case DEVICE_T2R_RE_PHASE_LOCK_PREPARE: iqueuemac_t2r_re_phase_lock_prepare(iqueuemac); break;
	 case DEVICE_T2R_WAIT_BEACON: iqueuemac_t2r_wait_beacon(iqueuemac);break;
	 case DEVICE_T2R_WAIT_OWN_SLOTS:iqueuemac_t2r_wait_own_slots(iqueuemac); break;
	 case DEVICE_T2R_TRANS_IN_VTDMA:iqueuemac_t2r_trans_in_slots(iqueuemac); break;
	 case DEVICE_T2R_WAIT_VTDMATRANS_FEEDBACK:iqueuemac_t2r_wait_vtdma_transfeedback(iqueuemac);break;
	 case DEVICE_T2R_TRANS_END:iqueuemac_t2r_end(iqueuemac);break;
	 default: break;
	}
}


/****************** device state machines - Transmit to Unknown *****/
void iqueuemac_t2u_send_preamble_init(iqueuemac_t* iqueuemac){

	iqueuemac_trun_on_radio(iqueuemac);
	iqueuemac->packet_received = false;
	iqueuemac->tx.preamble_sent = 0;
	iqueuemac->tx.got_preamble_ack = false;

	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE;
	iqueuemac->need_update = true;

	packet_queue_flush(&iqueuemac->rx.queue);
}

void iqueuemac_t2u_send_preamble(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.preamble_sent == 0){
		iqueue_mac_send_preamble(iqueuemac, NETOPT_ENABLE);
		iqueuemac_set_timeout(iqueuemac, TIMEOUT_PREAMBLE_DURATION, IQUEUEMAC_SUPERFRAME_DURATION_US);
	}else{
		iqueue_mac_send_preamble(iqueuemac, NETOPT_DISABLE);
	}

	iqueuemac->tx.preamble_sent ++;

	/******set preamble timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_PREAMBLE, IQUEUEMAC_PREAMBLE_INTERVAL_US);

	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_WAIT_PREAMBLE_ACK;
	iqueuemac->need_update = false;
}

void iqueuemac_t2u_wait_preamble_ack(iqueuemac_t* iqueuemac){

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
		iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_DATA;
		iqueuemac->need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_PREAMBLE_DURATION)){
		iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac->need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_PREAMBLE)){
		iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE;
		iqueuemac->need_update = true;
		return;
	}
}

void iqueuemac_t2u_send_data(iqueuemac_t* iqueuemac){

	iqueuemac_send_data_packet(iqueuemac, NETOPT_DISABLE);

	//iqueuemac->tx.tx_packet = NULL;

	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_WAIT_TX_FEEDBACK;
	iqueuemac->need_update = true;
	/**** add a "wait-for-tx feedback" period to wait for the ending of the tx operation ***/

	// in case router want to wait beacon for vtdma, it swithc states to t-2-r and wait-for-beacon here!!
	// first test this idea with simple node type!!
}

void iqueuemac_t2u_wait_tx_feedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){

	/*** add another condition here in the furture: the tx-feedback must be ACK-got,
	 * namely, completed, to ensure router gets the data correctly***/
	    if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_SUCCESS){

	    	gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
	    	iqueuemac->tx.tx_packet = NULL;

	    	/*** TX_FEEDBACK_SUCCESS means the router has success received the queue-length indicator ***/
	    	/*  add this part in the future to support vtdma in sending-to-unkown (to router type) */
	    	if((iqueuemac->tx.current_neighbour->queue.length > 0)&&(iqueuemac->tx.current_neighbour->mac_type == ROUTER)){

	    	    iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;
	    		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
	    		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE_DURATION);

	    		iqueuemac->tx.vtdma_para.get_beacon = false;
	    		iqueuemac_set_timeout(iqueuemac, TIMEOUT_WAIT_BEACON, IQUEUEMAC_SUPERFRAME_DURATION_US);
	    		// need to flush the rx-queue ??
	    		packet_queue_flush(&iqueuemac->rx.queue);

	    		if(iqueuemac->mac_type == ROUTER){
	    			iqueuemac->router_states.router_trans_state = R_TRANS_TO_ROUTER;
	    		}else{
	    		   	iqueuemac->node_states.node_trans_state = N_TRANS_TO_ROUTER;
	    		}

	    		iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_BEACON;
	    	}else{
	    	   	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
	    	}
	    }else{
	    	gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
	    	iqueuemac->tx.tx_packet = NULL;

	    	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
	    }

	    iqueuemac->need_update = true;
	}
}

void iqueuemac_t2u_end(iqueuemac_t* iqueuemac){

	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE);
	iqueuemac_clear_timeout(iqueuemac,TIMEOUT_PREAMBLE_DURATION);

	if(iqueuemac->tx.tx_packet){
		gnrc_pktbuf_release(iqueuemac->tx.tx_packet);
		iqueuemac->tx.tx_packet = NULL;
	}
	iqueuemac->tx.current_neighbour = NULL;

	iqueuemac->device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;

	if(iqueuemac->mac_type == ROUTER){
		/*********** judge and update the states before switch back to CP listening period   ***********/
	    iqueuemac->router_states.router_basic_state = R_LISTENNING;
		iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING;
		iqueuemac->router_states.router_new_cycle = false;

		iqueuemac_trun_off_radio(iqueuemac);
		//puts("Shuguo: router (device) is in t-2-u end.");
	}else{
		iqueuemac->node_states.node_basic_state = N_LISTENNING;

		iqueuemac->node_states.node_new_cycle = false;

		/*********** judge and update the states before switch back to CP listening period ***********/
		uint32_t current_phase;
		current_phase = _phase_now(iqueuemac);

		if(current_phase < RTT_US_TO_TICKS(IQUEUEMAC_QUIT_CP_MARGIN_US)){   ///iqueuemac->node_states.in_cp_period == true
			/** if there is no buffered packet to send, then we are sure to go to listen state!! **/
			if(iqueue_mac_find_next_tx_neighbor(iqueuemac) == false){
			    iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
			    //puts("Shuguo: node (device) is in t-2-u end, turn to listen.");
			}else{/** if there is buffered packet to send, then judge the receiver's phase.
                      if the phase is too far from now, then still go to listen state, otherwise, go to transmit state **/
				if(iqueuemac->tx.current_neighbour->mac_type == UNKNOWN){
					iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				}else{/** if the receiver is not unknown type, check how far its phase from now! **/
				    uint32_t wait_phase_duration;
				    wait_phase_duration = _ticks_until_phase(iqueuemac, iqueuemac->tx.current_neighbour->cp_phase);
				    wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;

				    if(wait_phase_duration > (2*IQUEUEMAC_CP_DURATION_US)){
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_CP_LISTEN;
				    }else{
				    	iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
				    }
				}
			}
		}else{
			iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING;
			iqueuemac_trun_off_radio(iqueuemac);
			//puts("Shuguo: node (device) is in t-2-u end, turn to sleep.");
		}
	}
	iqueuemac->need_update = true;
}

void iqueuemac_t2u_update(iqueuemac_t* iqueuemac)
{
	switch(iqueuemac->device_states.iqueuemac_device_t2u_state)
	{
	 case DEVICE_T2U_SEND_PREAMBLE_INIT: iqueuemac_t2u_send_preamble_init(iqueuemac);break;
     case DEVICE_T2U_SEND_PREAMBLE: iqueuemac_t2u_send_preamble(iqueuemac); break;
	 case DEVICE_T2U_WAIT_PREAMBLE_ACK: iqueuemac_t2u_wait_preamble_ack(iqueuemac); break;
	 case DEVICE_T2U_SEND_DATA: iqueuemac_t2u_send_data(iqueuemac); break;
	 case DEVICE_T2U_WAIT_TX_FEEDBACK: iqueuemac_t2u_wait_tx_feedback(iqueuemac);break;
	 case DEVICE_T2U_END: iqueuemac_t2u_end(iqueuemac);break;
	 default: break;
	}
}

/******************new router state machines*****/

void iqueue_mac_router_listen_cp_init(iqueuemac_t* iqueuemac){

	iqueuemac->router_states.router_new_cycle = false;
	/******set cp timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);

	iqueuemac_trun_on_radio(iqueuemac);
	iqueuemac->packet_received = false;

	iqueuemac->router_states.router_listen_state = R_LISTEN_CP_LISTEN;
	iqueuemac->need_update = true;

	//puts("Shuguo: router is now entering CP");
	//SHUGUO_SAY(iqueuemac->public_channel_num);

	iqueuemac->quit_current_cycle = false;

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

	if((iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_CP_END))||(iqueuemac->quit_current_cycle == true)){
		//puts("Shuguo: Router CP ends!!");
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_CP_END);
		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}

	/*
	if(iqueuemac->quit_current_cycle == true){
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_CP_END);
		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}*/
}

void iqueue_mac_router_cp_end(iqueuemac_t* iqueuemac){

	packet_queue_flush(&iqueuemac->rx.queue);

	_dispatch(iqueuemac->rx.dispatch_buffer);

	if(iqueuemac->quit_current_cycle == true){
		iqueuemac->quit_current_cycle = false;
		iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
	}else{
		iqueuemac->router_states.router_listen_state = R_LISTEN_SEND_BEACON;
	}
	iqueuemac->need_update = true;
}

void iqueue_mac_router_send_beacon(iqueuemac_t* iqueuemac){
    /**** run the sub-channel selection algorithm to select the sub-channel sequence ****/
	// iqueuemac_select_sub_channel_num(iqueuemac);

	/****** assemble and send the beacon ******/
	iqueuemac_assemble_and_send_beacon(iqueuemac);

	iqueuemac->router_states.router_listen_state = R_LISTEN_WAIT_BEACON_FEEDBACK;
	iqueuemac->need_update = true;

	//puts("Shuguo: router is now sending the beacon!!!");

}

void iqueuemac_router_wait_beacon_feedback(iqueuemac_t* iqueuemac){

	if(iqueuemac->tx.tx_finished == true){

		/****** router switch to sleep period or vTDMA period ******/
		if(iqueuemac->rx.router_vtdma_mana.total_slots_num > 0){
			iqueuemac->router_states.router_listen_state = R_LISTEN_VTDMA_INIT;
			iqueuemac->need_update = true;
		}else{ /**** no vTDMA period ****/
			if(iqueue_mac_find_next_tx_neighbor(iqueuemac)){

				iqueuemac->router_states.router_basic_state = R_TRANSMITTING;

				if(iqueuemac->tx.current_neighbour == &iqueuemac->tx.neighbours[0]){
					iqueuemac->router_states.router_trans_state = R_BROADCAST;
				}else{
					switch(iqueuemac->tx.current_neighbour->mac_type){
					  case UNKNOWN: {
						  iqueuemac->router_states.router_trans_state = R_TRANS_TO_UNKOWN;
					  }break;
					  case ROUTER: {
						  iqueuemac->router_states.router_trans_state = R_TRANS_TO_ROUTER;
				 	 }break;
				 	 case NODE: {
						  iqueuemac->router_states.router_trans_state = R_TRANS_TO_NODE;
					  }break;
					  default:break;
					}
				}
				iqueuemac->need_update = true;
			}else{
		        iqueuemac->router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
		        iqueuemac->need_update = true;
			}
		}
	}
}

void iqueue_mac_router_vtdma_init(iqueuemac_t* iqueuemac){

	/*** switch the radio to the subchannel ***/
	iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->sub_channel_num);

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

	packet_queue_flush(&iqueuemac->rx.queue);
	_dispatch(iqueuemac->rx.dispatch_buffer);

	/*** switch the radio to the public-channel!!! ***/
	iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->public_channel_num);

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

#if 0
	if(iqueue_mac_find_next_tx_neighbor(iqueuemac)){

		/*  stop lpm mode */
		lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;

		iqueuemac->router_states.router_basic_state = R_TRANSMITTING;

		if(iqueuemac->tx.current_neighbour == &iqueuemac->tx.neighbours[0]){
			iqueuemac->router_states.router_trans_state = R_BROADCAST;
		}else{
			switch(iqueuemac->tx.current_neighbour->mac_type){
			  case UNKNOWN: {
				  iqueuemac->router_states.router_trans_state = R_TRANS_TO_UNKOWN;
			  }break;
			  case ROUTER: {
				  iqueuemac->router_states.router_trans_state = R_TRANS_TO_ROUTER;
		 	 }break;
		 	 case NODE: {
				  iqueuemac->router_states.router_trans_state = R_TRANS_TO_NODE;
			  }break;
			  default:break;
			}
		}
		iqueuemac->need_update = true;
		//puts("shuguo: router sends in sleep");
	}else{
		if(iqueuemac->router_states.router_new_cycle == false){
		    /*  enable lpm mode, enter the sleep mode */
		    lpm_prevent_sleep &= ~(IQUEUEMAC_LPM_MASK);
		}
	}
#endif
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
	case R_LISTEN_CP_END: iqueue_mac_router_cp_end(iqueuemac); break;
	//case R_LISTEN_CREATE_BEACON: iqueue_mac_router_create_beacon(iqueuemac); break;
	case R_LISTEN_SEND_BEACON: iqueue_mac_router_send_beacon(iqueuemac); break;
	case R_LISTEN_WAIT_BEACON_FEEDBACK: iqueuemac_router_wait_beacon_feedback(iqueuemac);break;
	case R_LISTEN_VTDMA_INIT: iqueue_mac_router_vtdma_init(iqueuemac); break;
	case R_LISTEN_VTDMA: iqueue_mac_router_vtdma(iqueuemac); break;
	case R_LISTEN_VTDMA_END: iqueue_mac_router_vtdma_end(iqueuemac); break;
	case R_LISTEN_SLEEPING_INIT: iqueue_mac_router_sleep_init(iqueuemac); break;
	case R_LISTEN_SLEEPING: iqueue_mac_router_sleep(iqueuemac); break;
	case R_LISTEN_SLEEPING_END: iqueue_mac_router_sleep_end(iqueuemac); break;
	default: break;
   }
}
//////////////////////////////////////////////////
void iqueue_mac_router_transmit_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_states.router_trans_state)
   {
	case R_TRANS_TO_UNKOWN: iqueuemac_t2u_update(iqueuemac);break; //iqueuemac_router_t2u_update(iqueuemac); break;
	case R_TRANS_TO_ROUTER: iqueuemac_t2r_update(iqueuemac);break;//iqueuemac_router_t2r_update(iqueuemac);
	case R_TRANS_TO_NODE: iqueuemac_t2n_update(iqueuemac);break; //iqueuemac_router_t2n_update(iqueuemac);
	case R_BROADCAST: iqueuemac_device_broadcast_update(iqueuemac);break;
	default: break;
   }

}

void iqueue_mac_router_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->router_states.router_basic_state)
   {
	case R_INIT: iqueuemac_init_update(iqueuemac);break;
	case R_LISTENNING: iqueue_mac_router_listen_update(iqueuemac); break;
	case R_TRANSMITTING: iqueue_mac_router_transmit_update(iqueuemac); break;
	default: break;
   }
}

/******************************node state machinies**********************************/
void iqueue_mac_node_listen_cp_init(iqueuemac_t* iqueuemac){

	iqueuemac->node_states.node_new_cycle = false;

	iqueuemac_trun_on_radio(iqueuemac);

	/******set cp timeout ******/
	iqueuemac_set_timeout(iqueuemac, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);

	iqueuemac->quit_current_cycle = false;

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
/*
	    if(iqueuemac->quit_current_cycle == true){
	    	iqueuemac_trun_off_radio(iqueuemac);
	    	puts("shuguo: node quits this CP");
	    }*/
	}

	/** add a if judgement on rx_started event! don't quit CP if a reception is not completed! **/
	if((iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_CP_END))||(iqueuemac->quit_current_cycle == true)){
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_CP_END);
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}

    /*
	if(iqueuemac->node_states.in_cp_period == false)
	{
		iqueuemac->node_states.node_listen_state = N_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}
	*/
}

void iqueue_mac_node_listen_cp_end(iqueuemac_t* iqueuemac){

	packet_queue_flush(&iqueuemac->rx.queue);
	_dispatch(iqueuemac->rx.dispatch_buffer);

	//puts("shuguo: node end of cp, check queue-length ");
	if(iqueuemac->quit_current_cycle == true){
		iqueuemac->quit_current_cycle = false;
		iqueuemac->node_states.node_listen_state = N_LISTEN_SLEEPING_INIT;
		iqueuemac->need_update = true;
		return;
	}

	if(iqueue_mac_find_next_tx_neighbor(iqueuemac)){
		iqueuemac->node_states.node_basic_state = N_TRANSMITTING;

		if(iqueuemac->tx.current_neighbour == &iqueuemac->tx.neighbours[0]){
			iqueuemac->node_states.node_trans_state = N_BROADCAST;
		}else{
			switch(iqueuemac->tx.current_neighbour->mac_type){
			  case UNKNOWN: iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;break;
			  case ROUTER: {
				  iqueuemac->node_states.node_trans_state = N_TRANS_TO_ROUTER;
				  //puts("shuguo: node turn to send to router ");

		 	 }break;
		 	 case NODE: iqueuemac->node_states.node_trans_state = N_TRANS_TO_NODE;break;
		 	 default:break;
			}
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

#if 0
	if(iqueue_mac_find_next_tx_neighbor(iqueuemac)){
		/*  stop lpm mode */
		lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;

		iqueuemac->node_states.node_basic_state = N_TRANSMITTING;

		if(iqueuemac->tx.current_neighbour == &iqueuemac->tx.neighbours[0]){
			iqueuemac->node_states.node_trans_state = N_BROADCAST;
		}else{
			switch(iqueuemac->tx.current_neighbour->mac_type){
			  case UNKNOWN: iqueuemac->node_states.node_trans_state = N_TRANS_TO_UNKOWN;break;
			  case ROUTER:  iqueuemac->node_states.node_trans_state = N_TRANS_TO_ROUTER;break;
		 	  case NODE: iqueuemac->node_states.node_trans_state = N_TRANS_TO_NODE;break;
		 	 default:break;
			}
		}
		iqueuemac->need_update = true;
		//puts("Shuguo: node start sending in sleep.");
	}else{
		if(iqueuemac->node_states.node_new_cycle == false){
			/*  enable lpm mode, now enter the sleep mode */
			lpm_prevent_sleep &= ~(IQUEUEMAC_LPM_MASK);
			//puts("Shuguo: enter real sleep.");
		} /** No need_update!! **/
	}
#endif

	if(iqueuemac->node_states.node_new_cycle == true){
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


void iqueue_mac_node_transmit_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_trans_state)
   {
	case N_TRANS_TO_UNKOWN: iqueuemac_t2u_update(iqueuemac);break; //iqueue_mac_node_t2u_update(iqueuemac); break;
	case N_TRANS_TO_ROUTER: iqueuemac_t2r_update(iqueuemac);break; //iqueue_mac_node_t2r_update(iqueuemac); break;
	case N_TRANS_TO_NODE: iqueuemac_t2n_update(iqueuemac); break; //iqueuemac_node_t2n_update(iqueuemac); break;
	case N_BROADCAST: iqueuemac_device_broadcast_update(iqueuemac);break;
	default: break;
   }

}

void iqueue_mac_node_update(iqueuemac_t* iqueuemac){

	switch(iqueuemac->node_states.node_basic_state)
   {
	case N_INIT: iqueuemac_node_init_update(iqueuemac);break;
	case N_LISTENNING: iqueue_mac_node_listen_update(iqueuemac); break;
	case N_TRANSMITTING: iqueue_mac_node_transmit_update(iqueuemac); break;
	default: break;
   }
}

void iqueue_mac_update(iqueuemac_t* iqueuemac){

	if(iqueuemac->mac_type == ROUTER){
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
static void _event_cb(netdev2_t *dev, netdev2_event_t event)
{
    gnrc_netdev2_t *gnrc_netdev2 = (gnrc_netdev2_t*) dev->context;

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
    dev->context = (void*) gnrc_netdev2;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);
    /***************************************************************************/

    iqueuemac.mac_type = MAC_TYPE;

    xtimer_sleep(3);

    iqueuemac_init(&iqueuemac);

    uint32_t seed;
    seed = (uint32_t)iqueuemac.own_addr.addr[0];

    random_init(seed);

    //rtt_handler(IQUEUEMAC_EVENT_RTT_START);

    iqueuemac.need_update = true;

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

        while(iqueuemac.need_update == true)  //&&(iqueuemac.duty_cycle_started == true)
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

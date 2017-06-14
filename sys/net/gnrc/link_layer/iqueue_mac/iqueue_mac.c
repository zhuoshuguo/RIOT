/*
 * Copyright (C) 2016 INRIA
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
#include "msg.h"
#include "thread.h"
#include "random.h"
#include <timex.h>
#include <periph/rtt.h>
#include "net/gnrc.h"
#include "net/gnrc/nettype.h"
#include "net/netdev.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/mac/internal.h"
#include "net/gnrc/iqueue_mac/iqueue_mac.h"
#include "net/gnrc/iqueue_mac/iqueuemac_types.h"
#include "net/gnrc/netdev/ieee802154.h"
#include "include/iqueuemac_internal.h"
#include "net/gnrc/iqueue_mac/timeout.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#if defined(MODULE_OD) && ENABLE_DEBUG
#include "od.h"
#endif


#define LOG_LEVEL LOG_DEBUG
#include "log.h"

#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_INFO
#undef LOG_DEBUG

#define LOG_ERROR(...) LOG(LOG_ERROR, "ERROR: [iqmac] " __VA_ARGS__)
#define LOG_WARNING(...) LOG(LOG_WARNING, "WARNING: [iqmac] " __VA_ARGS__)
#define LOG_INFO(...) LOG(LOG_INFO, "INFO: [iqmac] " __VA_ARGS__)
#define LOG_DEBUG(...) LOG(LOG_DEBUG, "DEBUG: [iqmac] " __VA_ARGS__)


#define NETDEV_NETAPI_MSG_QUEUE_SIZE 8

kernel_pid_t iqueuemac_pid;
//static iqueuemac_t iqueuemac;

void iqueuemac_init(gnrc_netdev_t *gnrc_netdev)
{

	gnrc_netdev->iqueuemac.own_addr.len = gnrc_netdev->dev->driver->get(gnrc_netdev->dev,
	                                                                     NETOPT_ADDRESS,
	                                                                     gnrc_netdev->iqueuemac.own_addr.addr,
	                                                                     sizeof(gnrc_netdev->iqueuemac.own_addr.addr));

	//printf("iqueuemac: iqueuemac's own addrs is: %d, %d . \n ", iqueuemac->own_addr.addr[1], iqueuemac->own_addr.addr[0]);

	if(gnrc_netdev->iqueuemac.mac_type == ROUTER)
	{
		gnrc_netdev->iqueuemac.router_states.router_basic_state = R_INIT;  //R_LISTENNING;
		gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_PREPARE;

		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_INIT;
		gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_UNKOWN;

		gnrc_netdev->iqueuemac.router_states.router_new_cycle = false;

		gnrc_netdev->iqueuemac.rx.router_vtdma_mana.sub_channel_seq = 26;

		gnrc_netdev->iqueuemac.router_states.subchannel_occu_flags = 0;

		/*** set the father-router as itself ***/
		gnrc_netdev->iqueuemac.father_router_addr.len = gnrc_netdev->iqueuemac.own_addr.len;
		memcpy(gnrc_netdev->iqueuemac.father_router_addr.addr,
				gnrc_netdev->iqueuemac.own_addr.addr,
				gnrc_netdev->iqueuemac.own_addr.len);

		/*** initiate the sub_channel_num  ***/
		//uint16_t random_channel = iqueuemac->own_addr.addr[0] % 15;
		//iqueuemac->sub_channel_num = 11 + random_channel;
		gnrc_netdev->iqueuemac.sub_channel_num = 13;

	}

	gnrc_netdev->iqueuemac.pub_channel_1 = 26;
	gnrc_netdev->iqueuemac.pub_channel_2 = 11;
	gnrc_netdev->iqueuemac.cur_pub_channel = gnrc_netdev->iqueuemac.pub_channel_1;

	gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_BROADCAST_INIT;
	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2n_state = DEVICE_T2N_WAIT_CP_INIT;
	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;
	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;

	gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;

	/* Enable RX-start and TX-started and TX-END interrupts  */
    netopt_enable_t enable = NETOPT_ENABLE;
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_RX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_START_IRQ, &enable, sizeof(enable));
    gnrc_netdev->dev->driver->set(gnrc_netdev->dev, NETOPT_TX_END_IRQ, &enable, sizeof(enable));

    /* Enable preloading, so packet will only be sent when netdev state will be
     * set to NETOPT_STATE_TX */
    //iqueuemac->netdev->dev->driver->set(iqueuemac->netdev->dev, NETOPT_PRELOADING, &enable, sizeof(enable));

    /* Initialize broadcast sequence number. This at least differs from board
         * to board */
    gnrc_netdev->iqueuemac.tx.broadcast_seq = gnrc_netdev->iqueuemac.own_addr.addr[0];

    /* First neighbour queue is supposed to be broadcast queue */
    //int broadcast_queue_id = _alloc_neighbour(&gnrc_netdev->iqueuemac);
    //assert(broadcast_queue_id == 0);

    /* Setup broadcast tx queue */
    //uint8_t broadcast_addr[] = {0xff, 0xff};
    //_init_neighbour(_get_neighbour(gnrc_netdev->iqueuemac, 0), broadcast_addr, sizeof(broadcast_addr));

    /* Initialize receive packet queue
    packet_queue_init(&(iqueuemac->rx.queue),
    		          iqueuemac->rx._queue_nodes,
                      (sizeof(iqueuemac->rx._queue_nodes) / sizeof(packet_queue_node_t)));
     */

    /* Reset all timeouts just to be sure */
    iqueuemac_reset_timeouts(&gnrc_netdev->iqueuemac);

    gnrc_netdev->iqueuemac.packet_received = false;
    gnrc_netdev->iqueuemac.need_update = false;
    gnrc_netdev->iqueuemac.duty_cycle_started = false;
    gnrc_netdev->iqueuemac.quit_current_cycle = false;
    gnrc_netdev->iqueuemac.send_beacon_fail = false;
    gnrc_netdev->iqueuemac.rx_memory_full = false;
    gnrc_netdev->iqueuemac.phase_backoff = false;

    gnrc_netdev->iqueuemac.rx.check_dup_pkt.queue_head = 0;
    gnrc_netdev->iqueuemac.tx.last_tx_neighbor_id = 0;

	netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
	device_state->seq = gnrc_netdev->iqueuemac.own_addr.addr[0];

	for(int i=0;i<IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE;i++){
		gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
	}

}

static void rtt_cb(void* arg)
{
    msg_t msg;
    msg.content.value = ((uint32_t) arg ) & 0xffff;
    msg.type = IQUEUEMAC_EVENT_RTT_TYPE;
    msg_send(&msg, iqueuemac_pid);

    if (sched_context_switch_request) {
        thread_yield();
    }
}


void rtt_handler(uint32_t event, gnrc_netdev_t *gnrc_netdev)
{
    uint32_t alarm;
    switch(event & 0xffff)
    {
      /*******************************Router RTT management***************************/
      case IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE:{

          if(gnrc_netdev->iqueuemac.duty_cycle_started == false){
        	  gnrc_netdev->iqueuemac.duty_cycle_started = true;
        	  rtt_clear_alarm();
        	  /*** record the starting phase of iQueuemac ***/
        	  gnrc_netdev->iqueuemac.last_wakeup = rtt_get_counter();
          }else{
        	  gnrc_netdev->iqueuemac.last_wakeup = rtt_get_alarm(); //rtt_get_counter();
        	  gnrc_netdev->iqueuemac.router_states.router_new_cycle = true;
        	  // iqueuemac_stop_lpm();
          }

          //lpm_prevent_sleep |= IQUEUEMAC_LPM_MASK;


          //alarm = RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          alarm = gnrc_netdev->iqueuemac.last_wakeup + RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US);
          rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);

          update_neighbor_pubchan(gnrc_netdev);

          gnrc_netdev->iqueuemac.need_update = true;

      }break;

      /********************************************************/
      case IQUEUEMAC_EVENT_RTT_START:{

    	  if(gnrc_netdev->iqueuemac.mac_type == ROUTER)
    	  {
    		  gnrc_netdev->iqueuemac.duty_cycle_started = true;
    		  gnrc_netdev->iqueuemac.need_update = true;
    		  /*** set a random starting time here in the future, thus to avoid the same phase for neighbor devices.
    		  puts("iqueuemac: router starting duty cycling.");

    	      alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
    	      rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);  ***/
    	  }else{
    		  //puts("iqueuemac: node starting duty cycling.");
    		  /*** set a random starting time here in the future, thus to avoid the same phase for neighbor devices. ***/
     	      alarm = rtt_get_counter() + RTT_US_TO_TICKS(IQUEUEMAC_CP_DURATION_US);
     	      rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_N_NEW_CYCLE);
    	  }
      }break;

      default: break;

      }

}

void iqueuemac_phase_backoff(gnrc_netdev_t *gnrc_netdev)
{
	uint32_t alarm;
	/*** execute phase backoff for avoiding CP overlap. ***/
	rtt_clear_alarm();
    alarm = gnrc_netdev->iqueuemac.last_wakeup +
            RTT_US_TO_TICKS(IQUEUEMAC_SUPERFRAME_DURATION_US) +
            gnrc_netdev->iqueuemac.backoff_phase_ticks;
    rtt_set_alarm(alarm, rtt_cb, (void*) IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE);
    gnrc_netdev->iqueuemac.phase_changed = true;

    uint32_t backoff_us;
    backoff_us = RTT_TICKS_TO_US(gnrc_netdev->iqueuemac.backoff_phase_ticks);
    printf("bp %lu\n", backoff_us);
}


/****************** Device (both router and node) broadcast state machines*****/

void iqueuemac_device_broadcast_init(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);

	iqueuemac_trun_on_radio(gnrc_netdev);

	/*** assemble broadcast packet ***/
	gnrc_pktsnip_t* pkt = gnrc_netdev->iqueuemac.tx.tx_packet;

	iqueuemac_frame_broadcast_t iqueuemac_broadcast_hdr;
	iqueuemac_broadcast_hdr.header.type = FRAMETYPE_BROADCAST;
	iqueuemac_broadcast_hdr.seq_nr = gnrc_netdev->iqueuemac.tx.broadcast_seq;

	pkt->next = gnrc_pktbuf_add(pkt->next, &iqueuemac_broadcast_hdr, sizeof(iqueuemac_broadcast_hdr), GNRC_NETTYPE_IQUEUEMAC);
	if(pkt == NULL) {
		puts("iqueuemac: pktbuf add failed in iqueuemac_device_broadcast_init.");
		//relase the broadcast pkt and go to listen state??.
	}
	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_FINISH, IQUEUEMAC_SUPERFRAME_DURATION_US);

	gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_SEND_BROADCAST;
	gnrc_netdev->iqueuemac.need_update = true;

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
}

void iqueuemac_device_send_broadcast(gnrc_netdev_t *gnrc_netdev){

	/* if rx start, wait until rx is completed. */
	if(gnrc_netdev->iqueuemac.rx_started == true){
		return;
	}

	/* when rx completed, will reach here */
	if(gnrc_netdev->iqueuemac.packet_received == true){
		;//iqueue_router_broadcast_receive_packet_process(iqueuemac);; /// to be filt in!!
	}

	/***  disable auto-ack ***/
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_DISABLE);

	gnrc_pktbuf_hold(gnrc_netdev->iqueuemac.tx.tx_packet,1);

	iqueuemac_send(gnrc_netdev, gnrc_netdev->iqueuemac.tx.tx_packet, NETOPT_DISABLE);

	/* Enable Auto ACK again for data reception */
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);

	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_INTERVAL, IQUEUEMAC_BROADCAST_INTERVAL_US);

	gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_WAIT_BROADCAST_FEEDBACK;
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_device_wait_broadcast_feedback(gnrc_netdev_t *gnrc_netdev){

	/* if rx start, wait until rx is completed. */
	if(gnrc_netdev->iqueuemac.rx_started == true){
		return;
	}

	/* when rx completed, will reach here */
	if(gnrc_netdev->iqueuemac.packet_received == true){
		;//iqueue_router_broadcast_receive_packet_process(iqueuemac);; /// to be filt in!!
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_FINISH)){

		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_BROADCAST_INTERVAL);
		gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
		gnrc_netdev->iqueuemac.tx.tx_packet = NULL;

		gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_BROADCAST_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_INTERVAL)){

		gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_SEND_BROADCAST;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_device_broadcast_end(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_BROADCAST_INTERVAL);
	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_BROADCAST_FINISH);

	if(gnrc_netdev->iqueuemac.tx.tx_packet){
		gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
		gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
	}
	gnrc_netdev->tx.current_neighbor = NULL;

	gnrc_netdev->iqueuemac.device_states.device_broadcast_state = DEVICE_BROADCAST_INIT;

	if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
	    /*********** judge and update the states before switch back to CP listening period   ***********/
		gnrc_netdev->iqueuemac.router_states.router_basic_state = R_LISTENNING;
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING;
		gnrc_netdev->iqueuemac.router_states.router_new_cycle = false;

	    iqueuemac_trun_off_radio(gnrc_netdev);

	    //puts("iqueuemac: router is in broadcast end, switching back to sleeping period");
	}
	gnrc_netdev->iqueuemac.need_update = true;
}


void iqueuemac_device_broadcast_update(gnrc_netdev_t *gnrc_netdev){

	switch(gnrc_netdev->iqueuemac.device_states.device_broadcast_state)
	{
	 case DEVICE_BROADCAST_INIT: iqueuemac_device_broadcast_init(gnrc_netdev);break;
	 case DEVICE_SEND_BROADCAST: iqueuemac_device_send_broadcast(gnrc_netdev); break;
	 case DEVICE_WAIT_BROADCAST_FEEDBACK: iqueuemac_device_wait_broadcast_feedback(gnrc_netdev); break;
	 case DEVICE_BROADCAST_END: iqueuemac_device_broadcast_end(gnrc_netdev);break;
	 default: break;
	}
}


/****************** iQueue-MAC transmission to node state machines *****/

void iqueuemac_init_prepare(gnrc_netdev_t *gnrc_netdev){

	rtt_clear_alarm();

	uint32_t listen_period;

	listen_period = random_uint32_range(0, IQUEUEMAC_SUPERFRAME_DURATION_US);
	listen_period = (IQUEUEMAC_SUPERFRAME_DURATION_US*11/10) + listen_period + IQUEUEMAC_WAIT_RTT_STABLE_US;

	gnrc_netdev->iqueuemac.quit_current_cycle = false;
	gnrc_netdev->iqueuemac.router_states.init_retry = false;
	gnrc_netdev->iqueuemac.router_states.subchannel_occu_flags = 0;

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

	/******set TIMEOUT_COLLECT_BEACON_END timeout ******/
	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_COLLECT_BEACON_END, listen_period);

	gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_COLLECT_BEACONS;
	gnrc_netdev->iqueuemac.need_update = true;

}

void iqueuemac_init_collec_beacons(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.packet_received == true){
		gnrc_netdev->iqueuemac.packet_received = false;
	   	iqueuemac_packet_process_in_init(gnrc_netdev);
	}

	if(gnrc_netdev->iqueuemac.quit_current_cycle == true){
		iqueuemac_trun_off_radio(gnrc_netdev);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_COLLECT_BEACON_END);
		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_FINISH, IQUEUEMAC_SUPERFRAME_DURATION_US);
		gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_WAIT_BUSY_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	/*** it seems that this "init_retry" procedure is unnecessary here!! maybe delete it in the future ***/
	if(gnrc_netdev->iqueuemac.router_states.init_retry == true){
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_COLLECT_BEACON_END);
		gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
		gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_COLLECT_BEACON_END)){
		iqueuemac_init_choose_subchannel(gnrc_netdev);
		gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_ANNOUNCE_SUBCHANNEL;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_init_wait_busy_end(gnrc_netdev_t *gnrc_netdev){

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_BROADCAST_FINISH)){
		iqueuemac_trun_on_radio(gnrc_netdev);
		gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_init_announce_subchannel(gnrc_netdev_t *gnrc_netdev){


	//set csma retry number here??
	iqueuemac_send_announce(gnrc_netdev, NETOPT_ENABLE);

	gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_WAIT_ANNOUNCE_FEEDBACK;
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_init_wait_announce_feedback(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.tx.tx_finished == true){

		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/
		if(gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_SUCCESS){
			gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
			gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_END;
			gnrc_netdev->iqueuemac.need_update = true;
			return;
		}else{ //if(iqueuemac->tx.tx_feedback == TX_FEEDBACK_BUSY)
			gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_PREPARE;
			gnrc_netdev->iqueuemac.need_update = true;
		}
	}
}

void iqueuemac_init_end(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_COLLECT_BEACON_END);

	gnrc_netdev->iqueuemac.router_states.router_init_state = R_INIT_PREPARE;
	/*** switch to duty-cycle operation ***/
	gnrc_netdev->iqueuemac.router_states.router_basic_state = R_LISTENNING;
	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_INIT;

	//puts("router random ends.");
	/*** start duty-cycle ***/
	gnrc_netdev->iqueuemac.duty_cycle_started = false;
	rtt_handler(IQUEUEMAC_EVENT_RTT_R_NEW_CYCLE, gnrc_netdev);
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_init_update(gnrc_netdev_t *gnrc_netdev){

	switch(gnrc_netdev->iqueuemac.router_states.router_init_state)
	{
		case R_INIT_PREPARE: iqueuemac_init_prepare(gnrc_netdev);break;
		case R_INIT_COLLECT_BEACONS: iqueuemac_init_collec_beacons(gnrc_netdev);break;
		case R_INIT_WAIT_BUSY_END: iqueuemac_init_wait_busy_end(gnrc_netdev);break;
		case R_INIT_ANNOUNCE_SUBCHANNEL: iqueuemac_init_announce_subchannel(gnrc_netdev);break;
		case R_INIT_WAIT_ANNOUNCE_FEEDBACK: iqueuemac_init_wait_announce_feedback(gnrc_netdev);break;
		case R_INIT_END: iqueuemac_init_end(gnrc_netdev);break;
		default: break;
	}
}

/****************** iQueuemac: Transmit to router *****/
void iqueuemac_t2r_init(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_trun_off_radio(gnrc_netdev);

	/* turn to the neighbor's public channel */
	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->tx.current_neighbor->pub_chanseq);

	gnrc_netdev->iqueuemac.quit_current_cycle = false;

	/* set timer for the targeted router! */
	uint32_t wait_phase_duration;

	wait_phase_duration = _ticks_until_phase(gnrc_netdev, gnrc_netdev->tx.current_neighbor->cp_phase);
	wait_phase_duration = RTT_TICKS_TO_US(wait_phase_duration); // + IQUEUEMAC_WAIT_CP_SECUR_GAP_US;
	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_CP, wait_phase_duration);

	/*** flush the rx-queue here to reduce possible buffered packet in RIOT!! ***/
	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP;
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_t2r_wait_cp(gnrc_netdev_t *gnrc_netdev){

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_CP)){

		/* set up auto-ack for packet reception! */
		iqueuemac_set_autoack(gnrc_netdev, NETOPT_DISABLE);
		iqueuemac_set_ack_req(gnrc_netdev, NETOPT_ENABLE);

		iqueuemac_trun_on_radio(gnrc_netdev);
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_CP;

		/* set up auto-ack for packet reception! */
		//iqueuemac_set_autoack(iqueuemac, NETOPT_ENABLE);

		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_t2r_trans_in_cp(gnrc_netdev_t *gnrc_netdev){

	//to-do: should we add a rx-start security check as following?
	/* if rx start, wait until rx is completed.
	if(iqueuemac->rx_started == true){
		return;
	}

	if(iqueuemac->packet_received == true){
	   	iqueuemac->packet_received = false;
	   	iqueuemac_packet_process_in_t-2-r(iqueuemac);
	}

	// to be filt in! for example, add receive other's broadcast and preamble handle codes here!!!
	if(iqueuemac->quit_current_cycle == true){
		; //return;
	}*/

	if(gnrc_netdev->iqueuemac.tx.no_ack_contuer > 0){
		netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
		device_state->seq = gnrc_netdev->iqueuemac.tx.tx_seq;
	}

	/***  do not disable auto-ack here, we need auto-ack for data transmission and possible retransmission ***/
	/******Use CSMA here, and send_packet() will release the pkt itself !!!!******/
	int res;
	res = iqueuemac_send_data_packet(gnrc_netdev, NETOPT_ENABLE);
	if(res < 0){
		printf("t2r %d, drop pkt.\n", res);

		gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
		if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL){
			gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
			gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
		}

		gnrc_netdev->tx.current_neighbor = NULL;
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	//iqueuemac->tx.tx_packet = NULL;

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CPTRANS_FEEDBACK;
	gnrc_netdev->iqueuemac.need_update = false;

}

void iqueuemac_t2r_wait_cp_transfeedback(gnrc_netdev_t *gnrc_netdev){

	/* it seems that we don't need a rx-start security check here, since radio can't receive pkt when it is being in transmitting*/

	if(gnrc_netdev->iqueuemac.tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
				 * namely, completed, to ensure router gets the data correctly***/
		switch(gnrc_netdev_get_tx_feedback(gnrc_netdev)){

			case TX_FEEDBACK_SUCCESS:{
				/*** first release the pkt ***/
				gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
				gnrc_netdev->iqueuemac.tx.tx_packet = NULL;

				gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;

				/*** if has pending pkt, join the vTDMA period, first wait receiver's beacon ***/

				if(gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0){
					gnrc_netdev->iqueuemac.tx.vtdma_para.get_beacon = false;
					iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_BEACON, IQUEUEMAC_WAIT_BEACON_TIME_US);
					// need to flush the rx-queue ??
					gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_BEACON;
				}else{
					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				gnrc_netdev->iqueuemac.need_update = true;
			}break;

			case TX_FEEDBACK_BUSY:
			/*** if NOACK, regards it as phase-lock failed, mark the destination as unknown will try t-2-u next time. ***/
			case TX_FEEDBACK_NOACK:
			default:{
				/* this is for debug, delete when formal iqueuemac version is release! */
				/* delete this turn-off radio when formal iqueuemac version is release!
				 * since it (turn-off radio func here) is mainly for debug */
				iqueuemac_trun_off_radio(gnrc_netdev);
				if(gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_BUSY) {
				    puts("t2r:busy");
				}else if (gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_NOACK){
					puts("t2r:noack");
				}
				gnrc_netdev->iqueuemac.tx.no_ack_contuer ++;

				netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
				gnrc_netdev->iqueuemac.tx.tx_seq = device_state->seq - 1;

				if(gnrc_netdev->iqueuemac.tx.no_ack_contuer >= IQUEUEMAC_REPHASELOCK_THRESHOLD){
					//iqueuemac->tx.no_ack_contuer = 0xFF; //0;

					printf("t2r:noack %d, go t2u\n",gnrc_netdev->iqueuemac.tx.no_ack_contuer);
					gnrc_netdev->tx.current_neighbor->mac_type = UNKNOWN;

					gnrc_netdev->iqueuemac.tx.t2u_retry_contuer = 0;

					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END; //DEVICE_T2R_RE_PHASE_LOCK_PREPARE;

				}else{
					printf("t2r:noack %d, go t2r\n",gnrc_netdev->iqueuemac.tx.no_ack_contuer);
					/* go to t-2-r end and try t-2-r again. */
					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				gnrc_netdev->iqueuemac.need_update = true;
				/*** pkt trans failed, don't release the pkt here, retry to transmit in t-2-u procedure. ***/

			}break;
#if 0
			default:{
				puts("t2r default error, push pkt");

				iqueuemac->tx.no_ack_contuer = 0;

		        /* save payload pointer */
		        gnrc_pktsnip_t* payload = iqueuemac->tx.tx_packet->next->next;

		        /* remove iqueuemac header */
		        iqueuemac->tx.tx_packet->next->next = NULL;
		        gnrc_pktbuf_release(iqueuemac->tx.tx_packet->next);

		        /* make append payload after netif header again */
		        iqueuemac->tx.tx_packet->next = payload;

		        /* queue the pkt for transmission in next cycle */
		        if(_queue_tx_packet(iqueuemac, iqueuemac->tx.tx_packet) == false){
		        	puts("Push pkt failed in t2r");
		        }
		        iqueuemac->tx.tx_packet = NULL;

				iqueuemac->device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				iqueuemac->need_update = true;
			}break;
#endif
		}
	}
}

void iqueuemac_t2r_re_phase_lock_prepare(gnrc_netdev_t *gnrc_netdev){

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RE_PHASE_LOCK)){
		/*** leaving t-2-r, so initiate the state ***/
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;

		if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
			gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_UNKOWN;
		}

		gnrc_netdev->iqueuemac.need_update = true;
	}

}

void iqueuemac_t2r_wait_beacon(gnrc_netdev_t *gnrc_netdev){

    if(gnrc_netdev->iqueuemac.packet_received == true){
    	gnrc_netdev->iqueuemac.packet_received = false;
    	iqueuemac_wait_beacon_packet_process(gnrc_netdev);
    }

    if(gnrc_netdev->iqueuemac.quit_current_cycle == true)
    {
    	/* if we are here, means tx_packet is NULL, so just need to release neighbor also. */
    	//iqueuemac->tx.current_neighbour = NULL;

    	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_BEACON);
    	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
    	gnrc_netdev->iqueuemac.need_update = true;
		return;
    }

    if(gnrc_netdev->iqueuemac.tx.vtdma_para.get_beacon == true){

    	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_BEACON);

    	if(gnrc_netdev->iqueuemac.tx.vtdma_para.slots_num > 0){

    		/*** switch the radio to the sub-channel ***/
    		uint16_t tartget_sub_channel_seq;
    		tartget_sub_channel_seq = (uint16_t)gnrc_netdev->iqueuemac.tx.vtdma_para.sub_channel_seq;
    		iqueuemac_turn_radio_channel(gnrc_netdev, tartget_sub_channel_seq);

    		if(gnrc_netdev->iqueuemac.tx.vtdma_para.slots_position > 0){
    			/*** wait for the finish of switching channel !!! and then turn off the radio to save power ***/
    			iqueuemac_trun_off_radio(gnrc_netdev);

    			uint32_t wait_slots_duration;
    			wait_slots_duration = gnrc_netdev->iqueuemac.tx.vtdma_para.slots_position * IQUEUEMAC_VTDMA_SLOT_SIZE_US;
    			iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_OWN_SLOTS, wait_slots_duration);

    			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_OWN_SLOTS;
    			gnrc_netdev->iqueuemac.need_update = true;
    		}else{// be the first sender in vtdma

    			gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&(gnrc_netdev->tx.current_neighbor->queue));

    			if(pkt != NULL){
    				gnrc_netdev->iqueuemac.tx.tx_packet = pkt;
    				gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
    			}else{
    				puts("iqueueMAC-Error: NUll pktbuf!");
    				gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
    			}

    			gnrc_netdev->iqueuemac.need_update = true;
    		}
    	}else{/*** no slots get allocated, go to t-2-r end ***/
    		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
    		gnrc_netdev->iqueuemac.need_update = true;
    	}
    	return;
    }

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_BEACON)){
		gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
		puts("t2r:no beacon");
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_t2r_wait_own_slots(gnrc_netdev_t *gnrc_netdev){

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_OWN_SLOTS)){

		iqueuemac_trun_on_radio(gnrc_netdev);

		gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&(gnrc_netdev->tx.current_neighbor->queue));

		if(pkt != NULL){
			gnrc_netdev->iqueuemac.tx.tx_packet = pkt;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
		}else{
			puts("iqueueMAC-Error: NUll pktbuf, drop!");
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		}
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_t2r_trans_in_slots(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.tx.vtdma_para.slots_num > 0){

		/**** Delete the pkt no matter the transmission is success or not !!! ****/
		/**** Or, only delete the pkt when the feedback shows good !!! ****/
		//gnrc_pktsnip_t *pkt = packet_queue_head(&(iqueuemac->tx.current_neighbour.queue));

		/* since it is now on sub-channel, so there is possibility that this sender will receive a broadcast, preamble or data pkt towards it.
		 * so, no rx security check needed here. and no need to disable autoack here.
		 *  */

		if(gnrc_netdev->iqueuemac.tx.no_ack_contuer > 0){
			netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
			device_state->seq = gnrc_netdev->iqueuemac.tx.tx_seq;
		}

		/***  do not disable auto-ack here, we need auto-ack for data transmission and possible retransmission ***/
		/******disable CSMA here, and iqueuemac_send_data_packet() will release the pkt itself !!!!******/
		int res;
		res = iqueuemac_send_data_packet(gnrc_netdev, NETOPT_DISABLE);
		if(res < 0){
			printf("vtdma %d, drop pkt\n", res);

			if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL){
				gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
				gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
			}
			gnrc_netdev->tx.current_neighbor = NULL;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
			gnrc_netdev->iqueuemac.need_update = true;
			return;
		}

		gnrc_netdev->iqueuemac.tx.vtdma_para.slots_num --;
		//iqueuemac->tx.tx_packet = NULL;

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_VTDMATRANS_FEEDBACK;
		gnrc_netdev->iqueuemac.need_update = true;

	}else{/*** here means the slots have been used up !!! ***/
		/****  switch back to the public channel ****/
		//puts("v-end1");
		//iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->cur_pub_channel);
		if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL) {
			gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
			gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
			puts("v3:drop pkt");
			gnrc_netdev->tx.current_neighbor = NULL;
		}

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_t2r_wait_vtdma_transfeedback(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.tx.tx_finished == true){
		/*** add another condition here in the furture: the tx-feedback must be ACK-got,
		 * namely, completed, to ensure router gets the data correctly***/

		switch(gnrc_netdev_get_tx_feedback(gnrc_netdev)){

			case TX_FEEDBACK_SUCCESS:{
				//puts("v");
				/*** first release the pkt ***/
				gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
				gnrc_netdev->iqueuemac.tx.tx_packet = NULL;

				gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;

				/*** if the sender has pending pkt, continue vTDMA transmission ***/
				if ((gnrc_netdev->iqueuemac.tx.vtdma_para.slots_num > 0) &&
				   (gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0)) {
					gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.current_neighbor->queue);

					if(pkt != NULL){
						gnrc_netdev->iqueuemac.tx.tx_packet = pkt;
						gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;
					}else{
						puts("iqueueMAC-Error: NUll pktbuf!");
						gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
					}
				}else{
					/****  vtdma period ends, switch back to the public channel ****/
					//puts("v-end2");
					//iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->cur_pub_channel);

					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				gnrc_netdev->iqueuemac.need_update = true;
			}break;

			/*** if BUSY and NOACK, regards it as busy channel ***/
			case TX_FEEDBACK_BUSY:
			case TX_FEEDBACK_NOACK:
			default:{

				gnrc_netdev->iqueuemac.tx.no_ack_contuer = 1;

				netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
				gnrc_netdev->iqueuemac.tx.tx_seq = device_state->seq - 1;

				/*** do not release the pkt here, continue sending the same pkt ***/
				if(gnrc_netdev->iqueuemac.tx.vtdma_para.slots_num > 0){
					puts("v1");

					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_IN_VTDMA;

				}else{ /* if no slots for sending, queue the pkt for retry in next cycle */
					puts("v2");

		            /****  vtdma period ends, switch back to the public channel ****/
					//iqueuemac_turn_radio_channel(iqueuemac, iqueuemac->cur_pub_channel);
					gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_TRANS_END;
				}
				gnrc_netdev->iqueuemac.need_update = true;

			}break;
		}
	}
}

void iqueuemac_t2r_end(gnrc_netdev_t *gnrc_netdev){

	/* note that, if the node reach here from t-2-U, then, no_ack_contuer must be 0,
	 * since t-2-r will set it to 0 only when it totally ends.*/

	/* if t-2-r success or IQUEUEMAC_REPHASELOCK_THRESHOLD has been reached,
	 * clean the tx current_neighbour address. */
	if((gnrc_netdev->iqueuemac.tx.tx_packet != NULL) && (gnrc_netdev->iqueuemac.tx.no_ack_contuer == 0)){
		puts("t2r:drop pkt");
		gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
		gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
	}

	/* if IQUEUEMAC_REPHASELOCK_THRESHOLD hasn't been reached yet,
	 * don't clean the tx current_neighbour address. The node will try t-2-r again.
	 * otherwise, if no_ack_contuer is 0, clean current_neighbour. Prepare for t-2-u. */
	if(gnrc_netdev->iqueuemac.tx.no_ack_contuer == 0)
	{
		gnrc_netdev->tx.current_neighbor = NULL;
	}

	/*** clear all timeouts ***/
	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_CP);
	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_BEACON);
	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_OWN_SLOTS);

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_CP_INIT;

	if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
		/* if phase has been changed, figure out the related phase of tx-neighbors. */
		if(gnrc_netdev->iqueuemac.phase_changed == true){
			iqueuemac_figure_tx_neighbor_phase(gnrc_netdev);
		}

		/*********** judge and update the states before switch back to CP listening period   ***********/
		gnrc_netdev->iqueuemac.router_states.router_basic_state = R_LISTENNING;
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING;
		gnrc_netdev->iqueuemac.router_states.router_new_cycle = false;

		iqueuemac_trun_off_radio(gnrc_netdev);
		//puts("iqueuemac: router (device) is in t-2-r end.");

		/* set up auto-ack for packet reception! */
		iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);
	}
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_t2r_update(gnrc_netdev_t *gnrc_netdev)
{
	switch(gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state)
	{
	 case DEVICE_T2R_WAIT_CP_INIT: iqueuemac_t2r_init(gnrc_netdev);break;
     case DEVICE_T2R_WAIT_CP: iqueuemac_t2r_wait_cp(gnrc_netdev); break;
	 case DEVICE_T2R_TRANS_IN_CP: iqueuemac_t2r_trans_in_cp(gnrc_netdev); break;
	 case DEVICE_T2R_WAIT_CPTRANS_FEEDBACK: iqueuemac_t2r_wait_cp_transfeedback(gnrc_netdev); break;
	 case DEVICE_T2R_RE_PHASE_LOCK_PREPARE: iqueuemac_t2r_re_phase_lock_prepare(gnrc_netdev); break;
	 case DEVICE_T2R_WAIT_BEACON: iqueuemac_t2r_wait_beacon(gnrc_netdev);break;
	 case DEVICE_T2R_WAIT_OWN_SLOTS:iqueuemac_t2r_wait_own_slots(gnrc_netdev); break;
	 case DEVICE_T2R_TRANS_IN_VTDMA:iqueuemac_t2r_trans_in_slots(gnrc_netdev); break;
	 case DEVICE_T2R_WAIT_VTDMATRANS_FEEDBACK:iqueuemac_t2r_wait_vtdma_transfeedback(gnrc_netdev);break;
	 case DEVICE_T2R_TRANS_END:iqueuemac_t2r_end(gnrc_netdev);break;
	 default: break;
	}
}


/****************** device state machines - Transmit to Unknown *****/
void iqueuemac_t2u_send_preamble_init(gnrc_netdev_t *gnrc_netdev){

	puts("r");
	/* in case that rx_started was left as true during last t-2-u ending, so set it to false. */
	gnrc_netdev->iqueuemac.rx_started = false;

	/** since t-2-u is right following beacon, so the radio is still on, so we don't need to turn on it again. **/
	//iqueuemac_trun_on_radio(iqueuemac);

	gnrc_netdev->iqueuemac.quit_current_cycle = false;
	gnrc_netdev->iqueuemac.packet_received = false;
	gnrc_netdev->iqueuemac.tx.preamble_sent = 0;
	gnrc_netdev->iqueuemac.tx.got_preamble_ack = false;
	gnrc_netdev->iqueuemac.rx_memory_full = false;

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_PREPARE;
	gnrc_netdev->iqueuemac.need_update = true;

	/*** Must disable auto-ack here!! need this sentence!! becuase it may due to other's unknwn reasons,
	 * that this snode may has autoACK here. ***/
	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.pub_channel_1);
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_DISABLE);

	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.pub_channel_2);
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_DISABLE);

	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.pub_channel_1);
	gnrc_netdev->iqueuemac.tx.t2u_on_public_1 = true;

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
}

void iqueuemac_t2u_send_preamble_prepare(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);

	if(gnrc_netdev->iqueuemac.tx.preamble_sent != 0){
	    if(gnrc_netdev->iqueuemac.tx.t2u_on_public_1 == true){
		    iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.pub_channel_2);
		    gnrc_netdev->iqueuemac.tx.t2u_on_public_1 = false;
	    }else{
	    	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.pub_channel_1);
	    	gnrc_netdev->iqueuemac.tx.t2u_on_public_1 = true;
	    }
	    iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_MAX_PREAM_INTERVAL, IQUEUEMAC_MAX_PREAM_INTERVAL_US);
	}else{
		/* here, we set the pream_max_interval timeout to 5*MAX_PREAM_INTERVAL due to the fact that the first preamble is
		 * using csma for sending, and csma cost some time (could be large, i.e., larger than 4 ms). */
		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_MAX_PREAM_INTERVAL, (5*IQUEUEMAC_MAX_PREAM_INTERVAL_US));
	}

	gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval = false;

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE;
	gnrc_netdev->iqueuemac.need_update = true;

}

void iqueuemac_t2u_send_preamble(gnrc_netdev_t *gnrc_netdev)
{
	/* if memory full, release one pkt and reload next pkt */
	if(gnrc_netdev->iqueuemac.rx_memory_full == true){
		gnrc_netdev->iqueuemac.rx_memory_full = false;

		puts("memory full, drop one pkt");

		if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL){
			gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
			gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
			gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
		}

		gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.current_neighbor->queue);

		if(pkt != NULL){
			gnrc_netdev->iqueuemac.tx.tx_packet = pkt;
		}else{
			puts("iqueueMAC: NUll pkt, goto t2u end");
			gnrc_netdev->tx.current_neighbor = NULL;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
			gnrc_netdev->iqueuemac.need_update = true;
			return;
		}
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_MAX_PREAM_INTERVAL)){
		gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval = true;
	}

	/* if rx is going, wait until rx is completed. */
	if((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX) &&
	   (gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval == false)){
		/* when rx completed, will reach here */
		if(gnrc_netdev->iqueuemac.packet_received == true){
			gnrc_netdev->iqueuemac.packet_received = false;
			iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev);
		}

		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RX_END);
		if(gnrc_netdev->iqueuemac.quit_current_cycle == false){
		    iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RX_END, IQUEUEMAC_WAIT_RX_END_US);
		    return;
		}
	}

	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);

	if(gnrc_netdev->iqueuemac.packet_received == true){
		gnrc_netdev->iqueuemac.packet_received = false;
	   	iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev);
	}

	// to be filt in! for example, add receive other's broadcast and preamble handle codes here!!!
	if(gnrc_netdev->iqueuemac.quit_current_cycle == true){
		puts("q t2u");
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval == true) {
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	//if every thing goes fine, continue to send preamble.

	/***  disable auto-ack, namely disable pkt reception. ***/
	//iqueuemac_set_autoack(iqueuemac, NETOPT_DISABLE);

	int res;
	if(gnrc_netdev->iqueuemac.tx.preamble_sent == 0){
		res = iqueue_mac_send_preamble(gnrc_netdev, NETOPT_ENABLE);
		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_PREAMBLE_DURATION, IQUEUEMAC_PREAMBLE_DURATION_US);
	}else{
		res = iqueue_mac_send_preamble(gnrc_netdev, NETOPT_DISABLE);
	}

	if(res < 0){
		printf("preamble %d\n", res);
	}

	/* Enable Auto ACK again for data reception */
	//iqueuemac_set_autoack(iqueuemac, NETOPT_ENABLE);

	gnrc_netdev->iqueuemac.tx.preamble_sent ++;

	/* in case that memery is full, quit t-2-u and release pkt. */
	if(res == -ENOBUFS){
		puts("iq: nobuf for preamble, send preamble failed.");
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_WAIT_PREAMBLE_TX_END;
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_t2u_wait_preamble_tx_end(gnrc_netdev_t *gnrc_netdev)
{
	if(gnrc_netdev->iqueuemac.tx.tx_finished == true){
		/******set preamble interval timeout ******/
		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_PREAMBLE, IQUEUEMAC_PREAMBLE_INTERVAL_US);

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_WAIT_PREAMBLE_ACK;
		gnrc_netdev->iqueuemac.need_update = false;
		return;
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_MAX_PREAM_INTERVAL)){
		gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}
}

void iqueuemac_t2u_wait_preamble_ack(gnrc_netdev_t *gnrc_netdev)
{

	/* if memory full, release one pkt and reload next pkt */
	if(gnrc_netdev->iqueuemac.rx_memory_full == true){
		gnrc_netdev->iqueuemac.rx_memory_full = false;

		puts("memory full, drop one pkt");

		if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL){
			gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
			gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
			gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
		}

		gnrc_pktsnip_t *pkt = gnrc_priority_pktqueue_pop(&gnrc_netdev->tx.current_neighbor->queue);

		if(pkt != NULL){
			gnrc_netdev->iqueuemac.tx.tx_packet = pkt;
		}else{
			puts("iqueueMAC: NUll pkt, goto t2u end");
			gnrc_netdev->tx.current_neighbor = NULL;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
			gnrc_netdev->iqueuemac.need_update = true;
			return;
		}
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_MAX_PREAM_INTERVAL)){
		gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval = true;
	}

	/* if rx is going, wait until rx is completed. */
	if((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX)&&(gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval == false)){
		/* when rx completed, will reach here */
		if(gnrc_netdev->iqueuemac.packet_received == true){
			gnrc_netdev->iqueuemac.packet_received = false;
			iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev);
		}

		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
		if(gnrc_netdev->iqueuemac.quit_current_cycle == false){
		    iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RX_END, IQUEUEMAC_WAIT_RX_END_US);
		    return;
		}
	}

	if(gnrc_netdev->iqueuemac.packet_received == true){
		gnrc_netdev->iqueuemac.packet_received = false;
	   	iqueuemac_packet_process_in_wait_preamble_ack(gnrc_netdev);
	}

	// to be filt in! for example, add receive other's broadcast and preamble handle codes here!!!
	if(gnrc_netdev->iqueuemac.quit_current_cycle == true){
		puts("q t2u");
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(gnrc_netdev->iqueuemac.tx.got_preamble_ack == true){
		if(gnrc_netdev->iqueuemac.tx.t2u_on_public_1 == true){
			gnrc_netdev->tx.current_neighbor->pub_chanseq = gnrc_netdev->iqueuemac.pub_channel_1;
		}else{
			gnrc_netdev->tx.current_neighbor->pub_chanseq = gnrc_netdev->iqueuemac.pub_channel_2;
		}

		/* Enable Auto ACK again for data reception */
		iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);

		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_DATA;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_PREAMBLE_DURATION)){

		gnrc_netdev->iqueuemac.tx.t2u_retry_contuer ++;

		if(gnrc_netdev->iqueuemac.tx.t2u_retry_contuer >= IQUEUEMAC_T2U_RETYR_THRESHOLD) {
			puts("no preamble-ack, drop pkt.");
			gnrc_netdev->iqueuemac.tx.t2u_retry_contuer = 0;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);
		}else{
			gnrc_netdev->iqueuemac.tx.no_ack_contuer = IQUEUEMAC_REPHASELOCK_THRESHOLD;
		    netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
		    gnrc_netdev->iqueuemac.tx.tx_seq = device_state->seq - 1;

			puts("t2u no-preamack, rety");
			/* thus not to set current_neighbour to NULL in t2u-end */
			gnrc_netdev->iqueuemac.quit_current_cycle = true;
			gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		}

		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_PREAMBLE)){
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if(gnrc_netdev->iqueuemac.tx.reach_max_preamble_interval == true) {
		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_PREPARE;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}
}

void iqueuemac_t2u_send_data(gnrc_netdev_t *gnrc_netdev){

	/* if found this is a retrying data, reload its original seq. */
	if(gnrc_netdev->iqueuemac.tx.no_ack_contuer > 0){
		netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
		device_state->seq = gnrc_netdev->iqueuemac.tx.tx_seq;
	}

	/***  do not disable auto-ack here, we need auto-ack for data transmission and possible retransmission ***/
	int res;
	res = iqueuemac_send_data_packet(gnrc_netdev, NETOPT_ENABLE);
	if(res < 0){
		printf("t2u %d, drop pkt\n", res);

		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	//iqueuemac->tx.tx_packet = NULL;

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_WAIT_TX_FEEDBACK;
	gnrc_netdev->iqueuemac.need_update = true;
	/**** add a "wait-for-tx feedback" period to wait for the ending of the tx operation ***/

	// in case router want to wait beacon for vtdma, it swithc states to t-2-r and wait-for-beacon here!!
	// first test this idea with simple node type!!
}

void iqueuemac_t2u_wait_tx_feedback(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.tx.tx_finished == true){

	/*** add another condition here in the furture: the tx-feedback must be ACK-got,
	 * namely, completed, to ensure router gets the data correctly***/
	    if(gnrc_netdev_get_tx_feedback(gnrc_netdev) == TX_FEEDBACK_SUCCESS){

	    	gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
	    	gnrc_netdev->iqueuemac.tx.tx_packet = NULL;

	    	gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
	    	gnrc_netdev->iqueuemac.tx.t2u_retry_contuer = 0;

	    	/*** TX_FEEDBACK_SUCCESS means the router has success received the queue-length indicator ***/
	    	/*  add this part in the future to support vtdma in sending-to-unkown (to router type) */
	    	if((gnrc_priority_pktqueue_length(&gnrc_netdev->tx.current_neighbor->queue) > 0) &&
	    	   (gnrc_netdev->tx.current_neighbor->mac_type == ROUTER)){

	    		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;
	    		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
	    		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);
	    		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
	    		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_MAX_PREAM_INTERVAL);

	    		gnrc_netdev->iqueuemac.tx.vtdma_para.get_beacon = false;
	    		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_BEACON, IQUEUEMAC_WAIT_BEACON_TIME_US);
	    		// need to flush the rx-queue ??
	    		gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

	    		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2r_state = DEVICE_T2R_WAIT_BEACON;

	    		if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
	    			gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_ROUTER;
	    		}

	    	}else{
	    		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
	    	}
	    }else{
	    	gnrc_netdev->iqueuemac.tx.t2u_retry_contuer ++;

	    	if(gnrc_netdev->iqueuemac.tx.t2u_retry_contuer >= IQUEUEMAC_T2U_RETYR_THRESHOLD) {
	    	    printf("t2u data failed on ch %d. drop pkt.\n",gnrc_netdev->tx.current_neighbor->pub_chanseq);
	    	    gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
	    	    gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
	    	    gnrc_netdev->tx.current_neighbor = NULL;
	    	    gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
	    	    gnrc_netdev->iqueuemac.tx.t2u_retry_contuer = 0;
	    	    gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
	    	}else{
	    		gnrc_netdev->iqueuemac.tx.no_ack_contuer = IQUEUEMAC_REPHASELOCK_THRESHOLD;
	    		netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)gnrc_netdev->dev;
	    		gnrc_netdev->iqueuemac.tx.tx_seq = device_state->seq - 1;

	    		printf("t2u data failed on ch %d. rety.\n",gnrc_netdev->tx.current_neighbor->pub_chanseq);
	    		/* thus not to set current_neighbour to NULL in t2u-end */
	    		gnrc_netdev->iqueuemac.quit_current_cycle = true;
	    		gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_END;
	    	}
	    }

	    gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueuemac_t2u_end(gnrc_netdev_t *gnrc_netdev){

	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE);
	iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_PREAMBLE_DURATION);

	/* in case of quit_current_cycle is true, don't release tx-pkt and tx-neighbor, will try immediately next cycle.*/
	if(gnrc_netdev->iqueuemac.quit_current_cycle == false)
	{
		if(gnrc_netdev->iqueuemac.tx.tx_packet != NULL){
			gnrc_pktbuf_release(gnrc_netdev->iqueuemac.tx.tx_packet);
			gnrc_netdev->iqueuemac.tx.tx_packet = NULL;
			gnrc_netdev->iqueuemac.tx.no_ack_contuer = 0;
			puts("drop pkt");
		}
		gnrc_netdev->tx.current_neighbor = NULL;
	}

	gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state = DEVICE_T2U_SEND_PREAMBLE_INIT;

	if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
		/*********** judge and update the states before switch back to CP listening period   ***********/

		/* if phase has been changed, figure out the related phase of tx-neighbors. */
		if(gnrc_netdev->iqueuemac.phase_changed == true){
			iqueuemac_figure_tx_neighbor_phase(gnrc_netdev);
		}

		gnrc_netdev->iqueuemac.router_states.router_basic_state = R_LISTENNING;
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING;
		gnrc_netdev->iqueuemac.router_states.router_new_cycle = false;

		iqueuemac_trun_off_radio(gnrc_netdev);
		//puts("iqueuemac: router (device) is in t-2-u end.");
	}
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueuemac_t2u_update(gnrc_netdev_t *gnrc_netdev)
{
	switch(gnrc_netdev->iqueuemac.device_states.iqueuemac_device_t2u_state)
	{
	 case DEVICE_T2U_SEND_PREAMBLE_INIT: iqueuemac_t2u_send_preamble_init(gnrc_netdev);break;
	 case DEVICE_T2U_SEND_PREAMBLE_PREPARE: iqueuemac_t2u_send_preamble_prepare(gnrc_netdev);break;
     case DEVICE_T2U_SEND_PREAMBLE: iqueuemac_t2u_send_preamble(gnrc_netdev); break;
     case DEVICE_T2U_WAIT_PREAMBLE_TX_END: iqueuemac_t2u_wait_preamble_tx_end(gnrc_netdev); break;
	 case DEVICE_T2U_WAIT_PREAMBLE_ACK: iqueuemac_t2u_wait_preamble_ack(gnrc_netdev); break;
	 case DEVICE_T2U_SEND_DATA: iqueuemac_t2u_send_data(gnrc_netdev); break;
	 case DEVICE_T2U_WAIT_TX_FEEDBACK: iqueuemac_t2u_wait_tx_feedback(gnrc_netdev);break;
	 case DEVICE_T2U_END: iqueuemac_t2u_end(gnrc_netdev);break;
	 default: break;
	}
}

/******************new router state machines*****/
void iqueue_mac_router_listen_cp_init(gnrc_netdev_t *gnrc_netdev){

	/* reset last_seq_info. important! need to do every cycle.*/
	for(int i=0;i<IQUEUEMAC_RX_CHECK_DUPPKT_BUFFER_SIZE;i++){
		if(gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].node_addr.len != 0){
			gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].life_cycle ++;
			if(gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].life_cycle >= IQUEUEMAC_RX_CHECK_DUPPKT_UNIT_MAX_LIFE){
				gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].node_addr.len = 0;
				gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].node_addr.addr[0]=0;
				gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].node_addr.addr[1]=0;
				gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].seq=0;
				gnrc_netdev->iqueuemac.rx.check_dup_pkt.last_nodes[i].life_cycle = 0;
			}
		}
	}


#if 0
	iqueuemac->rx.check_dup_pkt.last_1.node_addr.addr[0] =0;
	iqueuemac->rx.check_dup_pkt.last_1.node_addr.addr[1] =0;
	iqueuemac->rx.check_dup_pkt.last_1.seq = 0;

	iqueuemac->rx.check_dup_pkt.last_2.node_addr.addr[0] =0;
	iqueuemac->rx.check_dup_pkt.last_2.node_addr.addr[1] =0;
	iqueuemac->rx.check_dup_pkt.last_2.seq = 0;
#endif

	gnrc_netdev->iqueuemac.router_states.router_new_cycle = false;
	/******set cp timeout ******/
	uint32_t listen_period;
	listen_period = random_uint32_range(0, IQUEUEMAC_CP_RANDOM_END_US) + IQUEUEMAC_CP_DURATION_US;

	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_CP_END, listen_period);
	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_CP_MAX, IQUEUEMAC_CP_DURATION_MAX_US);

	/* Enable Auto ACK again for data reception */
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);

	/* turn to public channel */
	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.cur_pub_channel);

	gnrc_netdev->iqueuemac.rx_started = false;
	gnrc_netdev->iqueuemac.packet_received = false;
	iqueuemac_trun_on_radio(gnrc_netdev);

	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_LISTEN;
	gnrc_netdev->iqueuemac.need_update = true;

	//puts("CP");

	gnrc_netdev->iqueuemac.cp_backoff_counter = 0;
	gnrc_netdev->iqueuemac.quit_current_cycle = false;
	gnrc_netdev->iqueuemac.get_other_preamble = false;
	gnrc_netdev->iqueuemac.send_beacon_fail = false;
	gnrc_netdev->iqueuemac.cp_end = false;
	gnrc_netdev->iqueuemac.got_preamble = false;

	gnrc_netdev->iqueuemac.phase_changed = false;

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

	/* backoff phase if needed */
	if(gnrc_netdev->iqueuemac.phase_backoff == true){
		gnrc_netdev->iqueuemac.phase_backoff = false;
		iqueuemac_phase_backoff(gnrc_netdev);
	}
}

void iqueue_mac_router_listen_cp_listen(gnrc_netdev_t *gnrc_netdev){

	/* in the future, we will add CP extension func. And we should remember to disable CP extension when
	 * iqueuemac->get_other_preamble is true occurs!!!
	 */

    if(gnrc_netdev->iqueuemac.packet_received == true){
    	gnrc_netdev->iqueuemac.packet_received = false;
    	iqueue_router_cp_receive_packet_process(gnrc_netdev);

        /*  here is the CP extension func.
         * Add a CP maximum limit in the future. */

    	/* if sent preamble-ACK, must wait for data. */
    	if(gnrc_netdev->iqueuemac.got_preamble == true){
    		gnrc_netdev->iqueuemac.got_preamble = false;
    		gnrc_netdev->iqueuemac.cp_end = false;
    		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_END);
    		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);
    	}else{
    		if((gnrc_netdev->iqueuemac.get_other_preamble == false) &&
    		   (gnrc_netdev->iqueuemac.cp_end == false) &&
    		   (gnrc_netdev->iqueuemac.quit_current_cycle == false)) {
        		gnrc_netdev->iqueuemac.got_preamble = false;
        		gnrc_netdev->iqueuemac.cp_end = false;
        		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_END);
        		iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_CP_END, IQUEUEMAC_CP_DURATION_US);
    		}
    	}
    }

	if((gnrc_netdev->iqueuemac.phase_backoff == true) && (gnrc_netdev->iqueuemac.phase_changed == false)){
		gnrc_netdev->iqueuemac.phase_backoff = false;
		iqueuemac_phase_backoff(gnrc_netdev);
	}

	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_CP_MAX)){
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_END);
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_MAX);
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_END;
		gnrc_netdev->iqueuemac.need_update = true;
		return;
	}

	if((iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_CP_END))){
		gnrc_netdev->iqueuemac.cp_end = true;
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_END);
	}

	if((gnrc_netdev->iqueuemac.cp_end == true)||(gnrc_netdev->iqueuemac.quit_current_cycle == true))
	{
		if((_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX)&&
			(gnrc_netdev->iqueuemac.cp_backoff_counter < IQUEUEMAC_MAX_CP_BACKOFF_COUNTER))
		{
			gnrc_netdev->iqueuemac.cp_backoff_counter ++;
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
			iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RX_END, IQUEUEMAC_WAIT_RX_END_US);
		}else{
			/** only timeout event and rx_complete event will reach here! **/
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_END);
			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_CP_MAX);
			gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_END;
			gnrc_netdev->iqueuemac.need_update = true;
		}
	}

    /**  ensure that don't break the reception
    if(iqueuemac->rx_started == false){

    	if((iqueuemac_timeout_is_expired(iqueuemac, TIMEOUT_CP_END))||(iqueuemac->quit_current_cycle == true)){
    		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_CP_END);
    		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_END;
    		iqueuemac->need_update = true;
    	}
    }**/

	/*
	if(iqueuemac->quit_current_cycle == true){
		iqueuemac_clear_timeout(iqueuemac,TIMEOUT_CP_END);
		iqueuemac->router_states.router_listen_state = R_LISTEN_CP_END;
		iqueuemac->need_update = true;
	}*/
}

void iqueue_mac_router_cp_end(gnrc_netdev_t *gnrc_netdev){

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);

	_dispatch(gnrc_netdev->rx.dispatch_buffer);

	if(gnrc_netdev->iqueuemac.quit_current_cycle == true){
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
	}else{
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SEND_BEACON;
	}
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueue_mac_router_send_beacon(gnrc_netdev_t *gnrc_netdev){
    /**** run the sub-channel selection algorithm to select the sub-channel sequence ****/
    // iqueuemac_select_sub_channel_num(iqueuemac);

	/* set device seq
	netdev_ieee802154_t *device_state = (netdev_ieee802154_t *)iqueuemac->netdev->dev;
	if(device_state->seq > 20){
		device_state->seq = 0;
	}
	printf("seq: %d\n",device_state->seq);
    */

	/***  disable auto-ack ***/
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_DISABLE);

	/****** assemble and send the beacon ******/
	int res;
	res = iqueuemac_assemble_and_send_beacon(gnrc_netdev);

	/* Enable Auto ACK again for data reception */
	//iqueuemac_set_autoack(iqueuemac, NETOPT_ENABLE);

	if(res < 0){
		printf("beacon %d\n", res);

		gnrc_netdev->iqueuemac.send_beacon_fail = true;
		gnrc_netdev->iqueuemac.need_update = true;
	}else{
		/* if the beacon has not been sent due to no slots. */
		if(gnrc_netdev->iqueuemac.rx.router_vtdma_mana.total_slots_num == 0) {
			gnrc_netdev->iqueuemac.send_beacon_fail = true;
			gnrc_netdev->iqueuemac.need_update = true;
		}else{
		    gnrc_netdev->iqueuemac.need_update = false;
		}
	}

	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_WAIT_BEACON_FEEDBACK;
	//puts("iqueuemac: router is now sending the beacon!!!");
}

void iqueuemac_router_wait_beacon_feedback(gnrc_netdev_t *gnrc_netdev){

	if((gnrc_netdev->iqueuemac.tx.tx_finished == true)||(gnrc_netdev->iqueuemac.send_beacon_fail == true)){

		/****** router switch to sleep period or vTDMA period ******/
		if((gnrc_netdev->iqueuemac.rx.router_vtdma_mana.total_slots_num > 0)&&(gnrc_netdev->iqueuemac.send_beacon_fail == false)){
			gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_VTDMA_INIT;
			gnrc_netdev->iqueuemac.need_update = true;
		}else{ /**** no vTDMA period ****/

			/* has packet to send */
			if(iqueue_mac_find_next_tx_neighbor(gnrc_netdev)){

				/* if it is for broadcast */
				if(gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]){
					if(gnrc_netdev->iqueuemac.get_other_preamble == false){
						gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
						gnrc_netdev->iqueuemac.router_states.router_trans_state = R_BROADCAST;
					}else{
				        gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
					}
					/* if phase has been changed, figure out the related phase of tx-neighbors. */
					if(gnrc_netdev->iqueuemac.phase_changed == true){
						iqueuemac_figure_tx_neighbor_phase(gnrc_netdev);
					}
				}else{
					switch(gnrc_netdev->tx.current_neighbor->mac_type){
					case UNKNOWN: {
						if(gnrc_netdev->iqueuemac.get_other_preamble == false){
							gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
							gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_UNKOWN;
						}else{
							gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
						}
					}break;
					case ROUTER: {
						gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
						gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_ROUTER;
				 	}break;
				 	case NODE: {
						gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_NODE;
					}break;
				 	default:{
						puts("iqueuemac: error! Unknow MAC type.");break;
					}
					}
				}
				gnrc_netdev->iqueuemac.need_update = true;
			}else{/**** no packet to send, go to sleep ****/
		        gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
		        gnrc_netdev->iqueuemac.need_update = true;
			}
		}
	}
}

void iqueue_mac_router_vtdma_init(gnrc_netdev_t *gnrc_netdev){

	/*** switch the radio to the subchannel ***/
	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.sub_channel_num);
	/* Enable Auto ACK again for data reception */
	iqueuemac_set_autoack(gnrc_netdev, NETOPT_ENABLE);

	/*** set the vTDMA period timeout!!! ***/
	uint32_t vtdma_duration;
	vtdma_duration = gnrc_netdev->iqueuemac.rx.router_vtdma_mana.total_slots_num * IQUEUEMAC_VTDMA_SLOT_SIZE_US;

	iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_VTDMA, vtdma_duration);

	gnrc_netdev->iqueuemac.vtdma_end = false;

	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_VTDMA;
	gnrc_netdev->iqueuemac.need_update = true;

}

void iqueue_mac_router_vtdma(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.packet_received == true){
	    gnrc_netdev->iqueuemac.packet_received = false;

	    /*** check whether this packet-process func. can be merged with cp-packet-process func. due to similar functionalities!!! ***/
	    //iqueue_router_cp_receive_packet_process(iqueuemac);
	    iqueuemac_router_vtdma_receive_packet_process(gnrc_netdev);
	}

 	if(iqueuemac_timeout_is_expired(&gnrc_netdev->iqueuemac, TIMEOUT_VTDMA)){
 		gnrc_netdev->iqueuemac.vtdma_end = true;
 	}

 	if(gnrc_netdev->iqueuemac.vtdma_end == true){

 	    if(_get_netdev_state(gnrc_netdev) == NETOPT_STATE_RX)
 		{
 			iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
 			iqueuemac_set_timeout(&gnrc_netdev->iqueuemac, TIMEOUT_WAIT_RX_END, IQUEUEMAC_WAIT_RX_END_US);
 			return;
 		}

 	 	//puts("iqueuemac: Router vTDMA ends!!");
		iqueuemac_clear_timeout(&gnrc_netdev->iqueuemac,TIMEOUT_WAIT_RX_END);
		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_VTDMA_END;
		gnrc_netdev->iqueuemac.need_update = true;
 	 }
}

void iqueue_mac_router_vtdma_end(gnrc_netdev_t *gnrc_netdev){

	gnrc_priority_pktqueue_flush(&gnrc_netdev->rx.queue);
	_dispatch(gnrc_netdev->rx.dispatch_buffer);

	/*** switch the radio to the public-channel!!! ***/
	iqueuemac_turn_radio_channel(gnrc_netdev, gnrc_netdev->iqueuemac.cur_pub_channel);

	/*** ensure that the channel-switching is finished before go to sleep to turn it off !!! ***/

	/*** see if there is pkt to send ***/
	if(iqueue_mac_find_next_tx_neighbor(gnrc_netdev)){

		if(gnrc_netdev->tx.current_neighbor == &gnrc_netdev->tx.neighbors[0]){
			if(gnrc_netdev->iqueuemac.get_other_preamble == false){
				gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
				gnrc_netdev->iqueuemac.router_states.router_trans_state = R_BROADCAST;
			}else{
		        gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
			}
			/* if phase has been changed, figure out the related phase of tx-neighbors. */
			if(gnrc_netdev->iqueuemac.phase_changed == true){
				iqueuemac_figure_tx_neighbor_phase(gnrc_netdev);
			}
		}else{
			switch(gnrc_netdev->tx.current_neighbor->mac_type){
			  case UNKNOWN: {
				  if(gnrc_netdev->iqueuemac.get_other_preamble == false){
					  gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
					  gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_UNKOWN;
				  }else{
					  gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
				  }
			  }break;
			  case ROUTER: {
				  gnrc_netdev->iqueuemac.router_states.router_basic_state = R_TRANSMITTING;
				  gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_ROUTER;
		 	 }break;
		 	 case NODE: {
				  gnrc_netdev->iqueuemac.router_states.router_trans_state = R_TRANS_TO_NODE;
			  }break;
			 default:{
				 puts("iqueuemac: error! Unknow MAC type.");break;
			 }
			}
		}
	}else{
        gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_INIT;
	}

	gnrc_netdev->iqueuemac.need_update = true;

}

void iqueue_mac_router_sleep_init(gnrc_netdev_t *gnrc_netdev){

	/* if phase has been changed, figure out the related phase of tx-neighbors. */
	if(gnrc_netdev->iqueuemac.phase_changed == true){
		iqueuemac_figure_tx_neighbor_phase(gnrc_netdev);
	}

	iqueuemac_trun_off_radio(gnrc_netdev);
	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING;
	gnrc_netdev->iqueuemac.need_update = true;

	//puts("iqueuemac: router is now entering sleeping period");
}

void iqueue_mac_router_sleep(gnrc_netdev_t *gnrc_netdev){

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
		//puts("iqueuemac: router sends in sleep");
	}else{
		if(iqueuemac->router_states.router_new_cycle == false){
		    /*  enable lpm mode, enter the sleep mode */
		    lpm_prevent_sleep &= ~(IQUEUEMAC_LPM_MASK);
		}
	}
#endif
	if(gnrc_netdev->iqueuemac.router_states.router_new_cycle == true){

		gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_SLEEPING_END;
		gnrc_netdev->iqueuemac.need_update = true;
	}
}

void iqueue_mac_router_sleep_end(gnrc_netdev_t *gnrc_netdev){

	gnrc_netdev->iqueuemac.router_states.router_listen_state = R_LISTEN_CP_INIT;
	gnrc_netdev->iqueuemac.need_update = true;
}

void iqueue_mac_router_listen_update(gnrc_netdev_t *gnrc_netdev){

	switch(gnrc_netdev->iqueuemac.router_states.router_listen_state)
   {
	case R_LISTEN_CP_INIT: iqueue_mac_router_listen_cp_init(gnrc_netdev); break;
	case R_LISTEN_CP_LISTEN: iqueue_mac_router_listen_cp_listen(gnrc_netdev); break;
	case R_LISTEN_CP_END: iqueue_mac_router_cp_end(gnrc_netdev); break;
	//case R_LISTEN_CREATE_BEACON: iqueue_mac_router_create_beacon(iqueuemac); break;
	case R_LISTEN_SEND_BEACON: iqueue_mac_router_send_beacon(gnrc_netdev); break;
	case R_LISTEN_WAIT_BEACON_FEEDBACK: iqueuemac_router_wait_beacon_feedback(gnrc_netdev);break;
	case R_LISTEN_VTDMA_INIT: iqueue_mac_router_vtdma_init(gnrc_netdev); break;
	case R_LISTEN_VTDMA: iqueue_mac_router_vtdma(gnrc_netdev); break;
	case R_LISTEN_VTDMA_END: iqueue_mac_router_vtdma_end(gnrc_netdev); break;
	case R_LISTEN_SLEEPING_INIT: iqueue_mac_router_sleep_init(gnrc_netdev); break;
	case R_LISTEN_SLEEPING: iqueue_mac_router_sleep(gnrc_netdev); break;
	case R_LISTEN_SLEEPING_END: iqueue_mac_router_sleep_end(gnrc_netdev); break;
	default: break;
   }
}
//////////////////////////////////////////////////
void iqueue_mac_router_transmit_update(gnrc_netdev_t *gnrc_netdev){

	switch(gnrc_netdev->iqueuemac.router_states.router_trans_state)
   {
	case R_TRANS_TO_UNKOWN: iqueuemac_t2u_update(gnrc_netdev);break; //iqueuemac_router_t2u_update(iqueuemac); break;
	case R_TRANS_TO_ROUTER: iqueuemac_t2r_update(gnrc_netdev);break;//iqueuemac_router_t2r_update(iqueuemac);
	case R_BROADCAST: iqueuemac_device_broadcast_update(gnrc_netdev);break;
	default: break;
   }

}

void iqueue_mac_router_update(gnrc_netdev_t *gnrc_netdev){

	switch(gnrc_netdev->iqueuemac.router_states.router_basic_state)
   {
	case R_INIT: iqueuemac_init_update(gnrc_netdev);break;
	case R_LISTENNING: iqueue_mac_router_listen_update(gnrc_netdev); break;
	case R_TRANSMITTING: iqueue_mac_router_transmit_update(gnrc_netdev); break;
	default: break;
   }
}

void iqueue_mac_update(gnrc_netdev_t *gnrc_netdev){

	if(gnrc_netdev->iqueuemac.mac_type == ROUTER){
	  iqueue_mac_router_update(gnrc_netdev);
	}
}

///static void _pass_on_packet(gnrc_pktsnip_t *pkt);

/**
 * @brief   Function called by the device driver on device events
 *
 * @param[in] event     type of event
 * @param[in] data      optional parameter
 */
static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) dev->context;

    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;

        msg.type = NETDEV_MSG_TYPE_EVENT;
        msg.content.ptr = (void*) gnrc_netdev;

        if (msg_send(&msg, gnrc_netdev->pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
    }
    else {
        DEBUG("gnrc_netdev: event triggered -> %i\n", event);
        switch(event) {

            case NETDEV_EVENT_RX_STARTED:
            	gnrc_netdev->iqueuemac.rx_started = true;
            	//puts("iqueuemac: rx-started event triggered.");
            	gnrc_netdev->iqueuemac.need_update = true;
            	break;

            case NETDEV_EVENT_RX_COMPLETE:
                {
                	gnrc_netdev->iqueuemac.need_update = true;

                    gnrc_pktsnip_t *pkt = gnrc_netdev->recv(gnrc_netdev);

                    if(pkt == NULL){
                    	gnrc_netdev->iqueuemac.rx_memory_full = true;
                    	puts("rx: pkt is NULL, memory full?");
                    	gnrc_netdev->iqueuemac.packet_received = false;
                    	gnrc_netdev->iqueuemac.rx_started = false;
                    	break;
                    }

                    if(!gnrc_netdev->iqueuemac.rx_started) {
       				   //LOG_WARNING("Maybe sending kicked in and frame buffer is now corrupted\n");
                    	puts("rx_pkt corrupted?");
       				    gnrc_pktbuf_release(pkt);
       				 gnrc_netdev->iqueuemac.rx_started = false;
                        break;
                    }

                    gnrc_netdev->iqueuemac.rx_started = false;

                    /* update the seq to avoid duplicate pkt.
                    gnrc_netif_hdr_t* netif_hdr;
                    netif_hdr = _gnrc_pktbuf_find(pkt, GNRC_NETTYPE_NETIF);
                    //printf("iqueuemac: the received packet rssi is: %d .\n", netif_hdr->rssi);
                    iqueuemac.rx.last_seq_info.seq = netif_hdr->seq;
                    */

                    if(!gnrc_mac_queue_rx_packet(&gnrc_netdev->rx, 0, pkt))
                   	{
                    	//LOG_ERROR("Can't push RX packet @ %p, memory full?\n", pkt);
                    	puts("can't push rx-pkt, memory full?");
                    	gnrc_pktbuf_release(pkt);
                    	gnrc_netdev->iqueuemac.packet_received = false;
                    	break;
                    }else{
                    	gnrc_netdev->iqueuemac.packet_received = true;
                    }

                    //iqueuemac.need_update = true;
                    /*
                    if (pkt) {
                        _pass_on_packet(pkt);
                    }*/
                }break;

            case NETDEV_EVENT_TX_COMPLETE:{
            	gnrc_netdev_set_tx_feedback(gnrc_netdev,TX_FEEDBACK_SUCCESS);
            	gnrc_netdev->iqueuemac.tx.tx_finished = true;
            	iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
            	gnrc_netdev->iqueuemac.need_update = true;
            }break;

           	case NETDEV_EVENT_TX_NOACK:{
           		gnrc_netdev_set_tx_feedback(gnrc_netdev,TX_FEEDBACK_NOACK);
           		gnrc_netdev->iqueuemac.tx.tx_finished = true;
           		iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
           		gnrc_netdev->iqueuemac.need_update = true;
           	}break;

           	case NETDEV_EVENT_TX_MEDIUM_BUSY:{
           		gnrc_netdev_set_tx_feedback(gnrc_netdev,TX_FEEDBACK_BUSY);
           		gnrc_netdev->iqueuemac.tx.tx_finished = true;
           		iqueuemac_set_raddio_to_listen_mode(gnrc_netdev);
           		gnrc_netdev->iqueuemac.need_update = true;
           	}break;
/*
        	case NETDEV_EVENT_TX_STARTED:{
        		if(iqueuemac.tx.got_preamble_ack == true){
        		  puts("iqueuemac: data packet transmission tx started!");
        		 }
        	}break;*/

#ifdef MODULE_NETSTATS_L2
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                dev->stats.tx_failed++;
                break;
            case NETDEV_EVENT_TX_COMPLETE:
                dev->stats.tx_success++;
                break;
#endif
            default: break;

                DEBUG("gnrc_netdev: warning: unhandled event %u.\n", event);
        }
    }
}

/**
 * @brief   Startup code and event loop of the gnrc_netdev layer
 *
 * @param[in] args  expects a pointer to the underlying netdev device
 *
 * @return          never returns
 */
static void *_gnrc_iqueuemac_thread(void *args)
{

    gnrc_netdev_t* gnrc_netdev = (gnrc_netdev_t *)args;
    netdev_t* dev = gnrc_netdev->dev;
        
    /**************************************origin*************************************/
    DEBUG("gnrc_netdev: starting thread\n");
    //gnrc_netdev_t *gnrc_netdev = (gnrc_netdev_t*) args;
    //netdev_t *dev = gnrc_netdev->dev;

    gnrc_netdev->pid = thread_getpid();

    gnrc_netapi_opt_t *opt;
    int res;
    msg_t msg, reply, msg_queue[NETDEV_NETAPI_MSG_QUEUE_SIZE];

    /* setup the MAC layers message queue */
    msg_init_queue(msg_queue, NETDEV_NETAPI_MSG_QUEUE_SIZE);
    /***************************************origin************************************/
    
    /*************************************iqueue-mac**************************************/
    /* RTT is used for scheduling wakeup */
    rtt_init();    
    
    /* Store pid globally, so that IRQ can use it to send msg */
    gnrc_netdev->iqueuemac.pid = thread_getpid();
    iqueuemac_pid = thread_getpid();
    /*************************************iqueue-mac**************************************/
    
    /***************************************************************************/
    /* register the event callback with the device driver */
    dev->event_callback = _event_cb;
    dev->context = (void*) gnrc_netdev;

    /* register the device to the network stack*/
    gnrc_netif_add(thread_getpid());

    /* initialize low-level driver */
    dev->driver->init(dev);
    /***************************************************************************/

    gnrc_netdev->iqueuemac.mac_type = MAC_TYPE;

    xtimer_sleep(3);

    iqueuemac_init(gnrc_netdev);

    uint32_t seed;
    seed = (uint32_t)gnrc_netdev->l2_addr[0];

    random_init(seed);

    gnrc_netdev->iqueuemac.need_update = true;

    /* start the event loop */
    while (1) {
        DEBUG("gnrc_netdev: waiting for incoming messages\n");
        msg_receive(&msg);
        /* dispatch NETDEV and NETAPI messages */
        switch (msg.type) {
            case NETDEV_MSG_TYPE_EVENT:
                DEBUG("gnrc_netdev: GNRC_NETDEV_MSG_TYPE_EVENT received\n");
                dev->driver->isr(dev);
                break;
            case GNRC_NETAPI_MSG_TYPE_SET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* set option for device driver */
                res = dev->driver->set(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->set: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
                /* read incoming options */
                opt = (gnrc_netapi_opt_t *)msg.content.ptr;
                DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_GET received. opt=%s\n",
                        netopt2str(opt->opt));
                /* get option from device driver */
                res = dev->driver->get(dev, opt->opt, opt->data, opt->data_len);
                DEBUG("gnrc_netdev: response of netdev->get: %i\n", res);
                /* send reply to calling thread */
                reply.type = GNRC_NETAPI_MSG_TYPE_ACK;
                reply.content.value = (uint32_t)res;
                msg_reply(&msg, &reply);
                break;

            /**************************************iqueue-mac********************************************/
            case IQUEUEMAC_EVENT_RTT_TYPE:{
                rtt_handler(msg.content.value, gnrc_netdev);
            }break;

            case IQUEUEMAC_EVENT_TIMEOUT_TYPE:{
             // printf("iqueuemac: Hitting a timeout event.\n");
              iqueuemac_timeout_make_expire((iqueuemac_timeout_t*) msg.content.ptr);
              gnrc_netdev->iqueuemac.need_update = true;
            }break;

            case GNRC_NETAPI_MSG_TYPE_SND:{
              DEBUG("gnrc_netdev: GNRC_NETAPI_MSG_TYPE_SND received\n");

              gnrc_pktsnip_t *pkt = (gnrc_pktsnip_t *)msg.content.ptr;

              if(!gnrc_mac_queue_tx_packet(&gnrc_netdev->tx, 0, pkt)) {
            	  LOG(LOG_WARNING, "WARNING: [GoMacH] TX queue full, drop packet\n");
              }
              gnrc_netdev->iqueuemac.need_update = true;

            }break;
            /**************************************iqueue-mac********************************************/

            default:
                DEBUG("gnrc_netdev: Unknown command %" PRIu16 "\n", msg.type);
                break;
        }

        while(gnrc_netdev->iqueuemac.need_update == true)  //&&(iqueuemac.duty_cycle_started == true)
        {
        	gnrc_netdev->iqueuemac.need_update = false;
            iqueue_mac_update(gnrc_netdev);
        }
    }
    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_iqueuemac_init(char *stack, int stacksize, char priority,
                        const char *name, gnrc_netdev_t *gnrc_netdev)
{
    kernel_pid_t res;

    /* check if given netdev device is defined and the driver is set */
    if (gnrc_netdev == NULL || gnrc_netdev->dev == NULL) {
        return -ENODEV;
    }

    /* create new gnrc_netdev thread */
    res = thread_create(stack, stacksize, priority, THREAD_CREATE_STACKTEST,
                         _gnrc_iqueuemac_thread, (void *)gnrc_netdev, name);
    if (res <= 0) {
        return -EINVAL;
    }

    return res;
}

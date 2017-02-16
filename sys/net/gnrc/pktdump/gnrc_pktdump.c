/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     net_pktdump
 * @{
 *
 * @file
 * @brief       Generic module to dump packages received via netapi to STDOUT
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */
#include <inttypes.h>
#include <stdio.h>

#include <errno.h>
#include "byteorder.h"
#include "thread.h"
#include "msg.h"
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#include "net/ipv6/addr.h"
#include "net/ipv6/hdr.h"
#include "net/udp.h"
#include "net/sixlowpan.h"
#include "od.h"
#include <periph/rtt.h>


uint32_t idlist[20];
uint32_t reception_list[20];

uint64_t delay_sum;
uint32_t system_start_time = 0;

/**
 * @brief   PID of the pktdump thread
 */
kernel_pid_t gnrc_pktdump_pid = KERNEL_PID_UNDEF;

/**
 * @brief   Stack for the pktdump thread
 */
static char _stack[GNRC_PKTDUMP_STACKSIZE];
#if 0
static void _dump_snip(gnrc_pktsnip_t *pkt)
{
    switch (pkt->type) {
        case GNRC_NETTYPE_UNDEF:
            printf("NETTYPE_UNDEF (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
#ifdef MODULE_GNRC_NETIF
        case GNRC_NETTYPE_NETIF:
            printf("NETTYPE_NETIF (%i)\n", pkt->type);
            gnrc_netif_hdr_print(pkt->data);
            break;
#endif
#ifdef MODULE_GNRC_SIXLOWPAN
        case GNRC_NETTYPE_SIXLOWPAN:
            printf("NETTYPE_SIXLOWPAN (%i)\n", pkt->type);
            sixlowpan_print(pkt->data, pkt->size);
            break;
#endif
#ifdef MODULE_GNRC_IPV6
        case GNRC_NETTYPE_IPV6:
            printf("NETTYPE_IPV6 (%i)\n", pkt->type);
            ipv6_hdr_print(pkt->data);
            break;
#endif
#ifdef MODULE_GNRC_ICMPV6
        case GNRC_NETTYPE_ICMPV6:
            printf("NETTYPE_ICMPV6 (%i)\n", pkt->type);
            break;
#endif
#ifdef MODULE_GNRC_TCP
        case GNRC_NETTYPE_TCP:
            printf("NETTYPE_TCP (%i)\n", pkt->type);
            break;
#endif
#ifdef MODULE_GNRC_UDP
        case GNRC_NETTYPE_UDP:
            printf("NETTYPE_UDP (%i)\n", pkt->type);
            udp_hdr_print(pkt->data);
            break;
#endif
#ifdef TEST_SUITES
        case GNRC_NETTYPE_TEST:
            printf("NETTYPE_TEST (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
#endif
        default:
            printf("NETTYPE_UNKNOWN (%i)\n", pkt->type);
            od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
            break;
    }
}
#endif

static void _dump(gnrc_pktsnip_t *pkt, uint32_t received_pkt_counter)
{
	/*
    int snips = 0;
    int size = 0;
    gnrc_pktsnip_t *snip = pkt;


    while (snip != NULL) {
        printf("~~ SNIP %2i - size: %3u byte, type: ", snips,
               (unsigned int)snip->size);
        _dump_snip(snip);
        ++snips;
        size += snip->size;
        snip = snip->next;
    }*/

    //printf("~~ PKT    - %2i snips, total size: %3i byte\n", snips, size);

	uint32_t *payload;

	uint32_t this_pkt_delay;

	uint32_t local_systime;

	this_pkt_delay = 0;
	local_systime = 0;

    //gnrc_netif_hdr_t *netif_hdr;

    //uint8_t* addr;
    //bool found_id;

    payload = pkt->data;

    received_pkt_counter -= 1;

    //found_id = false;

    if(payload[1] == 0x22222222) {

    	system_start_time = payload[0];
    	gnrc_pktbuf_release(pkt);
    	delay_sum = 0;

    	//printf("Dump: sys-start-time is %lu \n", system_start_time);
    	return;
    }


    if(payload[1] == 0xffffffff) {

    	puts("start exp results process");

    	received_pkt_counter -= 1;

    	//delay_sum = 0x123456789abcdef0;

    	uint32_t delausum_low, delausum_high;
    	delausum_low = delay_sum;
    	delausum_high = delay_sum >> 32;


    	printf("pkt delay sum hex (high): %lx \n", delausum_high);
    	printf("pkt delay sum hex (low): %lx \n", delausum_low);

    	delay_sum = (delay_sum / (uint64_t) received_pkt_counter);

    	printf("total received packet number: %lu \n", received_pkt_counter);

    	printf("average pkt delay: %lu \n", (uint32_t)delay_sum);

    	gnrc_pktbuf_release(pkt);
    	return;
    }

    local_systime = rtt_get_counter() - system_start_time;

    //printf("local_systime is %lu \n", RTT_TICKS_TO_US(local_systime));

    //printf("pkt genera time is %lu \n", RTT_TICKS_TO_US(payload[5] - payload[4]));

    this_pkt_delay = local_systime - (payload[5] - payload[4]);

    //printf("this_pkt_delay is %lu \n", RTT_TICKS_TO_US(this_pkt_delay));

    delay_sum += (uint64_t) RTT_TICKS_TO_US(this_pkt_delay);

#if 0
    int i=0;
    /* find id exist or not */
    for(i=0;i<20;i++){
    	if(idlist[i] == payload[1]){
    		found_id = true;
    		reception_list[i] ++;
    		break;
    	}
    }

    if(found_id == false){
    	for(i=0;i<20;i++){
    		if(idlist[i] == 0){
    			idlist[i] = payload[1];
    			reception_list[i] ++;
    			break;
    		}
    	}
    }
#endif

   // printf("s: %x, g: %lu, r: %lu, t: %lu. \n", addr[1], payload[0], reception_list[i], received_pkt_counter);

    //printf("%lx, %lu, %lu, %lu. \n", payload[1], payload[0], reception_list[i], received_pkt_counter);

    gnrc_pktbuf_release(pkt);
}

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;
    msg_t msg_queue[GNRC_PKTDUMP_MSG_QUEUE_SIZE];

    uint32_t received_pkt_counter;
    received_pkt_counter = 0;
    system_start_time = 0;

    delay_sum = 0;

    /* setup the message queue */
    msg_init_queue(msg_queue, GNRC_PKTDUMP_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    for(int i=0;i<20;i++){
    	idlist[i] =0;
    	reception_list[i] =0;
    }

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                //puts("PKTDUMP: over data received:");
            	received_pkt_counter ++;
                _dump(msg.content.ptr, received_pkt_counter);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                puts("PKTDUMP: data to send:");
                _dump(msg.content.ptr, received_pkt_counter);
                break;
            case GNRC_NETAPI_MSG_TYPE_GET:
            case GNRC_NETAPI_MSG_TYPE_SET:
                msg_reply(&msg, &reply);
                break;
            default:
                puts("PKTDUMP: received something unexpected");
                break;
        }
    }

    /* never reached */
    return NULL;
}

kernel_pid_t gnrc_pktdump_init(void)
{
    if (gnrc_pktdump_pid == KERNEL_PID_UNDEF) {
        gnrc_pktdump_pid = thread_create(_stack, sizeof(_stack), GNRC_PKTDUMP_PRIO,
                             THREAD_CREATE_STACKTEST,
                             _eventloop, NULL, "pktdump");
    }
    return gnrc_pktdump_pid;
}

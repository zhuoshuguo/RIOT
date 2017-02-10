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
#include "xtimer.h"

uint32_t own_addess;

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

static void _dump(gnrc_pktsnip_t *pkt)
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
    }

    printf("~~ PKT    - %2i snips, total size: %3i byte\n", snips, size);
    gnrc_pktbuf_release(pkt);
    */

	kernel_pid_t dev;
	uint8_t addr[2];
	size_t addr_len;
	gnrc_pktsnip_t *hdr;
	uint32_t *payload;

	int16_t dev2;

	dev2 = 4;
	/* parse interface */
	dev = (kernel_pid_t)dev2;

	payload = pkt->data;

	addr_len = 2;

	if(own_addess == 0x103e) {
	  	if(payload[3] == 0x0000447e){
	   		addr[0] = 0xa3;
	   		addr[1] = 0x12;
	   	}else if(payload[3] == 0x000052d2){
	   		addr[0] = 0x52;
	   		addr[1] = 0xd2;
	   	}
	}else if(own_addess == 0xa312) {
	         addr[0] = 0xe2;
	         addr[1] = 0x1a;
	}else if(own_addess == 0xe21a) {
	    if(payload[3] == 0x0000447e){
	    	addr[0] = 0x44;
	    	addr[1] = 0x7e;
	    }else if(payload[3] == 0x000052d2){
	    	addr[0] = 0x52;
	    	addr[1] = 0xd2;
	    }
	}else if(own_addess == 0x6f46) {
	    	if(payload[3] == 0x0000447e){
	    		addr[0] = 0x44;
	    		addr[1] = 0x7e;
	    	}else if(payload[3] == 0x000052d2){
	    		addr[0] = 0xe2;
	    		addr[1] = 0x1a;
	    	}
	}

	/** release old netif header **/
	gnrc_pktbuf_release(pkt->next);
	pkt->next= NULL;

	/** build new netif header **/
	hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
	if(hdr == NULL){
	   	puts("relay: buf full, drop pkt.");
	   	gnrc_pktbuf_release(pkt);
	   	return;
	}

	printf("%lx: %lu\n",payload[1], payload[0]);

	LL_PREPEND(pkt, hdr);

	int res;
	res = gnrc_netapi_send(dev, pkt);
	if(res < 1) {
	   	puts("relay: send data msg failed when push pkt.");
	}

}

static void *_eventloop(void *arg)
{
    (void)arg;
    msg_t msg, reply;
    msg_t msg_queue[GNRC_PKTDUMP_MSG_QUEUE_SIZE];

    /* setup the message queue */
    msg_init_queue(msg_queue, GNRC_PKTDUMP_MSG_QUEUE_SIZE);

    reply.content.value = (uint32_t)(-ENOTSUP);
    reply.type = GNRC_NETAPI_MSG_TYPE_ACK;

    int16_t devpid;
    devpid = 4;

    xtimer_sleep(2);

    uint8_t own_addr[2];
    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                    sizeof(own_addr));

    xtimer_sleep(3);

    own_addess = 0;
    own_addess = own_addr[0];
    own_addess = own_addess << 8;
    own_addess |= own_addr[1];

    while (1) {
        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV:
                //puts("PKTDUMP: data received:");
                _dump(msg.content.ptr);
                break;
            case GNRC_NETAPI_MSG_TYPE_SND:
                puts("PKTDUMP: data to send:");
                _dump(msg.content.ptr);
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

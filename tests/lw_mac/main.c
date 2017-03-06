/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a lot of functionality of RIOT
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Knüpfer <ludwig.knuepfer@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "timex.h"
#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/nettype.h"
#include "xtimer.h"
#include <periph/rtt.h>

/*
#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif
*/

#ifdef MODULE_LTC4150
#include "ltc4150.h"
#endif

#ifdef MODULE_NETIF
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#endif

uint32_t send_counter;
uint32_t send_counter1;
uint32_t send_counter2;
uint32_t own_address2;
uint32_t exp_start_time;
uint32_t exp_duration_ticks;

static void generate_and_send_pkt(void){

	    kernel_pid_t dev;
	    uint8_t addr[8];
	    size_t addr_len;
	    gnrc_pktsnip_t *hdr;
	    int16_t dev2;

	    gnrc_pktsnip_t* pkt;
	    uint32_t payload[20];

	    send_counter++;

	    payload[1] = own_address2;
	    payload[4] = exp_start_time;
	    payload[5] = rtt_get_counter();

	    dev2 = 4;
	    /* parse interface */
	    dev = (kernel_pid_t)dev2;

	    addr_len = 8;

	    if(own_address2 == 0x79f6) {  //

	    	//79:67:08:77:01:9f:33:1e

	    		payload[3] = 0x0000331e;

		        addr[0] = 0x79;
		        addr[1] = 0x67;

		        addr[2] = 0x26;
		        addr[3] = 0x7e;

		        addr[4] = 0x69;
		        addr[5] = 0x76;

		        addr[6] = 0x4c;
		        addr[7] = 0x66;

				/*
				//79:67:08:77:01:9f:33:1e
		        addr[0] = 0x79;
		        addr[1] = 0x67;

		        addr[2] = 0x08;
		        addr[3] = 0x77;

		        addr[4] = 0x01;
		        addr[5] = 0x9f;

		        addr[6] = 0x33;
		        addr[7] = 0x1e;
                */
		        payload[0] = send_counter;
		        //printf("%lx: %lu.\n", payload[3],send_counter);
	    }else if(own_address2 == 0xc13a) {  //e21a
			//79:67:08:77:01:9f:33:1e
	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x08;
	        addr[3] = 0x77;

	        addr[4] = 0x01;
	        addr[5] = 0x9f;

	        addr[6] = 0x33;
	        addr[7] = 0x1e;

		}

#if 0
	    if(own_address2 == 0xbcc6) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x52;
		        addr[1] = 0xd2;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x10;
		        addr[1] = 0x3e;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0x1b1a) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x10;
		        addr[1] = 0x3e;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x44;
		        addr[1] = 0x7e;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0x5ad6) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x6f;
		        addr[1] = 0x46;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x6f;
		        addr[1] = 0x46;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0x103e) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x52;
		        addr[1] = 0xd2;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0xa3;
		        addr[1] = 0x12;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0xa312) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x10;
		        addr[1] = 0x3e;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0xe2;
		        addr[1] = 0x1a;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0xe21a) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x52;
		        addr[1] = 0xd2;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x44;
		        addr[1] = 0x7e;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }else if(own_address2 == 0x6f46) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x52;
		        addr[1] = 0xd2;
		        send_counter1 ++;
		        payload[0] = send_counter1;
		        printf("%lx: %lu.\n", payload[3],send_counter1);
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x44;
		        addr[1] = 0x7e;
		        send_counter2 ++;
		        payload[0] = send_counter2;
		        printf("%lx: %lu.\n", payload[3],send_counter2);
	    	}
	    }
#endif


	    hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
	    if(hdr == NULL){
	    	puts("app: netif buf null!");
	    	send_counter --;
	    	return;
	    }

		/****** assemble and send the beacon ******/
		pkt = gnrc_pktbuf_add(NULL, payload, sizeof(payload), GNRC_NETTYPE_UNDEF);
		if(pkt == NULL) {

			gnrc_pktbuf_release(hdr);

			send_counter --;
			puts("app: data buf null!");
		}else{
		    LL_PREPEND(pkt, hdr);

		    int res;
		    res = gnrc_netapi_send(dev, pkt);

		    if(res < 1){
		    	puts("app: send data msg to mac failed!");
		    }

		    //printf("p: %lu.\n", send_counter);
		}

}


void *sender_thread(void *arg)
{
    (void) arg;
    msg_t msg;
    msg_t msg_queue[8];
    bool exp_end;

    uint32_t data_rate;
    uint32_t total_gene_num;
    data_rate = 0;
    total_gene_num = 0;

    /* setup the message queue */
    msg_init_queue(msg_queue, 8);

    send_counter = 0;
    send_counter1 = 0;
    send_counter2 = 0;

    int16_t devpid;

    devpid = 4;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));


    xtimer_sleep(1);

    own_address2 = 0;
    own_address2 = own_addr[0];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[1];

    printf("own add is %lx.\n", own_address2);

    gnrc_netreg_entry_t  me_reg;
    me_reg.demux_ctx = GNRC_NETREG_DEMUX_CTX_ALL;
    me_reg.target.pid = thread_getpid();

    //= { .demux_ctx = GNRC_NETREG_DEMUX_CTX_ALL, .pid = thread_getpid() };
    gnrc_netreg_register(GNRC_NETTYPE_APP, &me_reg);

   while (1) {

        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV: {
            	gnrc_pktsnip_t *pkt;
            	pkt = msg.content.ptr;
            	uint32_t *payload;

            	payload = pkt->data;
            	data_rate = payload[0];
            	total_gene_num = payload[2];

            	if(own_address2 == 0x79f6) {
            	    total_gene_num = payload[2];
            	}else {
                    total_gene_num = 0;
            	}

            	exp_start_time = payload[5];
            	exp_duration_ticks = payload[1];
            	exp_duration_ticks = exp_duration_ticks * 1000000;
            	exp_duration_ticks = RTT_US_TO_TICKS(exp_duration_ticks);

            	//printf("the exp-data_rate is %lu. \n", data_rate);
            	//printf("the exp-total_gene_num is %lu. \n", (long unsigned int) total_gene_num);

            	gnrc_pktbuf_release(pkt);

            } break;

            default:
                puts("PKTDUMP: received something unexpected");
                break;
        }

        break;
    }

   xtimer_usleep(500000);

   exp_end = false;

   while (1) {
   	//xtimer_sleep(1);
   	xtimer_usleep((uint32_t) data_rate * 1000);


   	if((send_counter < total_gene_num) && (rtt_get_counter() < (exp_duration_ticks + exp_start_time))){
   		for(int i=0; i<1; i++){
   			generate_and_send_pkt();
   		}
   	}else {
   	   	if(exp_end == false) {
   	   		printf("sender totally generated pkt num %lu .\n", send_counter);
   	   		exp_end =  false;
   	   	}
   	    exp_end = true;
   	}

   }

    return NULL;
}

char second_thread_stack[THREAD_STACKSIZE_MAIN];


int main(void)
{
#ifdef MODULE_LTC4150
    ltc4150_start();
#endif


#ifdef MODULE_NETIF
    gnrc_netreg_entry_t dump;

    dump.target.pid = gnrc_pktdump_pid;
    dump.demux_ctx = GNRC_NETREG_DEMUX_CTX_ALL;
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);
#endif

//kernel_pid_t pid =
    thread_create(second_thread_stack, sizeof(second_thread_stack),
                            THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST,
                            sender_thread, NULL, "shuguo_app");


    (void) puts("Welcome to RIOT!");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}

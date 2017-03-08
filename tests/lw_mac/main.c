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
#include <random.h>
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

	    if(own_address2 == 0x79f6) {  //51d6

	    	//79:67:08:77:01:9f:33:1e

	    		payload[3] = 0x0000331e;

	            addr[0] = 0x79;
	            addr[1] = 0x76;

	            addr[2] = 0x43;
	            addr[3] = 0x7f;

	            addr[4] = 0x72;
	            addr[5] = 0x2a;

	            addr[6] = 0x67;
	            addr[7] = 0x5e;

		        payload[0] = send_counter;
		        //printf("%lx: %lu.\n", payload[3],send_counter);
	    }else if(own_address2 == 0x4c66) {  //6f46
			//79:67:08:77:01:9f:33:1e
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x76;

            addr[2] = 0x43;
            addr[3] = 0x7f;

            addr[4] = 0x72;
            addr[5] = 0x2a;

            addr[6] = 0x67;
            addr[7] = 0x5e;

		}else if(own_address2 == 0x383a) {  //1b1a
			//79:67:08:77:01:9f:33:1e
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x76;

            addr[2] = 0x43;
            addr[3] = 0x7f;

            addr[4] = 0x72;
            addr[5] = 0x2a;

            addr[6] = 0x67;
            addr[7] = 0x5e;
            //////////////////////////////////////////////////////////

		}else if(own_address2 == 0x8032) {  //a312
			//79:67:3c:7c:2b:2a:c1:3a
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x67;

            addr[2] = 0x3c;
            addr[3] = 0x7c;

            addr[4] = 0x2b;
            addr[5] = 0x2a;

            addr[6] = 0xc1;
            addr[7] = 0x3a;

		}else if(own_address2 == 0x71f2) {  //52d2
			//79:67:3c:7c:2b:2a:c1:3a
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x67;

            addr[2] = 0x3c;
            addr[3] = 0x7c;

            addr[4] = 0x2b;
            addr[5] = 0x2a;

            addr[6] = 0xc1;
            addr[7] = 0x3a;
            //////////////////////////////////////////////////////////

		}else if(own_address2 == 0x2c02) {  //0f22
			//79:67:27:72:f4:57:9f:e6
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x67;

            addr[2] = 0x27;
            addr[3] = 0x72;

            addr[4] = 0xf4;
            addr[5] = 0x57;

            addr[6] = 0x9f;
            addr[7] = 0xe6;

		}else if(own_address2 == 0x4262) {  //6142
			//79:67:27:72:f4:57:9f:e6
	    	payload[3] = 0x0000331e;

            addr[0] = 0x79;
            addr[1] = 0x67;

            addr[2] = 0x27;
            addr[3] = 0x72;

            addr[4] = 0xf4;
            addr[5] = 0x57;

            addr[6] = 0x9f;
            addr[7] = 0xe6;

		}


	    hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);
	    if(hdr == NULL){
	    	puts("app: netif buf null!");
	    	return;
	    }

		/****** assemble and send the beacon ******/
		pkt = gnrc_pktbuf_add(NULL, payload, sizeof(payload), GNRC_NETTYPE_UNDEF);
		if(pkt == NULL) {

			gnrc_pktbuf_release(hdr);

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

            	} else if(own_address2 == 0x4c66) {
            		total_gene_num = payload[2];

            	}else if(own_address2 == 0x383a) {
            		total_gene_num = payload[2];
                     /////////////////////////////////////////////////////////////////////
            	}else if(own_address2 == 0x8032) {  //a312
            		total_gene_num = payload[2];

            	}else if(own_address2 == 0x71f2) {//52d2
            		total_gene_num = payload[2];

            		/////////////////////////////////////////////////////////////////////
            	}else if(own_address2 == 0x2c02) {   //0f22
            		total_gene_num = payload[2];

            	}else if(own_address2 == 0x4262) {   //0f22
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

 	uint32_t listen_period;
   	listen_period = random_uint32_range(0, 5000000);
   	xtimer_usleep(listen_period);

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

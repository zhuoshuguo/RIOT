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
#include "random.h"
#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "xtimer.h"
#include <periph/rtt.h>
/*
#if FEATURE_PERIPH_RTC
#include "periph/rtc.h"
#endif
*/

#ifdef MODULE_NETIF
#include "net/gnrc/pktdump.h"
#include "net/gnrc.h"
#endif


uint32_t own_address2;
uint32_t send_counter;

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
//	    payload[4] = exp_start_time;
//	    payload[5] = rtt_get_counter();

	    dev2 = 4;
	    /* parse interface */
	    dev = (kernel_pid_t)dev2;

//	    addr_len = 2;
//
//
//	    // set destination (sink) address
//		payload[3] = 0x0000331e;
//        addr[0] = 0x1e;
//        addr[1] = 0x33;


	    addr_len = 8;

	    // set destination (sink) address  79:67:35:7e:54:3a:79:f6
		payload[3] = 0x0000331e;
        addr[0] = 0x79;
        addr[1] = 0x67;

        addr[2] = 0x35;
        addr[3] = 0x7e;

        addr[4] = 0x54;
        addr[5] = 0x3a;

        addr[6] = 0x79;
        addr[7] = 0xf6;


        payload[0] = send_counter;
        //printf("%lx: %lu.\n", payload[3],send_counter);

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

//    uint32_t data_rate;
//    uint32_t total_gene_num;
//    uint32_t data_interval;
//    data_rate = 0;
//    total_gene_num = 0;


    int16_t devpid;

    devpid = 4;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));

    xtimer_sleep(3);

    own_address2 = 0;
    own_address2 = own_addr[0];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[1];

    printf("own add is %lx.\n", own_address2);

//  	uint32_t random_period;

//	random_period = random_uint32_range(0, 500000);
//
//	xtimer_usleep(random_period);


//   data_interval = (uint32_t) data_rate * 1000;

   	if (own_address2 == 0x4262) {
   	   	printf("Shuguo: cool! I can send packet \n");
   	} else {
   		printf("Shuguo: sad! I can't send packet \n");
   	}

   while (1) {
   	xtimer_sleep(1);
   	if (own_address2 == 0x4262) {
   	   	//printf("Shuguo: cool! And send packet \n");
   	   	generate_and_send_pkt();
   	}


//	uint32_t random_wait_period;
//	random_wait_period = random_uint32_range(data_interval - 500000, data_interval + 500000);
//   	xtimer_usleep(random_wait_period);

//   	if(send_counter < total_gene_num){
//   		for(int i=0; i<1; i++){
//   			generate_and_send_pkt();
//   		}
//   	}else {
//   	   	if(exp_end == false) {
//   	   		printf("sender totally generated pkt num %lu .\n", send_counter);
//   	   		exp_end =  false;
//   	   	}
//   	    exp_end = true;
//   	}

   }

    return NULL;
}


char second_thread_stack[THREAD_STACKSIZE_MAIN];


int main(void)
{
//#ifdef FEATURE_PERIPH_RTC
//    rtc_init();
//#endif

#ifdef MODULE_NETIF
    gnrc_netreg_entry_t dump = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                          gnrc_pktdump_pid);
    gnrc_netreg_register(GNRC_NETTYPE_UNDEF, &dump);
#endif

    (void) puts("Welcome to RIOT!");

    //kernel_pid_t pid =
    thread_create(second_thread_stack, sizeof(second_thread_stack),
                            THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST,
                            sender_thread, NULL, "shuguo_app");


    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}

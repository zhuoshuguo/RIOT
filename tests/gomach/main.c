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

        payload[0] = send_counter;

	    if(own_address2 == 0x79f6) {  //  5ad6

	    	//79:67:08:77:01:9f:33:1e

	    		payload[3] = 0x0000331e;

		        addr[0] = 0x79;
		        addr[1] = 0x67;

		        addr[2] = 0x08;
		        addr[3] = 0x77;

		        addr[4] = 0x01;
		        addr[5] = 0x9f;

		        addr[6] = 0x33;
		        addr[7] = 0x1e;

		        //printf("%lx: %lu.\n", payload[3],send_counter);
	    } else if(own_address2 == 0x383a) {  //1b1a

	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x35;
	        addr[3] = 0x7e;

	        addr[4] = 0x54;
	        addr[5] = 0x3a;

	        addr[6] = 0x79;
	        addr[7] = 0xf6;

		} else if(own_address2 == 0x8032) {  //a312
			/*/79:67:19:62:f5:60:38:3a
	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x19;
	        addr[3] = 0x62;

	        addr[4] = 0xf5;
	        addr[5] = 0x60;

	        addr[6] = 0x38;
	        addr[7] = 0x3a;
	        */

	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x35;
	        addr[3] = 0x7e;

	        addr[4] = 0x54;
	        addr[5] = 0x3a;

	        addr[6] = 0x79;
	        addr[7] = 0xf6;

		} else if(own_address2 == 0x2c02) {  //0f22

	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x35;
	        addr[3] = 0x7e;

	        addr[4] = 0x54;
	        addr[5] = 0x3a;

	        addr[6] = 0x79;
	        addr[7] = 0xf6;

		} else if(own_address2 == 0x4c66) {  //6f46

			/*
	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x08;
	        addr[3] = 0x77;

	        addr[4] = 0x01;
	        addr[5] = 0x9f;

	        addr[6] = 0x33;
	        addr[7] = 0x1e;
	        */

	        addr[0] = 0x79;
	        addr[1] = 0x67;

	        addr[2] = 0x35;
	        addr[3] = 0x7e;

	        addr[4] = 0x54;
	        addr[5] = 0x3a;

	        addr[6] = 0x79;
	        addr[7] = 0xf6;

		}

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

		printf("%lx: %lu.\n", own_address2, send_counter);

}


void *sender_thread(void *arg)
{
    (void) arg;

    uint32_t data_rate;
    data_rate = 0;


    send_counter = 0;
    send_counter1 = 0;
    send_counter2 = 0;

    int16_t devpid;

    devpid = 4;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));

    xtimer_sleep(2);

    own_address2 = 0;
    own_address2 = own_addr[0];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[1];

    printf("own add is %lx.\n", own_address2);

   xtimer_sleep(10);

   data_rate = 200;

   while (1) {
   	//xtimer_sleep(1);
   	xtimer_usleep((uint32_t) data_rate * 1000);

   		for(int i=0; i<1; i++){
   			//if (own_address2 == 0x79f6) {
   			    generate_and_send_pkt();
   			//}
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

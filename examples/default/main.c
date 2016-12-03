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
#include "xtimer.h"

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
uint32_t own_addess;

static void generate_and_send_pkt(void){

	    kernel_pid_t dev;
	    uint8_t addr[2];
	    size_t addr_len;
	    gnrc_pktsnip_t *hdr;
	    int16_t dev2;

	    gnrc_pktsnip_t* pkt;
	    uint32_t payload[12];

	    send_counter++;

	    payload[1] = own_addess;

	    dev2 = 4;
	    /* parse interface */
	    dev = (kernel_pid_t)dev2;

	    addr_len = 2;

	    if(own_addess == 0xbcc6) {
	    	payload[3] = 0x000052d2;
	        addr[0] = 0x52;
	        addr[1] = 0xd2;
	        payload[0] = send_counter;
	    	/*
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x52;
		        addr[1] = 0xd2;
		        send_counter1 ++;
		        payload[0] = send_counter1;
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x10;
		        addr[1] = 0x3e;
		        send_counter2 ++;
		        payload[0] = send_counter2;
	    	}*/
	    }else if(own_addess == 0x1b1a) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x10;
		        addr[1] = 0x3e;
		        send_counter1 ++;
		        payload[0] = send_counter1;
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0xa3;
		        addr[1] = 0x12;
		        send_counter2 ++;
		        payload[0] = send_counter2;
	    	}
	    }else if(own_addess == 0x5ad6) {
	    	if((send_counter%2)==0){
	    		payload[3] = 0x000052d2;
		        addr[0] = 0x6f;
		        addr[1] = 0x46;
		        send_counter1 ++;
		        payload[0] = send_counter1;
	    	}else{
	    		payload[3] = 0x0000447e;
		        addr[0] = 0x6f;
		        addr[1] = 0x46;
		        send_counter2 ++;
		        payload[0] = send_counter2;
	    	}
	    }else if(own_addess == 0x52d2) {
		    addr[0] = 0x44;
		    addr[1] = 0x7e;
	    }

	    //addr[0] = 0x10;
	    //addr[1] = 0x3e;

	    //addr[0] = 0x6f;
	    //addr[1] = 0x46;

	    //addr[0] = 0xa3;
	    //addr[1] = 0x12;

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

		    printf("p: %lu.\n", send_counter);
		}

}


void *sender_thread(void *arg)
{
    (void) arg;

    //printf("shuguo-app thread started, pid: %" PRIkernel_pid "\n", thread_getpid());

    send_counter = 0;
    send_counter1 = 0;
    send_counter2 = 0;

    int16_t devpid;

    devpid = 4;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));

    xtimer_sleep(3);

    own_addess = 0;
    own_addess = own_addr[0];
    own_addess = own_addess << 8;
    own_addess |= own_addr[1];

    printf("own add is %lx.\n", own_addess);

    xtimer_sleep(7);

    while (1) {

    	xtimer_sleep(1);
    	//xtimer_usleep(250000);

    	if(send_counter <10000){
    		for(int i=0; i<1; i++){
    			generate_and_send_pkt();
    		}
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

    dump.pid = gnrc_pktdump_pid;
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

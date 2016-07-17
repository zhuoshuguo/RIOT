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

static void generate_and_send_pkt(uint32_t send_counter){

	   kernel_pid_t dev;
	    uint8_t addr[8];
	    size_t addr_len;
	    gnrc_pktsnip_t *hdr;
	    int16_t dev2;

	    gnrc_pktsnip_t* pkt;
	    uint32_t payload[3];

	    payload[0] = send_counter;

	    dev2 = 4;
	    /* parse interface */  //5a:44:1e:68:a0:03:61:42
	    dev = (kernel_pid_t)dev2;

	    addr_len = 8;
	    addr[0] = 0x5a;
	    addr[1] = 0x44;

	    addr[2] = 0x1e;
	    addr[3] = 0x68;

	    addr[4] = 0xa0;
	    addr[5] = 0x03;

	    addr[6] = 0x61;
	    addr[7] = 0x42;

	    hdr = gnrc_netif_hdr_build(NULL, 0, addr, addr_len);

		/****** assemble and send the beacon ******/
		pkt = gnrc_pktbuf_add(NULL, payload, sizeof(payload), GNRC_NETTYPE_UNDEF);
		if(pkt == NULL) {
			puts("app: buf null!")    ;
		}

	    LL_PREPEND(pkt, hdr);

	    gnrc_netapi_send(dev, pkt);

	    printf("p: %lu.\n", send_counter);
}


void *sender_thread(void *arg)
{
    (void) arg;

    //printf("shuguo-app thread started, pid: %" PRIkernel_pid "\n", thread_getpid());

    uint32_t send_counter;
    send_counter =0;

    xtimer_sleep(5);

    while (1) {

    	xtimer_sleep(1);
    	if(send_counter <100){

    		for(int i=0; i<1; i++){
    			send_counter++;
    			generate_and_send_pkt(send_counter);
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

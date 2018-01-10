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
#include "net/gnrc/netdev.h"

typedef struct gnrc_netdev gnrc_netdev_t;

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

extern gnrc_netdev_t gnrc_netdev;

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

        payload[0] = send_counter;
	    payload[1] = own_address2;

	   	// report tdma slots number.
	   	//payload[2] = gnrc_netdev.gomach.csma_count;
	   	//payload[3] = gnrc_netdev.gomach.vtdma_count;

	   //	uint64_t *payload_long = (uint64_t *)payload;

	   //	payload_long[2] = gnrc_netdev.gomach.awake_duration_sum_ticks;
	   //	payload_long[3] = xtimer_now_usec64() - gnrc_netdev.gomach.system_start_time_ticks;

	    dev2 = 4;
	    /* parse interface */
	    dev = (kernel_pid_t)dev2;

	    addr_len = 8;

	    // 15:11:6b:10:65:fa:8a:22   m3-53
	    /*
        addr[0] = 0x15;
        addr[1] = 0x11;

        addr[2] = 0x6b;
        addr[3] = 0x10;

        addr[4] = 0x65;
        addr[5] = 0xfa;

        addr[6] = 0x8a;
        addr[7] = 0x22;
        */

	    //15:11:6b:10:65:f6:8b:26  m3-69
	    /*
        addr[0] = 0x15;
        addr[1] = 0x11;

        addr[2] = 0x6b;
        addr[3] = 0x10;

        addr[4] = 0x65;
        addr[5] = 0xf6;

        addr[6] = 0x8b;
        addr[7] = 0x26;
        */

	    //79:67:27:72:f4:57:9f:e6   bcc6
	    /*
        addr[0] = 0x79;
        addr[1] = 0x67;

        addr[2] = 0x27;
        addr[3] = 0x72;

        addr[4] = 0xf4;
        addr[5] = 0x57;

        addr[6] = 0x9f;
        addr[7] = 0xe6;
        */


        switch (own_address2) {
            case 0x65fbbe26:  //m3 - 11
            case 0x65f9be36:
            case 0x65f3af06:
            case 0x65fc8b22:
            case 0x65f78a36:
            case 0x65f8a83a:
            case 0x65f4be52:
            case 0x65f9a802:
            case 0x65fd8a3a:{
                puts("sleep-1200");
            	xtimer_sleep(1200);
            	break;
            }

            case 0x65f95c02:
            case 0x65f7be36:
            case 0x65fb8b26:
            case 0x65fba836:
            case 0x65f8a822:
            case 0x65fabe52:
            case 0x65f9af06:
            case 0x65fa8a2a:
            case 0x65f7a80a:{
                puts("sleep-900");
            	xtimer_sleep(900);
            	break;
            }
            //case 0x65f8a93a:
            case 0x65f7a922:
            case 0x65faa92a:
            case 0x65f6a802:
            case 0x65f7a926:
            case 0x65fbbe3a:
            case 0x65fb8b22:
            case 0x65f8be36:
            case 0x65fb8b2a:
            case 0x65fca92a:{
            //15:11:6b:10:65:f8:a9:3a
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xf8;

                addr[6] = 0xa9;
                addr[7] = 0x3a;
            	break;
            }
            //case 0x65fb8b36:   //m3 46
            case 0x65faa832:
            case 0x65f7bf52:
            case 0x65f8a926:
            case 0x65f9a836:
            case 0x65f48a26:
            case 0x65fbaf12:
            case 0x65fa8a22:
            case 0x65fd8b3a:
            case 0x65f68b22:{  //m3 57
                //15:11:6b:10:65:fb:8b:36
                    addr[0] = 0x15;
                    addr[1] = 0x11;

                    addr[2] = 0x6b;
                    addr[3] = 0x10;

                    addr[4] = 0x65;
                    addr[5] = 0xfb;

                    addr[6] = 0x8b;
                    addr[7] = 0x36;
            	break;
            }
            //case 0x65fb8b32:
            case 0x65f6a83a:
            case 0x65fca806:
            case 0x65f98b36:
            case 0x65f8a806:
            case 0x65f8a826:
            case 0x65f75c16:
            case 0x65fca82a:{
                //15:11:6b:10:65:fb:8b:32
                    addr[0] = 0x15;
                    addr[1] = 0x11;

                    addr[2] = 0x6b;
                    addr[3] = 0x10;

                    addr[4] = 0x65;
                    addr[5] = 0xfb;

                    addr[6] = 0x8b;
                    addr[7] = 0x32;
            	break;
            }
            default:{
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xfa;

                addr[6] = 0x8a;
                addr[7] = 0x22;
                break;
            }
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

		    printf("p: %lx: %lu.\n", own_address2, send_counter);
		}

}


void *sender_thread(void *arg)
{
    (void) arg;

    uint32_t data_rate;

    send_counter = 0;
    send_counter1 = 0;
    send_counter2 = 0;

    int16_t devpid;

    devpid = 4;


    uint8_t own_addr[8];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS_LONG, 0, &own_addr,
                            sizeof(own_addr));


    own_address2 = 0;
    own_address2 = own_addr[4];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[5];

    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[6];

    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[7];

    xtimer_sleep(10);
    printf("own add is %lx.\n", own_address2);

	data_rate = 1;

 	uint32_t listen_period;
   	listen_period = random_uint32_range(10, 120);
 	printf("random wait: %lu s.\n", listen_period);

   	xtimer_sleep(listen_period);

   	data_rate = 1;
   	//puts("start pushing data!");

    while (1) {
	    for(int i=0; i<1; i++){   //65:f6:8b:26
	    //if ((own_address2 != 0x65f68b26) && (send_counter < 10))
	    if ((own_address2 != 0x65fb8b36) && (send_counter < 10000))
		    generate_and_send_pkt();
	    }

	    //data_rate = random_uint32_range(15, 25);
        xtimer_sleep(data_rate);
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

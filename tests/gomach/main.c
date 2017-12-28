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
	   	payload[2] = gnrc_netdev.gomach.csma_count;
	   	payload[3] = gnrc_netdev.gomach.vtdma_count;

	   	uint64_t *payload_long = (uint64_t *)payload;

	   	payload_long[2] = gnrc_netdev.gomach.awake_duration_sum_ticks;
	   	payload_long[3] = xtimer_now_usec64() - gnrc_netdev.gomach.system_start_time_ticks;

	    dev2 = 4;
	    /* parse interface */
	    dev = (kernel_pid_t)dev2;

	    addr_len = 8;

	    //15:11:6b:10:65:f6:8b:26  m3-69
        addr[0] = 0x15;
        addr[1] = 0x11;

        addr[2] = 0x6b;
        addr[3] = 0x10;

        addr[4] = 0x65;
        addr[5] = 0xf6;

        addr[6] = 0x8b;
        addr[7] = 0x26;

#if 0
        switch (own_address2) {
            case 0x65fd5836:   //m1
            case 0x65fd5936:  //m2
            case 0x65f85b06:  //m3
            case 0x65fa5402:  //m4
            case 0x65f95b02:  //m6
            case 0x65f75822:  //m7
            case 0x65f85b3a:  //m8
            case 0x65f8592a:  //m9
            case 0x65fa5a26:  //20
            case 0x65fc5932:  {//21
            // 15:11:6b:10:65:fd:54:36  //22
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xfd;

                addr[6] = 0x54;
                addr[7] = 0x36;
                break;
            }
            //////////////// cluster-1
            case 0x65f9581a:   //m11
            case 0x65f85406:  //m12
            case 0x65fa5822:  //m13
            case 0x65fa5b22:  //m14
            case 0x65fb5406:  //m15
            case 0x65fa5926:  //m17
            case 0x65fa5b36:  //m18
            case 0x65fa583a:  //m23
            case 0x65f55b32:  {//25
            // 15:11:6b:10:65:f5:58:02  //24
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xf5;

                addr[6] = 0x58;
                addr[7] = 0x02;
                break;
            }
            /////////////////////////// cluster-2
            case 0x65fa5a32:   //m37
            case 0x65f95832:  //m39
            case 0x65fc590a:  //m41
            case 0x65f65a22:  //m42
            case 0x65fd5902:  //m43
            case 0x65fa5836:  //m44
            case 0x65fa5922:  //m45
            case 0x65fa590a:  {//m46
            // 15:11:6b:10:65:fb:5b:26  //40
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xfb;

                addr[6] = 0x5b;
                addr[7] = 0x26;
                break;
            }
            /////////////////////////// cluster-3
            case 0x65fc5832:   //m29
            case 0x65f85a22:  //m30
            case 0x65fd593a:  //m31
            case 0x65f75b22:  //m32
            //case 0x65f85a22:  //m33
            case 0x65f95836:  //m35
            case 0x65f75802:  //m36
            case 0x65f65b36:  //m47
            case 0x65f45506:  //m48
            case 0x65f75a32:  //m49
            case 0x65f95a26:  {//m50
            // 15:11:6b:10:65:f8:5a:36  //34
                addr[0] = 0x15;
                addr[1] = 0x11;

                addr[2] = 0x6b;
                addr[3] = 0x10;

                addr[4] = 0x65;
                addr[5] = 0xf8;

                addr[6] = 0x5a;
                addr[7] = 0x36;
                break;
            }
            default: {
                return;
            break;
            }
        }
#endif

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
   	listen_period = random_uint32_range(30, 300);
 	printf("random wait: %lu s.\n", listen_period);

   	xtimer_sleep(listen_period);

   	data_rate = 120;
   	//puts("start pushing data!");

    while (1) {
	    for(int i=0; i<1; i++){   //65:f6:8b:26
	    if ((own_address2 != 0x65f68b26) && (send_counter < 5))
		    generate_and_send_pkt();
	    }
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

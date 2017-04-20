/*
 * Copyright (C) 2015 Freie Universität Berlin
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
 * @brief       Example application for demonstrating the RIOT network stack
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "thread.h"
#include "shell.h"
#include "msg.h"
#include "timex.h"
#include "random.h"
#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/nettype.h"
#include "xtimer.h"
#include <periph/rtt.h>
#include "net/gnrc/iqueue_mac/iqueuemac_types.h"

typedef struct iqueuemac iqueuemac_t;

uint32_t send_counter;
uint32_t send_counter1;
uint32_t send_counter2;
uint32_t own_address2;
uint32_t exp_start_time;
uint32_t exp_duration_ticks;

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);
extern void start_server(char *port_str);

extern int _gnrc_rpl_init(char *arg);
extern void udp_send(char *addr_str, char *port_str, uint32_t *data, size_t datasize, unsigned int num,
        unsigned int delay);

extern iqueuemac_t iqueuemac;

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};


static void generate_and_send_pkt(void){

    uint32_t num = 1;
    uint32_t delay = 0;

    char *add = "2001:db8::1";
    char *port = "8808";


    uint32_t payload[10];

   	send_counter++;

   	payload[0] = send_counter;
   	payload[1] = own_address2;

   	payload[5] = iqueuemac.awake_duration_sum;
   	payload[6] = rtt_get_counter() - iqueuemac.system_start_time;

    if(own_address2 != 0x5ad6) {
        udp_send(add, port, payload, sizeof(payload), num, delay);
    }
}


static void send_exp_feedback(void){

    uint32_t num = 1;
    uint32_t delay = 0;

    char *add = "2001:db8::1";
    char *port = "8808";

    uint32_t payload[10];

   	payload[0] = 0xEEEE;

   	//payload[5] = iqueuemac.awake_duration_sum;
   	//payload[6] = rtt_get_counter() - iqueuemac.system_start_time;

    if(own_address2 != 0x5ad6) {
        udp_send(add, port, payload, sizeof(payload), num, delay);
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
    uint32_t burst_num;
    data_rate = 0;
    total_gene_num = 0;

    /* setup the message queue */
    msg_init_queue(msg_queue, 8);

    send_counter = 0;
    send_counter1 = 0;
    send_counter2 = 0;

    int16_t devpid;

    devpid = 7;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));


    xtimer_sleep(3);

    own_address2 = 0;
    own_address2 = own_addr[0];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[1];

    printf("own add is %lx.\n", own_address2);

    /* RPL must be initialized on that particular interface-7 */
    char inface = '7';
    _gnrc_rpl_init(&inface);

	/* start udp server on port 8808 */
    char *udpport = "8808";
    start_server(udpport);

    gnrc_netreg_entry_t  me_reg = { .demux_ctx = GNRC_NETREG_DEMUX_CTX_ALL, .pid = thread_getpid() };
    gnrc_netreg_register(GNRC_NETTYPE_APP, &me_reg);


    burst_num = 1;

   while (1) {

        msg_receive(&msg);

        switch (msg.type) {
            case GNRC_NETAPI_MSG_TYPE_RCV: {
            	gnrc_pktsnip_t *pkt;
            	pkt = msg.content.ptr;
            	uint32_t *payload;

            	payload = pkt->data;

            	data_rate = payload[1];
            	total_gene_num = payload[2];
            	//exp_start_time = payload[5];
            	burst_num = payload[3];

            	exp_duration_ticks = payload[1];
            	exp_duration_ticks = exp_duration_ticks * 1000000;
            	exp_duration_ticks = RTT_US_TO_TICKS(exp_duration_ticks);

            	printf("the exp-data_rate is %lu. \n", data_rate);
            	printf("the exp-total_gene_num is %lu. \n", total_gene_num);

            	iqueuemac.exp_started = true;

            	gnrc_pktbuf_release(pkt);

            } break;

            default:
                puts("PKTDUMP: received something unexpected");
                break;
        }

        break;
    }

   send_exp_feedback();
   puts("send exp-feedback");

   exp_end = false;

   xtimer_sleep(100);

	/* start recording duty-cycle */
   iqueuemac.system_start_time = rtt_get_counter();

  	iqueuemac.last_radio_on_time = iqueuemac.system_start_time;
	iqueuemac.awake_duration_sum = 0;
	iqueuemac.radio_is_on = true;

	xtimer_sleep(150);

	uint32_t random_period;

	random_period = random_uint32_range(0, 60);

	xtimer_sleep(random_period);

   puts("start push data");

   while (1) {

   	xtimer_usleep((uint32_t) data_rate * 1000);

   	if((send_counter < total_gene_num)&&(exp_end == false)){  //total_gene_num
   		for(int i=0; i<burst_num; i++){
   			generate_and_send_pkt();
   		}
   		if (send_counter >= total_gene_num) {
   			printf("sender totally generated pkt num %lu .\n", send_counter);
   		}
   	}else {
   	   	if(exp_end == false) {
   	   		//printf("sender totally generated pkt num %lu .\n", send_counter);
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
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */

    thread_create(second_thread_stack, sizeof(second_thread_stack),
                            THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST,
                            sender_thread, NULL, "shuguo_app");


    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    
    /* should be never reached */
    return 0;
}

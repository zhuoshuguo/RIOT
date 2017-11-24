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
#include "net/gnrc/netdev.h"

typedef struct gnrc_netdev gnrc_netdev_t;

uint32_t send_counter;
uint32_t send_counter1;
uint32_t send_counter2;
uint32_t own_address2;
uint32_t exp_start_time;
uint32_t exp_duration_ticks;

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

extern int _gnrc_rpl_init(char *arg);
extern void udp_send(char *addr_str, char *port_str, uint32_t *data, size_t datasize, unsigned int num,
        unsigned int delay);

extern int _netif_add(char *cmd_name, kernel_pid_t dev, int argc, char **argv);

extern void start_server(char *port_str);

extern gnrc_netdev_t gnrc_netdev;

//extern int _gnrc_rpl_dodag_root(char *arg1, char *arg2);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};


static void generate_and_send_pkt(void){

    uint32_t num = 1;
    uint32_t delay = 0;

    char *add = "2001:db8::1"; //fe80::7b67:877:19f:331e
    char *port = "8808";


    uint32_t payload[10];

   	send_counter++;

   	payload[0] = send_counter;
   	payload[1] = own_address2;
   	// report tdma slots number.
   	payload[2] = gnrc_netdev.gomach.csma_count;
   	payload[3] = gnrc_netdev.gomach.vtdma_count;

   	uint64_t *payload_long = (uint64_t *)payload;

   	payload_long[2] = gnrc_netdev.gomach.awake_duration_sum_ticks;
   	payload_long[3] = xtimer_now_usec64() - gnrc_netdev.gomach.system_start_time_ticks;


    if(own_address2 != 0x5ad6) {
        udp_send(add, port, payload, sizeof(payload), num, delay);

        printf("P: %lx,%lu.\n", own_address2,send_counter);
    }
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

    devpid = 7;

    uint8_t own_addr[2];

    gnrc_netapi_get(devpid, NETOPT_ADDRESS, 0, &own_addr,
                            sizeof(own_addr));


    own_address2 = 0;
    own_address2 = own_addr[0];
    own_address2 = own_address2 << 8;
    own_address2 |= own_addr[1];

    /* configure a global IPv6 address for the root node
    char *ifconfig = "ifconfig";
    char *ipadd = "2001:db8::6";
    kernel_pid_t dev = 7;
    _netif_add(ifconfig, dev, 1, &ipadd);
    */

    xtimer_sleep(10);

    /* RPL must be initialized on that particular interface-7 */
    char inface = '7';
    _gnrc_rpl_init(&inface);

    /* start udp server on port 8808 */
    char *udpport = "8808";
    start_server(udpport);

    //xtimer_sleep(15);

    /* Starting RPL
    char *instanceid = "6";
    _gnrc_rpl_dodag_root(instanceid, ipadd);
    */
    while (1) {
        xtimer_sleep(10);
        if (RTT_TICKS_TO_MIN(rtt_get_counter()) >= 80) {
            break;
        }
    }

	uint32_t random_period;

	random_period = random_uint32_range(0, 60);

	xtimer_sleep(random_period);

   puts("start push data");
   data_rate = 60;

    while (1) {
        xtimer_sleep((uint32_t) data_rate);

        if (RTT_TICKS_TO_MIN(rtt_get_counter()) <= 150) {
    	    for(int i=0; i<5; i++){
                generate_and_send_pkt();
            }
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

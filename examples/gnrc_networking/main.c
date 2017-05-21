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


uint32_t send_counter;

uint32_t send_counter1;
uint32_t send_counter2;
uint32_t send_counter3;
uint32_t send_counter4;
uint32_t send_counter5;

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

extern int _netif_add(char *cmd_name, kernel_pid_t dev, int argc, char **argv);

extern int _gnrc_rpl_dodag_root(char *arg1, char *arg2);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};



static void generate_and_send_pkt(void){

    uint32_t num = 1;
    uint32_t delay = 0;

    char *port = "8808";

    uint32_t  expset[5];

	send_counter ++;

	/* data rate */
	expset[0] = own_address2;

	/* exp duration */
	//gnrc_netdev2->lwmac.exp_duration = 300; //seconds
	//expset[1] = gnrc_netdev2->lwmac.exp_duration;

	/* exp total generate packet number */
	expset[2] = 300;

	/* the burst number */
	expset[3] = 6;

	//char *add = "2001:db8::2"; //
    //udp_send(add, port, expset, sizeof(expset), num, delay);

    puts("send data");

    uint32_t sequence;
    sequence = send_counter % 5;

    if(sequence == 1) {
    	send_counter1 ++;
    	expset[1] = send_counter1;

    	char *add = "2001:db8::2"; //
        udp_send(add, port, expset, sizeof(expset), num, delay);
    } else if (sequence == 2) {
    	send_counter2 ++;
    	expset[1] = send_counter2;

    	char *add = "2001:db8::3";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (sequence == 3) {
    	send_counter3 ++;
    	expset[1] = send_counter3;

    	char *add = "2001:db8::4";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (sequence == 4) {
    	send_counter4 ++;
    	expset[1] = send_counter4;

    	char *add = "2001:db8::5";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (sequence == 0) {
    	send_counter5 ++;
    	expset[1] = send_counter5;

    	char *add = "2001:db8::6";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }
/*
    if(send_counter == 0) {
    	char *add = "2001:db8::5844:55d:4a55:6f46"; //
        udp_send(add, port, expset, sizeof(expset), num, delay);
    } else if (send_counter == 1) {
    	char *add = "2001:db8::5844:2b69:30f0:f22";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 2) {
    	char *add = "2001:db8::5844:1e68:a003:6142";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 3) {
    	char *add = "2001:db8::5855:605c:5109:447e";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 4) {
    	char *add = "2001:db8::5844:2c50:d550:a312";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 5) {
    	char *add = "2001:db8::5844:1f5f:809:e21a";//
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 6) {
    	char *add = "2001:db8::5844:3d54:41b8:52d2";
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 7) {
    	char *add = "2001:db8::5844:3a41:d643:1b1a";//
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 8) {
    	char *add = "2001:db8::5844:451:d774:bcc6";//
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }else if (send_counter == 9) {
    	char *add = "2001:db8::5844:2b54:22bc:103e";//
        udp_send(add, port, expset, sizeof(expset), num, delay);
    }
    */
}

void *sender_thread(void *arg)
{
    (void) arg;


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

    /* configure a global IPv6 address for the root node */
    char *ifconfig = "ifconfig";
    char *ipadd = "2001:db8::1";
    kernel_pid_t dev = 7;
    _netif_add(ifconfig, dev, 1, &ipadd);


    /* RPL must be initialized on that particular interface-7 */
    char inface = '7';
    _gnrc_rpl_init(&inface);

	/* start udp server on port 8808 */
    char *udpport = "8808";
    start_server(udpport);

    xtimer_sleep(20);
    //puts("start RPL");

    /* Starting RPL */
    char *instanceid = "1";
    _gnrc_rpl_dodag_root(instanceid, ipadd);

    xtimer_sleep(40);

   while (1) {
	   xtimer_sleep(5);

	   generate_and_send_pkt();

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

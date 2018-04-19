/*
 * Copyright (C) 2015 Inria
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
 * @brief       Basic ccn-lite relay example (produce and consumer via shell)
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 *
 * @}
 */

#include <stdio.h>

#include "thread.h"
#include "tlsf-malloc.h"
#include "msg.h"
#include "shell.h"
#include "ccn-lite-riot.h"
#include "net/gnrc/netif.h"
#include "xtimer.h"

//extern int _ccnl_interest(int argc, char **argv);
extern int _ccnl_content222(int argc, char **argv);

/* main thread's message queue */
#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

/* 10kB buffer for the heap should be enough for everyone */
#define TLSF_BUFFER     (10240 / sizeof(uint32_t))
static uint32_t _tlsf_heap[TLSF_BUFFER];

void *sender_thread(void *arg)
{
    (void) arg;

    ccnl_set_local_producer(_ccnl_content222);

    xtimer_sleep(25);
    puts("start ccnl_cont");

    char *arguments[3]={"ccnl_cont","/nancy","Hello"};

    _ccnl_content222(3, arguments);
    puts("end ccnl_cont");

    while (1) {
        ;
    }

    return NULL;
}

char second_thread_stack[THREAD_STACKSIZE_MAIN];


int main(void)
{

    thread_create(second_thread_stack, sizeof(second_thread_stack),
                            THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST,
                            sender_thread, NULL, "shuguo_app");



    tlsf_create_with_pool(_tlsf_heap, sizeof(_tlsf_heap));
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);

    puts("Basic CCN-Lite example");

    ccnl_core_init();

    ccnl_start();

    /* get the default interface */
    kernel_pid_t ifs[GNRC_NETIF_NUMOF];

    /* set the relay's PID, configure the interface to use CCN nettype */
    if ((gnrc_netif_get(ifs) == 0) || (ccnl_open_netif(ifs[0], GNRC_NETTYPE_CCN) < 0)) {
        puts("Error registering at network interface!");
        return -1;
    }

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);



    return 0;
}

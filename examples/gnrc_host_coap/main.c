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

#include "shell.h"
#include "msg.h"

#include "xtimer.h"
#include "thread.h"

#include "net/nanocoap.h"
#include "net/nanocoap_sock.h"

/* import "ifconfig" shell command, used for printing addresses */
extern int _gnrc_netif_config(int argc, char **argv);

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];


#ifndef THREAD_STACKSIZE_COAP
    #define THREAD_STACKSIZE_COAP (256U)
#endif

#ifndef COAP_INBUF_SIZE
    #define COAP_INBUF_SIZE (256U)
#endif

char coap_thread_stack[THREAD_STACKSIZE_COAP];

void *coap_thread(void *arg)
{
    (void) arg;

    printf("Starting CoAP Server on Port: %d\n", COAP_PORT);
    /* initialize nanocoap server instance */
    uint8_t buf[COAP_INBUF_SIZE];
    sock_udp_ep_t local = { .port=COAP_PORT, .family=AF_INET6 };
    nanocoap_server(&local, buf, sizeof(buf));

    printf("Error CoAP Server ended.\n");
    /* should be never reached */
    return NULL;
}



int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    puts("Waiting for address autoconfiguration...");
    xtimer_sleep(3);

    /* print network addresses */
    puts("Configured network interfaces:");
    _gnrc_netif_config(0, NULL);

    thread_create(coap_thread_stack, sizeof(coap_thread_stack),
                            THREAD_PRIORITY_MAIN + 1, THREAD_CREATE_STACKTEST,
                            coap_thread, NULL, "CoAP");

    /* wait for coap server to start*/
    xtimer_sleep(1);
    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(0, line_buf, SHELL_DEFAULT_BUFSIZE);

    printf("Error main ended.\n");
    /* should be never reached */
    return 0;
}

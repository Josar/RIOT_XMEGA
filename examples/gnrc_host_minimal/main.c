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

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

extern int udp_cmd(int argc, char **argv);

static const shell_command_t shell_commands[] = {
    { "udp", "send data over UDP and listen on UDP ports", udp_cmd },
    { NULL, NULL, NULL }
};

/* set interval to 1 second */
#define INTERVAL (600U * US_PER_SEC)// 10min
/*   600 000 000 */
/* 4 294 967 295 */

/*  410 065 408us */
/* 2400 000 000us */
void *second_thread(void *arg)
{
    (void) arg;
    xtimer_ticks32_t last_wakeup = xtimer_now();
    uint32_t runtime =0;

    while(1) {
        xtimer_periodic_wakeup(&last_wakeup, INTERVAL);
        runtime += 10;
        printf("Runtime counter: %9"PRIu32"min\n", runtime );
        printf("Runtime xtimer : %9"PRIu32"us\n\n", xtimer_now_usec() );
    }
    return NULL;
}

char second_thread_stack[256];

int main(void)
{
    /* we need a message queue for the thread running the shell in order to
     * receive potentially fast incoming networking packets */
    msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    puts("RIOT network stack example application");

    thread_create(second_thread_stack, sizeof(second_thread_stack),
                            THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST,
                            second_thread, NULL, "Alive");

    /* start shell */
    puts("All up, running the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    /* should be never reached */
    return 0;
}

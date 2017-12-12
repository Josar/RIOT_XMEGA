/*
 * Copyright (C) 2015 INRIA
 * Copyright (C) 2015 Eistec AB
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     core_util
 * @{
 *
 * @file
 * @brief       Crash handling functions
 *
 * @author      Kévin Roussel <Kevin.Roussel@inria.fr>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#include <string.h>
#include <stdio.h>

#include "assert.h"
#include "kernel_defines.h"
#include "cpu.h"
#include "irq.h"
#include "panic.h"
#include "periph/pm.h"
#include "log.h"

#if defined(DEVELHELP) && defined(MODULE_PS)
#include "ps.h"
#endif

#include "xtimer.h"

const char assert_crash_message[] = "FAILED ASSERTION.";

/* flag preventing "recursive crash printing loop" */
static int crashed = 0;

void __attribute__((weak)) panic_arch(void) {}

void print_uint64( uint64_t value ){

    char temp_arr[21];
    uint8_t i =19;
    uint8_t j=0;

    temp_arr[20]= '\0';

    while (value >0 ){
        temp_arr[i--] = '0' + value%10 ;
        value /= 10;
    }
    while( i<=19){
        temp_arr[j++] = (char )temp_arr[++i];
    }
    //LOG_ERROR("%s us\n", (temp_arr+19-i) );
    LOG_ERROR("%s us\n", temp_arr );
}

/* WARNING: this function NEVER returns! */
NORETURN void core_panic(core_panic_t crash_code, const char *message)
{
#ifdef NDEBUG
    (void) crash_code;
#endif

    if (crashed == 0) {
        /* print panic message to console (if possible) */
        crashed = 1;
#ifndef NDEBUG
        if (crash_code == PANIC_ASSERT_FAIL) {
            cpu_print_last_instruction();
        }
#endif
        LOG_ERROR("*** RIOT kernel panic:\n%s\n", message);
        LOG_ERROR("*** Runtime: "); print_uint64(xtimer_now_usec64());
#ifdef DEVELHELP
#ifdef MODULE_PS
        ps();
        LOG_ERROR("\n");
#endif

        LOG_ERROR("*** halted.\n\n");
#else
        LOG_ERROR("*** rebooting...\n\n");
#endif
    }
    /* disable watchdog and all possible sources of interrupts */
    irq_disable();
    panic_arch();
#ifndef DEVELHELP
    /* DEVELHELP not set => reboot system */
    pm_reboot();
#else
    /* DEVELHELP set => power off system */
    pm_off();
#endif

    /* tell the compiler that we won't return from this function
       (even if we actually won't even get here...) */
    UNREACHABLE();
}

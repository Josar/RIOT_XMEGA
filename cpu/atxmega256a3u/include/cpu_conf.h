/*
 * Copyright (C) 2015 Freie Universität Berlin, Hinnerk van Bruinehsen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_atxmega256a3u
 * @{
 *
 * @file
 * @brief           Implementation specific CPU configuration options
 *
 * @author          Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author          Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author          Josua Arndt <jarndt@ias.rwth-aachen.de>
 */

#ifndef CPU_CONF_H
#define CPU_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Kernel configuration
 *
 * Since printf seems to get memory allocated by the linker/avr-libc the stack
 * size tested sucessfully even with pretty small stacks.k
 * @{
 */
// core/include/debug.h prints if(sched_active_thread->stack_size > THREAD_EXTRA_STACKSIZE_PRINTF)
// so small THREAD_EXTRA_STACKSIZE_PRINTF prints more than a big one

/* keep
 * THREAD_STACKSIZE_IDLE > THREAD_EXTRA_STACKSIZE_PRINTF
 * to avoid not printing of debug in interrupts
 */
#ifndef THREAD_EXTRA_STACKSIZE_PRINTF
    #define THREAD_EXTRA_STACKSIZE_PRINTF    (128) //128
#endif

#ifndef THREAD_STACKSIZE_DEFAULT
    #define THREAD_STACKSIZE_DEFAULT   (512)
#endif
/** @} */

#ifndef THREAD_STACKSIZE_IDLE
    #define THREAD_STACKSIZE_IDLE      (129)//219
#endif
/**
 * @brief   Stack size used for the exception (ISR) stack
 * @{
 */
#define ISR_STACKSIZE              (0)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPU_CONF_H */
/** @} */

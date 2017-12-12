/*
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file
 * @brief       Low-level timer driver implementation for the ATmega family
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <avr/interrupt.h>

#include "board.h"
#include "cpu.h"
#include "thread.h"

#include "periph/timer.h"
#include "periph_conf.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"


/**
 * @brief   All timers have three channels
 */
#define CHANNELS                (2)

/**
 * @brief   We have 5 possible prescaler values
 */
#define PRESCALE_NUMOF          (7U)

/**
 * @brief   Possible prescaler values, encoded as 2 ^ val
 */
static const uint8_t prescalers[] = { 0, 1, 2, 3, 6, 8, 10 };

/**
 * @brief   Timer state context
 */
typedef struct {
	TC1_t *dev;          /**< timer device */
    timer_cb_t cb;              /**< interrupt callback */
    void *arg;                  /**< interrupt callback argument */
    uint8_t mode;               /**< remember the configured mode */
    uint8_t isrs;               /**< remember the interrupt state */
} ctx_t;


// #define TIMER_0_MASK        &TIMSK1
// #define TIMER_0_FLAG        &TIFR1

/**
 * @brief   Allocate memory for saving the device states
 * @{
 */
#ifdef TIMER_NUMOF
static ctx_t ctx[] = {
#ifdef TIMER_0
    { TIMER_0, NULL, NULL, 0, 0 },
#endif
#ifdef TIMER_1
    { TIMER_1, NULL, NULL, 0, 0 },
#endif
#ifdef TIMER_2
    { TIMER_2, NULL, NULL, 0, 0 },
#endif
#ifdef TIMER_3
    { TIMER_3, NULL, NULL, 0, 0 },
#endif
};
#else
/* fallback if no timer is configured */
static ctx_t *ctx[] = {{ NULL }};
#endif
/** @} */

/**
 * @brief Setup the given timer
 */
int timer_init(tim_t tim, unsigned long freq, timer_cb_t cb, void *arg)
{
    DEBUG("timer.c: freq = %ld, Core Clock = %ld\n", freq, CLOCK_CORECLOCK);


    uint8_t pre = 0;

    /* make sure given device is valid */
    if (tim >= TIMER_NUMOF) {
        return -1;
    }

    /* figure out if freq is applicable */
    for (; pre < PRESCALE_NUMOF; pre++) {
        if ((CLOCK_CORECLOCK >> prescalers[pre]) == freq) {
            break;
        }
    }
    if (pre == PRESCALE_NUMOF) {
        DEBUG("timer.c: prescaling failed!\n");
        return -1;
    }

    /* stop and reset timer */
    ctx[tim].dev->CTRLA = 0; // Stop
    ctx[tim].dev->CTRLFSET = TC_CMD_RESET_gc ; // Force Reset

    /* save interrupt context and timer Prescaler */
    ctx[tim].cb   = cb;
    ctx[tim].arg  = arg;
    ctx[tim].mode  = ( 0x0F & (pre+1) ) ;



    /* enable timer with calculated prescaler */
    ctx[tim].dev->CTRLA = ctx[tim].mode;
    DEBUG("timer.c: prescaler set to %d \n", ctx[tim].dev->CTRLA );

    return 0;
}

int timer_set(tim_t tim, int channel, unsigned int timeout)
{
	DEBUG("timer_set channel %d to %u\n", channel, timeout );
	return timer_set_absolute(tim, channel, timer_read(tim) + timeout);
}

int timer_set_absolute(tim_t tim, int channel, unsigned int value)
{
    if (channel >= CHANNELS) {
        return -1;
    }
    DEBUG("timer_set_absolute channel %d to %u\n", channel, value );

    // Compare or Capture Disable
    ctx[tim].dev->CTRLB &= ~(1 << (channel + TC0_CCAEN_bp));

    // Clear Interrupt Flag
    ctx[tim].dev->INTFLAGS &= ~(1 << (channel + TC0_CCAIF_bp));

    // set value to compare
    *(((uint16_t*)(&ctx[tim].dev->CCA))+channel)= (uint16_t)value;

    /* Compare or Capture Interrupt  Medium Level */
    ctx[tim].dev->INTCTRLB |= ( TC_CCAINTLVL_HI_gc << channel );

    /* Enable PMIC interrupt level medium. */
    PMIC.CTRL |= PMIC_HILVLEN_bm;

    // Compare or Capture Enable
    ctx[tim].dev->CTRLB |= (1 << (channel + TC0_CCAEN_bp));

    return 1;
}

int timer_clear(tim_t tim, int channel)
{
    if (channel >= CHANNELS) {
        return -1;
    }

    DEBUG("timer_clear channel %d\n", channel  );

    // Compare or Capture Disable
    ctx[tim].dev->CTRLB &= ~(1 << (channel + TC0_CCAEN_bp));

    // Clear Interrupt Flag
    // ctx[tim].dev->INTFLAGS &= ~(1 << (channel + TC0_CCAIF_bp));
  //The CCxIF is automatically cleared when the corresponding interrupt vector is executed.

    return 0;
}

unsigned int timer_read(tim_t tim)
{
	DEBUG("timer_read");
	return (unsigned int)ctx[tim].dev->CNT;
}

void timer_stop(tim_t tim)
{
	DEBUG("timer_stop");
    ctx[tim].dev->CTRLA = 0;
}

void timer_start(tim_t tim)
{
	DEBUG("timer_start");
    ctx[tim].dev->CTRLA = ctx[tim].mode;
}

#ifdef TIMER_NUMOF
static inline void _isr(tim_t tim, int channel)
{
    __enter_isr();

    DEBUG("timer _isr channel %d\n", channel);

    // Compare or Capture Disable
    ctx[tim].dev->CTRLB &= ~(1 << (channel + TC0_CCAEN_bp));

    ctx[tim].cb(ctx[tim].arg, channel);

    //PORTE.OUTTGL = PIN7_bm ;

    if (sched_context_switch_request) {
        thread_yield();
    }

    __exit_isr();
}
#endif

#ifdef TIMER_0
ISR(TIMER_0_ISRA, ISR_BLOCK)
{
    _isr(0, 0);
}

ISR(TIMER_0_ISRB, ISR_BLOCK)
{
    _isr(0, 1);
}
ISR(TIMER_0_OVF, ISR_BLOCK)
{

	// _isr(0, 2);
}
#endif /* TIMER_0 */



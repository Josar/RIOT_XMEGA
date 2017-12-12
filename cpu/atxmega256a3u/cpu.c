/*
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atmega256rfr2
 * @{
 *
 * @file
 * @brief       Implementation of the CPU initialization
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @}
 */

#include "cpu.h"
#include "periph/init.h"

#include "board.h"
#include <avr/io.h>

#include "avr/wdt.h"
#include "avr/pgmspace.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/**
 * @brief Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
	wdt_reset();
	wdt_disable();

    /* trigger static peripheral initialization */
    periph_init();
}

/* This is a vector which is aliased to __vector_default,
 * the vector executed when an ISR fires with no accompanying
 * ISR handler. This may be used along with the ISR() macro to
 * create a catch-all for undefined but used ISRs for debugging
 * purposes.
 */
// SCIRQS – Symbol Counter Interrupt Status Register
// BATMON – Battery Monitor Control and Status Register
// IRQ_STATUS /1 – Transceiver Interrupt Status Register
// EIFR – External Interrupt Flag Register
// PCIFR – Pin Change Interrupt Flag Register

ISR(BADISR_vect)
{
	while(1){

		if( (PORTF.IN & PIN3_bm)!=0 ){
			  PORTF.OUTSET = PIN2_bm ;

			  PORTF.OUTCLR = PIN3_bm ;
		 }else
		 {
			 PORTF.OUTCLR = PIN2_bm ;
			 PORTF.OUTSET = PIN3_bm ;
		 }

		 _delay_ms( 50 );
	};
}

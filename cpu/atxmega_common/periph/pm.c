/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *               2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atmega_common
 * @{
 *
 * @file
 * @brief       Implementation of common AVR periph/pm functions
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */
#include "irq.h"
#include "periph/pm.h"

#include "periph_conf.h"

#include "cpu.h"
#include "board.h"
#include "xtimer.h"
#include <stdio.h>

#include "thread.h"

void pm_off(void)
{
	// TODO
	printf("pm_off\n");
}

void pm_reboot(void)
{
    printf("pm_reboot\n");
	reboot();
}

void pm_set_lowest(void) {

//    printf("\npm_set_lowest\n");
//    thread_stack_print();


//	PORTF.OUTCLR = PIN3_bm ;
//	PORTF.OUTCLR = PIN2_bm ;
//	PORTE.OUTCLR = PIN7_bm ;
//	PORTE.OUTCLR = PIN6_bm ;

//	_delay_ms(100);

//	PORTF.OUTSET = PIN3_bm ;
//	PORTF.OUTSET = PIN2_bm ;
//	PORTE.OUTSET = PIN6_bm ;
//	PORTE.OUTSET = PIN7_bm ;

//	_delay_ms(100);


	/*TODO implement power save Modes*/
}

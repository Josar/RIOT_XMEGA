/*
 * Copyright (C) 2018 Josau Arndt
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
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 *
 * @}
 */
#include "irq.h"
#include "periph/pm.h"

#include "cpu.h"
#include "xtimer.h"
#include <stdio.h>

#include "thread.h"
#include "board.h"
#include "periph_conf.h"

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

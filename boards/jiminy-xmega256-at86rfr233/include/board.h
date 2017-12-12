/*
 * Copyright (C) 2016 RWTH Aachen, Josua Arndt
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_pinoccio-mega256rfr2 Pinoccio Mega 256rfr2
 * @ingroup     boards
 * @brief       Board specific files for the Pinoccio Mega 256rfr2 board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for Pinoccio Mega 256rfr2 board.
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Clock configuration
 * @{
 */
#define CLOCK_CORECLOCK     (32000000UL)

/** @} */

/**
* @brief As the CPU is too slow to handle 115200 baud, we set the default
*        baudrate to 9600 for this board
*/

#define UART_STDIO_BAUDRATE (460800)
#define UART_STDIO_DEV (0)

int16_t xmega_calculate_bsel_bscale(uint32_t fcpu, uint32_t baud, uint8_t* clk2x,  uint16_t* bsel , int8_t* bscale);

/**
 * @brief   LED pin definitions and handlers
 * @{
 */
#define LED_PORT		PORTF

#define LED1_bm			PIN3_bm
#define LED2_bm			PIN2_bm

/*
* Context Swap Interrupt with Port A Pin 1
*
*
*
*/
#define AVR_CONTEXT_SWAP_INIT do { \
	cli();\
	PORTA.DIR |= PIN1_bm; \
	PORTA.OUTCLR= 	PIN1_bm; \
	PORTA.INT0MASK = PIN1_bm; \
	PORTA.PIN1CTRL	= PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc; \
	PORTA.INTCTRL	= PORT_INT0LVL_MED_gc; \
	PMIC.CTRL|= PMIC_MEDLVLEN_bm;\
	sei();\
} while (0)

#define AVR_CONTEXT_SWAP_INTERRUPT_VECT PORTA_INT0_vect

#define AVR_CONTEXT_SWAP_TRIGGER do { \
	PORTA.OUTTGL=PIN1_bm;\
}while(0)

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */

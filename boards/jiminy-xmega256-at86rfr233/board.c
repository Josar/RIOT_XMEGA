/*
 * Copyright (C) 2016 RWTH Aachen, Josua Arndt
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *               2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_pinoccio-mega256rfr2
 * @{
 *
 * @file
 * @brief       Board specific implementations for the Pinoccio Mega 256rfr2 board
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 *
 * @}
 */
#include <avr/cpufunc.h>

#include "board.h"

#include <stdio.h>
#include <avr/io.h>

#include "cpu.h"
#include "uart_stdio.h"

#include "periph/init.h"

void system_stdio_init(void);
static int uart_putchar(char c, FILE *stream);
static int uart_getchar(FILE *stream);

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE uart_stdin = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

#define time_to_wait 500


void clk_init(void)
{
    uint8_t *reg = (uint8_t *)&PR.PRGEN;
    uint8_t i;

        /* Turn off all peripheral clocks that can be turned off. */
        for (i = 0; i <= 7; i++) {
            *(reg++) = 0xff;
        }

        reg = (uint8_t *)&PR.PRGEN;
        /* Turn on all peripheral clocks that can be turned on. */
        for (i = 0; i <= 7; i++) {
            *(reg++) = 0x00;
        }

        /* XMEGA A3U [DATASHEET] p.23 After reset, the device starts up running from the 2MHz internal oscillator.
         * The other clock sources, DFLLs and PLL, are turned off by default.*/

        // Configure clock to 32MHz with calibration
        // application note AVR1003

        // From errata http://www.avrfreaks.net/forum/xmega-dfll-does-it-work
        // In order to use the automatic runtime calibration for the 2 MHz or the 32 MHz internal oscillators, the DFLL for both oscillators and both oscillators has to be enabled for one to work.
        // TODO Test if this is also relevsant for the atxmega256a3u
        // Quick fix just let the 2MHz clock running
        OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;  /* Enable the internal 32MHz & 32KHz oscillators */
        while(!(OSC.STATUS & OSC_RC32KRDY_bm));       /* Wait for 32Khz oscillator to stabilize */
        while(!(OSC.STATUS & OSC_RC32MRDY_bm));       /* Wait for 32MHz oscillator to stabilize */
        DFLLRC32M.CTRL = DFLL_ENABLE_bm ;             /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
        DFLLRC2M.CTRL = DFLL_ENABLE_bm ;             /* Enable DFLL - defaults to calibrate against internal 32Khz clock */

        /* Disable CCP for Protected IO register and set new value*/
        /* Set system clock prescalers to zero */
        /* PSCTRL contains A Prescaler Value and one value for and B and C Prescaler */
         _PROTECTED_WRITE(CLK.PSCTRL, CLK_PSADIV_1_gc|CLK_PSBCDIV_1_1_gc);
         /*
            * Previous instruction takes 3 clk cycles with -Os option
            * we need another clk cycle before we can reuse it.
            */
         __asm__ __volatile__("nop");

         /* Disable CCP for Protected IO register and set new value*/
         /* Switch to 32MHz clock */
         _PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc);

        // OSC.CTRL &= ~OSC_RC2MEN_bm;                   /* Disable 2Mhz oscillator */


         /* wait for calibratin to finisch */
    //      _delay_ms( time_to_wait );


         /*Set Perifery clock*/
}


void board_init(void)
{

    /* initialize the CPU */
    cpu_init();
    /* Setup clock and auto calibration */
    clk_init();

    /* initialize stdio via USART_0 */
    system_stdio_init();

    /* trigger static peripheral initialization */
    periph_init();

	/* config LEDs*/
	PORTF.DIRSET =  PIN3_bm|PIN2_bm ; // Set pins 2, 3 on port F to be output.
	PORTE.DIRSET =  PIN7_bm|PIN6_bm ;

	PORTF.OUTCLR = PIN3_bm|PIN2_bm ; // Set pins 2, 3 on port F to low.
	PORTE.OUTCLR = PIN7_bm|PIN6_bm ;


    //TODO led init

    /* initialize the board LED (connected to pin PB7) */
    /* Ports Pins as Output */
//    LED_PORT_DDR |= BLUE|RED|GREEN;
    /* All Pins High so LEDs are off */
//    LED_PORT |= BLUE|RED|GREEN;

     irq_enable();

//    puts("Board init");

}

/**
 * @brief Initialize the System, initialize IO via UART_0
 */
void system_stdio_init(void)
{

    /* initialize Pins used for stdout */
	PORTC.DIRSET =  PIN7_bm ; // output for UART_TX
	PORTC.DIRCLR =  PIN6_bm ; // input for UART_RX


	/* initialize UART_0 for use as stdout */
    uart_stdio_init();

    stdout = &uart_stdout;
    stdin = &uart_stdin;

    /* Flush stdout */
    /* Make a very visible start printout */
//    puts("\n___________\r\n");
}

static int uart_putchar(char c, FILE *stream)
{
    (void) stream;
    uart_stdio_write(&c, 1);
    return 0;
}

int uart_getchar(FILE *stream)
{
    (void) stream;
    char c;
    uart_stdio_read(&c, 1);
    return (int)c;
}

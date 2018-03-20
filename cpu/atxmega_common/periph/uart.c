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
 * @brief       Low-level UART driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 *
 * Support static BAUD rate calculation using UART_STDIO_BAUDRATE.
 * Set UART_STDIO_BAUDRATE to the desired baud rate and pass it as a -D argument
 * at compliation time (e.g. in the boards Makefile.include file).
 * UART_BAUD_TOL can be set to guarantee a BAUD rate tolerance at compile time or
 * to switch to double speed transmission (U2X) to achieve a lower tolerance.
 * At runtime, this tolerance is not guaranteed to be met.
 * However, an error message will be displayed at compile time.
 *
 * @}
 */

#include "cpu.h"
#include "sched.h"
#include "thread.h"

#include "periph/uart.h"

#include <avr/io.h>

#include <stdlib.h>
#include <stdio.h>

#include "../../../boards/jiminy-xmega256-at86rf233/include/board.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Maximum percentage error in calculated baud before switching to
 *          double speed transmission (U2X)
 *
 * Takes whole numbers from 0 to 100, inclusive, with a default of 2.
 */
#if defined(UART_BAUD_TOL)
/* BAUD_TOL is defined here as it is used by the setbaud.h utility */
#define BAUD_TOL UART_BAUD_TOL
#else
#define BAUD_TOL 200
#endif

int16_t xmega_calculate_bsel_bscale(uint32_t fcpu, uint32_t baud, uint8_t* clk2x,
                                    uint16_t* bsel , int8_t* bscale);

/**
 * @brief   Configured device map
 * @{
 */
#if UART_NUMOF
static USART_t* dev[] = {
#ifdef UART_0
    UART_0,
#endif
#ifdef UART_1
    UART_1,
#endif
#ifdef UART_2
    UART_2,
#endif
#ifdef UART_3
    UART_3
#endif
};
#else
/* fallback if no UART is defined */
static const mega_uart_t *dev[] = { NULL };
#endif

/**
 * @brief   Allocate memory to store the callback functions.
 */
static uart_isr_ctx_t isr_ctx[UART_NUMOF];

int8_t _check_bsel(uint32_t* baud, uint32_t* calcbaud,int16_t* precision ){

    /* Avoid negative values and with precision of two positions
     * after the decimal point for the deviation in percent
     *
     * result is the absolute deviation eg. 10 001
     */
    /* MAX BAUD fper/2 = 16MHz a shift of 8 can be used to increase accuracy */
    uint16_t pre;

    if( *baud < *calcbaud ){
        pre = (uint16_t)((((uint64_t)(*calcbaud )*10000)/ (*baud)));
    }else
    {
        pre = (uint16_t)(((uint64_t)(*baud)*10000)/ (*calcbaud));
    }

    if( pre < ((uint16_t)*precision)){
//        DEBUG("baud %" PRIu32 " calcbaud %" PRIu32 " \n", *baud, *calcbaud);
//        DEBUG("Absolute deviation in percent %" PRIu32 " \n", pre);
        *precision = pre;
        return 1;
    }
    return 0;

}

int16_t _xmega_bsel_bscale(uint32_t* fper, uint32_t* baud, uint8_t clk2x,
                           uint16_t* bsel , int8_t* bscale ){

    uint32_t calcbaud =0;
    int16_t precision =UINT_MAX;

    int32_t sub=0;
    uint32_t deno = 0;
    uint32_t num = 0;

    uint16_t locBsel=0;
    int8_t locBscale;

    /* Some explanation for equation transformation
     * bsel =  ( (fper / (2^bscale*16*baud) ) - 1 )
     *      =  ( (fper / (2^bscale*(16-8*clk2x)*baud) ) - 1 )
     *      =  (  fper - (2^bscale*(16-8*clk2x)*baud) ) / (2^bscale*(16-8*clk2x)*baud)

     * deno = ( baud * (16-8*clk2x)   * 2^bscale    )
     *      = ( baud * (1<<(4-clk2x)) * (1<<bscale) )
     *      = ( baud<<( 4-clk2x +bscale));
     *
     * some rounding math explanation for unsigned integer
     * ABS(x +0,5)  = (x*10+ 5 )/(10)
     * Equation above is as follows, using denominator as base to round
     * bsel = ( x    - y )/y
     *      = ( x    - y +0,5*y )/y
     *      = ( 2*x  - y )/(2*y)
     *      = ((x<<1)- y )/(y<<1)
     *
     * baud = (fper*1/2^bscale) / ((bsel+1)*16)
     */
    for( locBscale=0; locBscale<=7; locBscale++ ){

        deno = ((*baud)<<(4-clk2x+locBscale));
        sub  = ((*fper)<<1) - (deno);

        if(sub <=0){
            break;
        }

        locBsel = (sub/(deno<<1));

        if(locBsel >=4095){
            continue;
        }

        /* Omit division by 16 get higher accuracy at small baudrates*/
        calcbaud = ((*fper)>>locBscale)/( (locBsel+1)<<(4-clk2x) );

        if( _check_bsel( baud , &calcbaud , &precision) ){
            *bsel = locBsel;
            *bscale = locBscale;
        }
    }

    /* More math for the negative equation
     * bscale is negative so 1/2^bscale = 2^|bscale| which is a factor and not a
     * division again runding the result before division with 0.5*denominator
     *
     * bsel = 1/2^bscale *( fcpu / ( (16*baud)-1) )
     *      = (  fcpu*2^|bscale| - (16*baud)*2^|bscale| )/(16*baud)
     *      = (  fcpu*2^|bscale| - (16*baud)*2^|bscale| + 0.5*(16*baud) )/(16*baud)
     *
     * deno  =  (16/2*baud) = (baud<<(3-clk2x))
     *
     * bsel = (  (fcpu - (deno<<(1))<<|bscale|) + deno )/(deno<<1)
     *
     * Baud = (fper*1/2^bscale)/ (16*(bsel+1/2^bscale))
     */
    for( locBscale=-1; locBscale>=-7; locBscale-- ){

        deno = ((*baud)<<(3-clk2x));
        num  = (((*fper)-(deno<<1))<<(-locBscale))+deno;
        num  = (num/(deno<<1));

        if(num >=4095){
            break;
        }

        locBsel = (uint16_t)( num & 0xFFF);

        /* Omit division by 16 get higher accuracy at small baudrates*/
        calcbaud = ((*fper)<<(-locBscale))/((locBsel+(1<<(-locBscale))) <<(4-clk2x));

        if( _check_bsel( baud, &calcbaud , &precision) ){
            *bsel = locBsel;
            *bscale = locBscale;
        }
    }

    return precision;
}

/**
 * @brief   Calculates bsel and bscale for a given periphery clock and baudrate.
 *          Limitation are the periphery clock maximum is 32MHz, unsigned int
 *          overflow if bigger. And the periphery clock has to be not smaller
 *          then 1 when divided by 128.
 *
 * fper/128 !=0 must be devide able by 128
 * fper*128 != uint23_max => 32MHz max fper
 */
int16_t xmega_calculate_bsel_bscale(uint32_t fcpu, uint32_t baud, uint8_t* clk2x,
                                    uint16_t* bsel , int8_t* bscale){

    int16_t precision =0;

    precision = _xmega_bsel_bscale(&fcpu, &baud, 0, bsel, bscale );

    /* default 0,1% precision, required precision is at least 2% */
    if(precision <= (10000+BAUD_TOL)){
        return (precision-10000);
    }

    precision = _xmega_bsel_bscale(&fcpu, &baud, 1, bsel, bscale );
    *clk2x = 1;

    return (precision-10000);
}

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    int8_t  bscale;
    uint8_t clk2x;
    uint16_t bsel;

    /* make sure the given device is valid */
    if (uart >= UART_NUMOF) {
        return UART_NODEV;
    }

    /* register interrupt context */
    isr_ctx[uart].rx_cb = rx_cb;
    isr_ctx[uart].arg   = arg;

    /* disable and reset UART */
    dev[uart]->CTRLA = 0;
    dev[uart]->CTRLB = 0;
    dev[uart]->CTRLC = 0;

    /* configure UART to 8N1 mode */
    dev[uart]->CTRLC = USART_CMODE_ASYNCHRONOUS_gc|USART_PMODE_DISABLED_gc| USART_CHSIZE_8BIT_gc ;

    /* set clock divider */
    xmega_calculate_bsel_bscale(CLOCK_CORECLOCK, baudrate, &clk2x, &bsel, &bscale);

    dev[uart]->BAUDCTRLA = (uint8_t)(bsel&0x00ff);
    dev[uart]->BAUDCTRLB = (bscale<<USART_BSCALE_gp) | ((uint8_t)((bsel&0x0fff)>>8));
    if(clk2x==1){
       dev[uart]->CTRLB |= USART_CLK2X_bm;
    }

     /* enable RX and TX Interrupts and set level*/
    if (rx_cb) {

    	dev[uart]->CTRLA = USART_RXCINTLVL_HI_gc ;
        dev[uart]->CTRLB = 0x10 | 0x08 ;

        /* Enable PMIC interrupt level low. */
        PMIC.CTRL |= PMIC_HILVLEN_bm;
    }
    else {
    	/* only transmit */
    	 dev[uart]->CTRLB =  USART_TXEN_bm ;
    }

    sei();

    _delay_ms(100);
    DEBUG("Set clk2x %" PRIu8 " bsel %" PRIu16 "bscale %" PRIi8 "\n", clk2x, bsel, bscale);
    return UART_OK;
}

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
	// PORTE.OUTSET = PIN6_bm ;
	for (size_t i = 0; i < len; i++) {
		while ( !( dev[uart]->STATUS & USART_DREIF_bm)  ) {};
		dev[uart]->DATA = data[i];
    }
	//PORTE.OUTCLR = PIN6_bm ;
}

static inline void isr_handler(int num)
{
    if( (dev[num]->STATUS & USART_RXCIF_bm) == 0) {
    	return;
    }

	isr_ctx[num].rx_cb(isr_ctx[num].arg, dev[num]->DATA);

    if (sched_context_switch_request) {
        thread_yield();
    }
}

#ifdef UART_0_RXC_ISR
ISR(UART_0_RXC_ISR, ISR_BLOCK)
{
    __enter_isr();
    isr_handler(0);
    __exit_isr();
}
#endif /* UART_0_RXC_ISR */

#ifdef UART_1_RXC_ISR
ISR(UART_1_RXC_ISR, ISR_BLOCK)
{
    __enter_isr();
    isr_handler(1);
    __exit_isr();
}
#endif /* UART_1_RXC_ISR */

#ifdef UART_2_RXC_ISR
ISR(UART_0_RXC_ISR, ISR_BLOCK)
{
    __enter_isr();
    isr_handler(2);
    __exit_isr();
}
#endif /* UART_2_RXC_ISR */

#ifdef UART_3_RXC_ISR
ISR(UART_3_RXC_ISR, ISR_BLOCK)
{
    __enter_isr();
    isr_handler(3);
    __exit_isr();
}
#endif /* UART_3_ISR */

/*
 * Copyright (C) 2015 Daniel Amkaer Sorensen
 *               2016 Freie Universität Berlin
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
 * @brief       Low-level SPI driver implementation for ATmega family
 *
 * @author      Daniel Amkaer Sorensen <daniel.amkaer@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */


#include "cpu.h"
#include "periph_cpu.h"

#include "mutex.h"
#include "assert.h"
#include "periph/gpio.h"
#include "periph/spi.h"

/**
 * @brief   Define base SPI Port address
 * @{
 */
#define SPI_PORT_BASE      ((uint16_t)&PORTC)
#define SPI_BUS_BASE      ((uint16_t)&SPIC)
#define SPI_BUS_OFFSET    (0x100)
/** @} */

static mutex_t lock = MUTEX_INIT;

/**
 * @brief     generate the bus base address of the given bus
 */
static inline SPI_t* bus_base(spi_t bus)
{
    return ((SPI_t*) (SPI_PORT_BASE +(SPI_BUS_OFFSET*bus)));
}
// bus number from base address
//(((SPI_t*)bus) - SPI_BASE_PORT)/SPI_PORT_OFFSET;
//uint8_t bus_num = _pin_num(bus);


/* TODO find general way to configer power register */
void spi_init(spi_t bus)
{

    /* power off the SPI peripheral */
    PR.PRPD |= PR_SPI_bm;

//   PR.PRPD = PR_SPI_bm;
   /* trigger the pin configuration */
   spi_init_pins(bus);
}


void spi_init_pins(spi_t bus)
{

//    SPI_t* spi = bus_base(bus);


    (void)bus;
    /* the pin configuration for this CPU is for now :
     * - PD4: SS   (configure as output, but unused)
     * - PD5: MOSI (configure as output)
     * - PD6: MISO (configure as input - done automatically)
     * - PD7: SCK  (configure as output)
     *
     * The SS pin must be configured as output for the SPI device to work as
     * master correctly, though we do not use it for now (as we handle the chip
     * select externally for now)
     */

    PORTD.DIRSET = AT86RF2XX_PARAM_RESET ; // Reset Pin as output
    PORTD.OUTCLR = AT86RF2XX_PARAM_RESET ; // low, reset

    PORTD.DIRCLR = AT86RF2XX_PARAM_INT ; // interrupt as input

    PORTD.DIRSET = AT86RF2XX_PARAM_SLEEP ; // SLT Pin as output
    PORTD.OUTCLR = AT86RF2XX_PARAM_SLEEP ; // low


    PORTD.OUTSET = AT86RF2XX_PARAM_CS; // high

    PORTD.DIRSET = AT86RF2XX_PARAM_CS; // output
    PORTD.DIRSET = AT86RF2XX_PARAM_MOSI; // output
    PORTD.DIRSET = AT86RF2XX_PARAM_SPI_CLK_PIN; // output

    PORTD.DIRCLR = AT86RF2XX_PARAM_MISO; //input


    PORTD.OUTSET = AT86RF2XX_PARAM_RESET ; // high, disable reset
}

int spi_init_cs(spi_t bus, spi_cs_t cs){

	(void)bus;
	(void)cs;
    PORTD.OUTSET = AT86RF2XX_PARAM_CS;

    return 0;
}

int spi_acquire(spi_t bus, spi_cs_t cs, spi_mode_t mode, spi_clk_t clk)
{
    (void)cs;
    (void)mode;
    (void)clk;
    (void)bus;

    /* lock the bus and power on the SPI peripheral */
    mutex_lock(&lock);
    PR.PRPD &= ~PR_SPI_bm;

    /* configure active, as master, with given mode and clock */
    SPID.CTRL = SPI_ENABLE_bm| SPI_MASTER_bm |SPI_CLK2X_bm | SPI_PRESCALER0_bm;

    /* clear interrupt flag by reading STATUS register by reading DATA */
    (void)SPID.STATUS;
    (void)SPID.DATA;

    /* Enable Module*/
    //SPID.CTRL |=  SPI_ENABLE_bm;

    return SPI_OK;
	return 0;
}

void spi_release(spi_t bus)
{
	(void)bus;
    /* Disable Module*/
    SPID.CTRL &= ~ SPI_ENABLE_bm;

    /* power off */
    PR.PRPD |= PR_SPI_bm;

    mutex_unlock(&lock);
}

void spi_transfer_bytes(spi_t bus, spi_cs_t cs, bool cont,
                        const void *out, void *in, size_t len)
{
	(void)bus;

    uint8_t *out_buf = (uint8_t *)out;
    uint8_t *in_buf = (uint8_t *)in;

    assert(out_buf || in_buf);

    if (cs != SPI_CS_UNDEF) {
        // gpio_clear((gpio_t)cs);
        PORTD.OUTCLR = AT86RF2XX_PARAM_CS;
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = (out_buf) ? out_buf[i] : 0;

        SPID.DATA = tmp;

        while(!(SPID.STATUS & SPI_IF_bm )){}

        tmp = SPID.DATA ;

        if (in_buf) {
            in_buf[i] = tmp;
        }
    }

    if ((!cont) && (cs != SPI_CS_UNDEF)) {
        //gpio_set((gpio_t)cs);
        PORTD.OUTSET = AT86RF2XX_PARAM_CS;
    }
}

uint8_t spi_transfer_byte(spi_t bus, spi_cs_t cs, bool cont, uint8_t out)
{
	(void)bus;

        if (cs != SPI_CS_UNDEF) {
           // gpio_clear((gpio_t)cs);
           PORTD.OUTCLR = AT86RF2XX_PARAM_CS;
       }

       SPID.DATA = out;

       while( !(SPID.STATUS & SPI_IF_bm )){}


       if ((!cont) && (cs != SPI_CS_UNDEF)) {
           //gpio_set((gpio_t)cs);
           PORTD.OUTSET = AT86RF2XX_PARAM_CS;
       }

       return SPID.DATA ;
}

uint8_t spi_transfer_reg(spi_t bus, spi_cs_t cs, uint8_t reg, uint8_t out)
{
    assert(reg);
    (void)bus;

    if (cs != SPI_CS_UNDEF) {
        // gpio_clear((gpio_t)cs);
        PORTD.OUTCLR = AT86RF2XX_PARAM_CS;
    }

    SPID.DATA = reg;

    while(!(SPID.STATUS & SPI_IF_bm )){}

    (void) SPID.DATA; // clear SPI_IF_bm flag

    SPID.DATA = (out) ? out: 0;

    while(!(SPID.STATUS & SPI_IF_bm )){}

    if (cs != SPI_CS_UNDEF) {
        //gpio_set((gpio_t)cs);
        PORTD.OUTSET = AT86RF2XX_PARAM_CS;
    }

    return SPID.DATA ;
}

void spi_transfer_regs(spi_t bus, spi_cs_t cs, uint8_t reg,
                       const void *out, void *in, size_t len)
{
	(void)bus;
    uint8_t *out_buf = (uint8_t *)out;
        uint8_t *in_buf = (uint8_t *)in;

        assert(out_buf || in_buf);

        if (cs != SPI_CS_UNDEF) {
            // gpio_clear((gpio_t)cs);
            PORTD.OUTCLR = AT86RF2XX_PARAM_CS;
        }

        SPID.DATA = reg;

        while(!(SPID.STATUS & SPI_IF_bm )){}

        for (size_t i = 0; i < len; i++) {
            uint8_t tmp = (out_buf) ? out_buf[i] : 0;

            SPID.DATA = tmp;

            while(!(SPID.STATUS & SPI_IF_bm )){}

            tmp = SPID.DATA ;

            if (in_buf) {
                in_buf[i] = tmp;
            }
        }

        if (cs != SPI_CS_UNDEF) {
            //gpio_set((gpio_t)cs);
            PORTD.OUTSET = AT86RF2XX_PARAM_CS;
        }
}


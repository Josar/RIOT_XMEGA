/*
 * Copyright (C) 2015 HAW Hamburg
 *               2016 INRIA

 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_atmega_common
 * @ingroup     drivers_periph_gpio
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation for ATmega family
 *
 * @author      René Herthel <rene-herthel@outlook.de>
 * @author      Francisco Acosta <francisco.acosta@inria.fr>
 * @author      Laurent Navet <laurent.navet@gmail.com>
 *
 * @}
 */


#include <stdio.h>

#include <avr/interrupt.h>

#include "cpu.h"
#include "periph/gpio.h"

#include "periph_conf.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define GPIO_PORT_BASE       ((uint16_t)&PORTA)
#define GPIO_PORT_OFFSET   (0x20)

//#define GPIO_OFFSET_PIN_PIN     (0x03)

#define GPIO_PORT_Q_OFFSET      (0xA0)

/* TODO find a good way to implement this */
/*
 * @brief     Define GPIO interruptions for an specific atxmega CPU
 *
 *            Most general way to get appropriate external interrupt count
 *

 *
 */

/*
 *            #if defined(PORTA_INT0_vect_num)
 *                 #define PORTA_INT_NUMOF   (2U)
 *            #else
 *                #define PORTA_INT_NUMOF    (0U)
 *            #endif
 */
//#define IS_DEFINED(x) IS_DEFINED2(x)
//#define IS_DEFINED2(x) (#x[0] == 0 || (#x[0] >= '1' && #x[0] <= '9'))
//#define CHECK_PORT_INT_NUM( x )  ( ( IS_DEFINED(x ## _INT0_vect_num) ) ? (2U) : (0U) )
//
//
//#define PORTA_INT_NUMOF CHECK_PORT_INT_NUM( PORTA )
//#define PORTB_INT_NUMOF CHECK_PORT_INT_NUM( PORTB )
//#define PORTC_INT_NUMOF CHECK_PORT_INT_NUM( PORTC )
//#define PORTD_INT_NUMOF CHECK_PORT_INT_NUM( PORTD )
//#define PORTE_INT_NUMOF CHECK_PORT_INT_NUM( PORTE )
//#define PORTF_INT_NUMOF CHECK_PORT_INT_NUM( PORTF )
//#define PORTH_INT_NUMOF CHECK_PORT_INT_NUM( PORTH )
//#define PORTJ_INT_NUMOF CHECK_PORT_INT_NUM( PORTJ )
//#define PORTK_INT_NUMOF CHECK_PORT_INT_NUM( PORTK )
//#define PORTQ_INT_NUMOF CHECK_PORT_INT_NUM( PORTQ )
//#define PORTR_INT_NUMOF CHECK_PORT_INT_NUM( PORTR )
//
//
//#define GPIO_EXT_INT_NUMOF      (   PORTA_INT_NUMOF +  PORTB_INT_NUMOF
//                                  + PORTC_INT_NUMOF +  PORTD_INT_NUMOF
//                                  + PORTD_INT_NUMOF +  PORTE_INT_NUMOF
//                                  + PORTF_INT_NUMOF +  PORTH_INT_NUMOF
//                                  + PORTJ_INT_NUMOF +  PORTK_INT_NUMOF
//                                  + PORTQ_INT_NUMOF +  PORTR_INT_NUMOF
//                                 )
//


#define GPIO_EXT_INT_NUMOF (14U) /* atxmega256a3u */

static gpio_isr_ctx_t config[GPIO_EXT_INT_NUMOF];

/**
 * @brief     Extract the pin number of the given pin
 */
static inline uint8_t _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}

/**
 * @brief     Extract the port number of the given pin
 */
static inline uint8_t _port_num(gpio_t pin)
{
    return (pin >> 4) & 0x0f;
}

/**
 * @brief     Generate the PORTx address of the give pin.
 */
static inline uint16_t _port_addr(gpio_t pin)
{
    uint8_t port_num = _port_num(pin);
    uint16_t port_addr = GPIO_PORT_BASE;

    port_addr += port_num * GPIO_PORT_OFFSET;

    /* PORT_Q and PORT_R have an offset, PORT_K = 8 last port with no offset */
    if (port_num > 8 ) {
        port_addr += GPIO_PORT_Q_OFFSET;
    }

    return port_addr;
}


int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    DEBUG("gpio_init pin = %#02x mode = %#02x\n", pin, mode );

    uint8_t pin_num = _pin_num(pin);
    uint16_t port_addr = _port_addr(pin);

    PORT_t* port = (PORT_t *)_port_addr(pin);

    /*Set input and low or output*/
    switch ( (mode&0xf0) ) {
        case GPIO_IN:
            port->OUTCLR = (1 << pin_num );
            port->DIRCLR = (1 << pin_num );
//            _SFR_MEM8( port_addr +2) = (1 << pin_num );
//            _SFR_MEM8( port_addr +6) = (1 << pin_num );
            break;
        case GPIO_OUT:
            port->DIRSET = (1 << pin_num );
//            _SFR_MEM8( port_addr +1) = (1 << pin_num );
            break;
        default:
            return -1;
    }

    /*Set Output/pull configuration */
    // port->INTCTRL = (mode&0x0f)<<3;
    _SFR_MEM8( port_addr + (1 << pin_num ) + 0x10) = (mode&0x0f)<<3;

    return 0;
}


/* TODO improve for selecting INT0/1 , select right config[], set INTMASK */
/* TODO improve for selecting priority level */
int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg)
{
    DEBUG("gpio_init_int pin = %#02x mode = %#02x flank = %#02x\n", pin, mode, flank );
    uint8_t pin_num = _pin_num(pin);
    uint8_t port_num = _port_num(pin);
    PORT_t* port = (PORT_t *)_port_addr(pin);

    /* set callback */
    config[port_num].cb = cb;
    config[port_num].arg = arg;

    /* Disable Interrupts */
    cli();

    gpio_init(pin, mode);

    /* Enable Interrupt in INT0MASK for given pin, ONLY*/
    port->INT0MASK = (1 << pin_num) ;
    //_SFR_MEM8( port_addr + 0x0A) |= (1 << pin_num);

    /*Set Input/sense configuration configuration */
    *(&(port->PIN0CTRL) + (1 << pin_num) ) = (flank&0x0f);
    //_SFR_MEM8( port_addr + (1 << pin_num) + 0x10) = (flank&0x0f);

    /*Select level for Interrupt INT0MASK */
    port->INTCTRL =  PORT_INT0LVL_MED_gc;
    // _SFR_MEM8( port_addr + 0x09) |=  PORT_INT0LVL_MED_gc;


    /* Enable global Multi-level Interrupt Controller */
    PMIC.CTRL|= PMIC_MEDLVLEN_bm;

    /* set global interrupt flag, enable interrupts*/
    sei();

    return 0;
}

/* TODO Improve for INT0/1 */
void gpio_irq_enable(gpio_t pin)
{
    /* RF_IRQ    - PD2 */
    DEBUG("gpio_irq_enable pin = %#02x \n", pin );

    /* Enable Interrupt in INT0MASK for given pin*/
    ((PORT_t *)_port_addr(pin))->INT0MASK |= (1 << _pin_num(pin) );
//    _SFR_MEM8( _port_addr(pin) + 0x0A) |= (1 << _pin_num(pin) );
}

/* TODO Improve for INT0/1 */
void gpio_irq_disable(gpio_t pin)
{
    DEBUG("gpio_irq_enable pin = %#02x \n", pin );

    /* Disable Interrupt in INT0MASK for given pin*/
    ((PORT_t *)_port_addr(pin))->INT0MASK &= ~(1 << _pin_num(pin) );
//    _SFR_MEM8( _port_addr(pin) + 0x0A) &= ~(1 << _pin_num(pin) );
}

int gpio_read(gpio_t pin)
{
    DEBUG("gpio_read pin = %#02x \n", pin );
    return ( ((PORT_t *)_port_addr(pin))->IN & (1 << _pin_num(pin)));
//    return ( _SFR_MEM8(_port_addr(pin) + 0x08) & (1 << _pin_num(pin)) );
}


void gpio_set(gpio_t pin)
{
    DEBUG("gpio_set pin = %#02x \n", pin );
    ((PORT_t *)_port_addr(pin))->OUTSET = (1 << _pin_num(pin));
//    _SFR_MEM8(_port_addr(pin) + 0x05) = (1 << _pin_num(pin));
}

void gpio_clear(gpio_t pin)
{
    DEBUG("gpio_clear pin = %#02x \n", pin );
    ((PORT_t *)_port_addr(pin))->OUTCLR = (1 << _pin_num(pin));
//    _SFR_MEM8(_port_addr(pin) + 0x06) = (1 << _pin_num(pin));
}


static inline void irq_handler(uint8_t port_num)
{
    DEBUG("irq_handler port = 0x%02x \n", port_num );

    __enter_isr();
    config[port_num].cb(config[port_num].arg);
    __exit_isr();
}

/*TODO improve for INT0/1 */
/* AVR_CONTEXT_SWAP_INTERRUPT_VECT  is PORTA_INT0_vect*/
//#if defined(PORTA_INT0_vect)
//ISR(PORTA_INT0_vect, ISR_BLOCK)
//{
//    irq_handler(0);
//}
//#endif
#if defined(PORTA_INT1_vect)
ISR(PORTA_INT1_vect, ISR_BLOCK)
{
    irq_handler(0);
}
#endif

#if defined(PORTB_INT0_vect)
ISR(PORTB_INT0_vect, ISR_BLOCK)
{
    irq_handler(1);
}
#endif
#if defined(PORTB_INT1_vect)
ISR(PORTB_INT1_vect, ISR_BLOCK)
{
    irq_handler(1);
}
#endif

#if defined(PORTC_INT0_vect)
ISR(PORTC_INT0_vect, ISR_BLOCK)
{
    irq_handler(2);
}
#endif
#if defined(PORTC_INT1_vect)
ISR(PORTC_INT1_vect, ISR_BLOCK)
{
    irq_handler(2);
}
#endif

#if defined(PORTD_INT0_vect)
ISR(PORTD_INT0_vect, ISR_BLOCK)
{
    irq_handler(3);
}
#endif
#if defined(PORTD_INT1_vect)
ISR(PORTD_INT1_vect, ISR_BLOCK)
{
    irq_handler(3);
}
#endif

#if defined(PORTE_INT0_vect)
ISR(PORTE_INT0_vect, ISR_BLOCK)
{
    irq_handler(4);
}
#endif
#if defined(PORTE_INT1_vect)
ISR(PORTE_INT1_vect, ISR_BLOCK)
{
    irq_handler(4);
}
#endif

#if defined(PORTF_INT0_vect)
ISR(PORTF_INT0_vect, ISR_BLOCK)
{
    irq_handler(5);
}
#endif
#if defined(PORTF_INT1_vect)
ISR(PORTF_INT1_vect, ISR_BLOCK)
{
    irq_handler(5);
}
#endif

#if defined(PORTH_INT0_vect)
ISR(PORTH_INT0_vect, ISR_BLOCK)
{
    irq_handler(12);
}
#endif
#if defined(PORTH_INT1_vect)
ISR(PORTH_INT1_vect, ISR_BLOCK)
{
    irq_handler(13);
}
#endif

#if defined(PORTJ_INT0_vect)
ISR(PORTJ_INT0_vect, ISR_BLOCK)
{
    irq_handler(14);
}
#endif
#if defined(PORTLJ_INT1_vect)
ISR(PORTJ_INT1_vect, ISR_BLOCK)
{
    irq_handler(15);
}
#endif

#if defined(PORTK_INT0_vect)
ISR(PORTK_INT0_vect, ISR_BLOCK)
{
    irq_handler(16);
}
#endif
#if defined(PORTLK_INT1_vect)
ISR(PORTK_INT1_vect, ISR_BLOCK)
{
    irq_handler(17);
}
#endif

#if defined(PORTQ_INT0_vect)
ISR(PORTQ_INT0_vect, ISR_BLOCK)
{
    irq_handler(18);
}
#endif
#if defined(PORTLQ_INT1_vect)
ISR(PORTQ_INT1_vect, ISR_BLOCK)
{
    irq_handler(19);
}
#endif

#if defined(PORTR_INT0_vect)
ISR(PORTR_INT0_vect, ISR_BLOCK)
{
    irq_handler(10);
}
#endif
#if defined(PORTR_INT1_vect)
ISR(PORTR_INT1_vect, ISR_BLOCK)
{
    irq_handler(10);
}
#endif

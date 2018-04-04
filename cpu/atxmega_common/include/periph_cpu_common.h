/*
 * Copyright (C) 2018 Josua Arndt
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup         cpu_atmega_common
 * @{
 *
 * @file
 * @brief           CPU specific definitions for internal peripheral handling
 *
 * @author          Josua Arndt <jarndt@ias.rwth-aachen.de>
 */

#ifndef PERIPH_CPU_COMMON_H
#define PERIPH_CPU_COMMON_H

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Pin mode select macro
 *
 * The Pin mode is determined by bit 3-5 (OPC) of the PINnCTRL Register
 * and the Data Direction register (DIR)
 */
#define PIN_MODE_SEL(DIR, OPC )          (DIR << 4 | OPC)
/**
 * @brief   Available pin modes
 *
 * Generally, a pin can be configured to be input or output. In output mode, a
 * pin can further be put into push-pull or open drain configuration. Though
 * this is supported by most platforms, this is not always the case, so driver
 * implementations may return an error code if a mode is not supported.
 */
#define HAVE_GPIO_MODE_T
typedef enum GPIO_MODE {
    GPIO_IN                 = PIN_MODE_SEL(0, 0),   /**< configure as input without pull resistor (TOTEM)*/
    GPIO_IN_BSKPR           = PIN_MODE_SEL(0, 1),   /**< configure as input with push-pull mode (BUSKEEPER) */
    GPIO_IN_PD              = PIN_MODE_SEL(0, 2),   /**< configure as input with pull-down resistor */
    GPIO_IN_PU              = PIN_MODE_SEL(0, 3),   /**< configure as input with pull-up resistor */

    GPIO_IN_WRD_OR          = PIN_MODE_SEL(0, 4),   /**< configure as input with wired OR  */
    GPIO_IN_WRD_AND         = PIN_MODE_SEL(0, 5),   /**< configure as input with wired AND */
    GPIO_IN_WRD_OR_PULL     = PIN_MODE_SEL(0, 6),   /**< configure as input with with wired OR and pull-down resistor */
    GPIO_IN_WRD_AND_PULL    = PIN_MODE_SEL(0, 7),   /**< configure as input with with wired AND and pull-up resistor */

    GPIO_OUT                = PIN_MODE_SEL(1, 1),   /**< configure as output with push-pull mode (BUSKEEPER) */
    GPIO_OUT_PD             = PIN_MODE_SEL(1, 2),   /**< configure as output with push-pull mode and Pull-Down */
    GPIO_OUT_PU             = PIN_MODE_SEL(1, 3),   /**< configure as output with push-pull mode and Pull-Up */
    GPIO_OUT_WRD_OR_PULL    = PIN_MODE_SEL(1, 6),   /**< configure as output with with wired OR and pull-down resistor */
    GPIO_OUT_WRD_AND_PULL   = PIN_MODE_SEL(1, 7),   /**< configure as output with with wired AND and pull-up resistor */
} gpio_mode_t;

/**
 * @brief   Definition of possible active flanks for external interrupt mode
 */
#define HAVE_GPIO_FLANK_T
typedef enum {
    GPIO_BOTH       = 0,    /**< emit interrupt on both flanks */
    GPIO_RISING     = 1,    /**< emit interrupt on rising flank */
    GPIO_FALLING    = 2,    /**< emit interrupt on falling flank */
    GPIO_LOW_LEVEL  = 3,    /**< emit interrupt on low level */
    GPIO_IN_DISS    = 7,    /**< PORTA - PORTF buffer disable for better performance with analog functionality*/
} gpio_flank_t;

/**
 * @name    Power management configuration
 * @{
 */
#define PROVIDES_PM_SET_LOWEST
#define PROVIDES_PM_OFF
/** @} */

///**
// * @brief   Use some common SPI functions
// * @{
// */
//#define PERIPH_SPI_NEEDS_INIT_CS
//#define PERIPH_SPI_NEEDS_TRANSFER_BYTE
//#define PERIPH_SPI_NEEDS_TRANSFER_REG
//#define PERIPH_SPI_NEEDS_TRANSFER_REGS
///** @} */

/**
 * @brief   Define default UART type identifier
 * @{
 */

//typedef struct XMEGA_UART_T {
//    USART_t dev;           /* USART_t UART device */
//    register8_t pr;        /* Power Reduction Register */
//    uint16_t* rx_isr;       /* RX Callback handler */
//} xmega_uart_t;


/** @} */


/**
 * @brief   xmega type for SPI devices
 * @{
 */
//#define HAVE_SPI_T
//#define spi_t SPI_t;
// TODO it
//In file included from /media/sf_Linux_Work/RIOT/cpu/atxmega_common/periph/spi.c:29:0:
///media/sf_Linux_Work/RIOT/drivers/include/periph/spi.h:188:21: error: unknown type name 'bus'
// void spi_init(spi_t bus);

/** @} */

/**
 * @brief   SPI mode select macro
 *
 * The polarity is determined by bit 3 in the configuration register, the phase
 * by bit 2.
 */
#define SPI_MODE_SEL(pol, pha)          ((pol << 3) | (pha << 2))

/**
 * @brief   Override the SPI mode values
 *
 * As the mode is set in bit 3 and 2 of the configuration register, we put the
 * correct configuration there
 * @{
 */
#define HAVE_SPI_MODE_T
typedef enum {
    SPI_MODE_0  = SPI_MODE_SEL(0, 0),   /**< mode 0 */
    SPI_MODE_1  = SPI_MODE_SEL(0, 1),   /**< mode 1 */
    SPI_MODE_2  = SPI_MODE_SEL(1, 0),   /**< mode 2 */
    SPI_MODE_3  = SPI_MODE_SEL(1, 1)    /**< mode 3 */
} spi_mode_t;
/** @} */

/**
 * @brief   SPI speed selection macro
 *
 * We encode the speed in bits 7, 1, and 0, where bit0 and bit1 hold the
 * prescaler bits, while bit7 holds the CLK2X bit.
 */
#define SPI_CLK_SEL(CLK2X, pr1, pr0)    ((CLK2X << 7) | (pr1 << 1) | pr0)
/**
 * @brief   Override SPI speed values
 *
 * We assume a master clock speed of 32MHz here.
 * @{
 */
#define HAVE_SPI_CLK_T
typedef enum {
    SPI_CLK_250KHZ          = SPI_CLK_SEL(0, 0, 0), /**< 32/128 -> 250KHz */
    SPI_CLK_500KHZ          = SPI_CLK_SEL(0, 0, 1), /**< 32/64  -> 500kHz */
    SPI_CLK_2MHZ            = SPI_CLK_SEL(0, 1, 0), /**< 32/16  -> 2MHz */
    SPI_CLK_8MHZ            = SPI_CLK_SEL(0, 1, 1), /**< 32/4   -> 8MHz */
    SPI_CLK_16MHZ           = SPI_CLK_SEL(1, 0, 0), /**< 32/2   -> 16MHz */
    SPI_CLK_4MHZ            = SPI_CLK_SEL(1, 0, 0), /**< 32/8   -> 4MHz */
    SPI_CLK_1MHZ            = SPI_CLK_SEL(1, 0, 0), /**< 32/32   -> 1MHz */
    SPI_CLK_500MHZ_CLK2X    = SPI_CLK_SEL(1, 0, 0)  /**< 32/64   -> 500kHz */
} spi_clk_t;
/** @} */

#define spi_intlvl_t    SPI_INTLVL_t


#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CPU_COMMON_H */
/** @} */

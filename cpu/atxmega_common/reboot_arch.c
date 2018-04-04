/*
 * Copyright (C) 2018 Josua Arndt
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
 * @brief       Implementation of the kernels reboot interface
 *
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @}
 */

#include <avr/io.h>

#include "cpu.h"

#include "debug.h"

void reboot(void)
{
    DEBUG("Reboot Software Reset\n" );

    /* XMEGA AU [MANUAL] p. 116 CTRL -Control register
     * page 13 3.12.1 Sequence for write operation to protected I/O registers
     * page 15 3.14.1 CCP – Configuration Change Protection register
     */

    /* Disable CCP for Protected IO registerand set new value*/
    _PROTECTED_WRITE(RST_CTRL, RST_SWRST_bm);
    while (1) {}
}

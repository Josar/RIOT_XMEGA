/*
 * Copyright (C) 2018 Josua Arndt
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  atmega256rfr2
 * @{
 *
 * @file
 * @brief       Low-level CPUID driver implementation
 *
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 *
 * @}
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "periph/cpuid.h"

#ifdef MODULE_AT86RF2XX
	#include "at86rf2xx_registers.h"
#endif

#define ENABLE_DEBUG 0
#include "debug.h"


/**
 * @brief   Starting offset of CPU_ID
 *
 *  CPIUD is taken from MCU Control Register and Signature.
 *  CPUID:  1e  42  98  06  ff  1f  0b  01
 *  XMEGA AU [MANUAL] p.44
 */
#define AT86RF2XX_MAN_ID_0 0x1F
#define AT86RF2XX_VERSION 0x01
void cpuid_get(void *id)
{

	uint8_t addr[CPUID_LEN] = {
	        //SIGNATURE_0,  /* 0x1E Atmel JEDEC manufacturer ID */
	        //SIGNATURE_1,  /* 0x98 */
	        //SIGNATURE_2,  /* 0x42 */
	        MCU_DEVID0,         /* 0x1E Atmel JEDEC manufacturer ID */
	        MCU_DEVID2,         /* 0x98 Device number */
	        MCU_DEVID1,         /* 0x42 Flash size indicator */
	        MCU_REVID,          /* Device revision, 0 = A ... */
	        MCU_JTAGUID,        /* Identify devices with identical device IDs */
#ifdef MODULE_AT86RF2XX
	        AT86RF2XX_MAN_ID_0, /* 0x1F Atmel JEDEC manufacturer ID */
	        AT86RF2XX_PARTNUM,  /* transceiver part number */
	        AT86RF2XX_VERSION,  /* transceiver revision */
#else
	        0,0,0
#endif
	};

#if ENABLE_DEBUG
	DEBUG("CPUID: " );
	for(uint8_t i=0; i<CPUID_LEN; i++)
	{
	    DEBUG(" %02x ", addr[i] );
	}
	DEBUG("\n" );
#endif

    memcpy( id , addr, CPUID_LEN);
}

/**************************************************************************************************

  Copyright 2010 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS�
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board definition file.
 *   Target : Texas Instruments DK-LM3S9B96
 *            Stellaris Development Kit with EM Adapter
 *   Radios : CC2500
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_BOARD_DEFS_H
#define MRFI_BOARD_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "stdint.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"



/* ------------------------------------------------------------------------------------------------
 *                                Global Storing Last SPI Read Result
 * ------------------------------------------------------------------------------------------------
 */

/*
 * The common SPI implementation in the radio code assumes a non-FIFOed SPI
 * controller so we need to model this in the Stellaris implementation.  To
 * do this, every byte written is followed by a SPI read into this variable
 * and the value in this variable is returned any time a SPI read is requested.
 * This is a bit unpleasant but allows us to pick up support for all the
 * existing radio families without having to product Stellaris-specific
 * code for each one.
 */
extern unsigned char g_ucSPIReadVal;

/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                     GDO0 Pin Configuration (on PH0 for MOD1, PG0 for MOD2)
 * ------------------------------------------------------------------------------------------------
 */
#define GDO0_PIN						GPIO_PIN_6
#define GDO2_PIN						GPIO_PIN_7
#define GDOx_BASE 						GPIO_PORTA_BASE
#define CONFIG_GDO0_PIN_AS_INPUT()   	GPIOPinTypeGPIOInput(GDOx_BASE, GDO0_PIN)
#define GDO0_PIN_IS_HIGH()           	GPIOPinRead(GDOx_BASE,GDO0_PIN)
#define GDO2_PIN_IS_HIGH()           	GPIOPinRead(GDOx_BASE,GDO2_PIN)

#define GDOx_INT_VECTOR              	INT_GPIOA
#define ENABLE_GDOx_INT()            	GPIOPinIntEnable(GDOx_BASE, GDO0_PIN|GDO2_PIN)
#define DISABLE_GDOx_INT()           	GPIOPinIntDisable(GDOx_BASE, GDO0_PIN|GDO2_PIN)

#define ENABLE_GDO0_INT()            	GPIOPinIntEnable(GDOx_BASE, GDO0_PIN)
#define DISABLE_GDO0_INT()           	GPIOPinIntDisable(GDOx_BASE, GDO0_PIN)

#define ENABLE_GDO2_INT()            	GPIOPinIntEnable(GDOx_BASE, GDO2_PIN)
#define DISABLE_GDO2_INT()           	GPIOPinIntDisable(GDOx_BASE, GDO2_PIN)

//#define GDO0_INT_IS_ENABLED()        	( HWREG(GDOx_BASE + GPIO_O_IM) & GDO0_PIN )
//#define GDO2_INT_IS_ENABLED()        	( HWREG(GDOx_BASE + GPIO_O_IM) & GDO2_PIN )

#define CLEAR_GDOx_INT_FLAG()        	GPIOPinIntClear(GDOx_BASE, GDO0_PIN | GDO2_PIN)
#define GDO0_INT_FLAG_IS_SET()       	(GPIOPinIntStatus(GDOx_BASE, false) & GDO0_PIN)
#define GDO2_INT_FLAG_IS_SET()       	(GPIOPinIntStatus(GDOx_BASE, false) & GDO2_PIN)

#define CONFIG_GDO0_RISING_EDGE_INT() 	GPIOIntTypeSet(GDOx_BASE, GDO0_PIN, GPIO_RISING_EDGE)
#define CONFIG_GDO0_FALLING_EDGE_INT() 	GPIOIntTypeSet(GDOx_BASE, GDO0_PIN, GPIO_FALLING_EDGE

#define CONFIG_GDO2_RISING_EDGE_INT() 	GPIOIntTypeSet(GDOx_BASE, GDO2_PIN, GPIO_RISING_EDGE)
#define CONFIG_GDO2_FALLING_EDGE_INT() 	GPIOIntTypeSet(GDOx_BASE, GDO2_PIN, GPIO_FALLING_EDGE)



/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* Chip select*/
#define SPI_BASE                      	   	GPIO_PORTA_BASE
#define SPI_CSN_PIN                       	GPIO_PIN_3


/* Note - for safely, when we drive CSn low, we ensure that the inactive
 * module's CSn line is driven high.  This prevents both modules being
 * selected at once.
 */
#define SPI_CONFIG_CSN_PIN_AS_OUTPUT()  	GPIOPinTypeGPIOOutput(SPI_BASE,SPI_CSN_PIN)
#define SPI_DRIVE_CSN_HIGH()            	GPIOPinWrite(SPI_BASE,SPI_CSN_PIN, SPI_CSN_PIN)

#define SPI_DRIVE_CSN_LOW()            GPIOPinWrite(SPI_BASE, SPI_CSN_PIN, 0)
#define SPI_CSN_IS_HIGH()              GPIOPinRead(SPI_BASE, SPI_CSN_PIN)

/* SCLK Pin Configuration */
#define SPI_SCLK_PIN		            	GPIO_PIN_2
#define SPI_CONFIG_SCLK_PIN_AS_OUTPUT()  	GPIOPinTypeSSI(SPI_BASE,SPI_SCLK_PIN )
//#define MRFI_SPI_DRIVE_SCLK_HIGH()            /* Not used. */
//#define MRFI_SPI_DRIVE_SCLK_LOW()             /* Not used. */

/* MOSI Pin Configuration*/
#define SPI_MOSI_PIN						GPIO_PIN_5
#define SPI_CONFIG_MOSI_PIN_AS_OUTPUT()    	GPIOPinTypeSSI(SPI_BASE, SPI_MOSI_PIN)
//#define MRFI_SPI_DRIVE_SI_HIGH()              /* Not used. */
//#define MRFI_SPI_DRIVE_SI_LOW()               /* Not used. */

/* MISO Pin Configuration (PF4) */
#define SPI_MISO_PIN              			GPIO_PIN_4
#define SPI_CONFIG_MISO_PIN_AS_INPUT()     	GPIOPinTypeSSI(SPI_BASE, SPI_MISO_PIN)
#define SPI_MISO_IS_HIGH()                  GPIOPinRead(SPI_BASE, SPI_MISO_PIN)

/* SPI Port Configuration */
#define MRFI_SPI_CONFIG_PORT()                /* Not used. */



/* SPI critical section macros */

typedef unsigned char  mrfiSpiIState_t;
#define SPI_ENTER_CRITICAL_SECTION(x)    ENTER_CRITICAL_SECTION(x)
#define SPI_EXIT_CRITICAL_SECTION(x)     EXIT_CRITICAL_SECTION(x)


/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  1 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */

/* Initialization macro  Note that configuration of individual pins is done
 * elsewhere. This macro is responsible for peripheral-level configuration
 * and pin muxing.  It also enables the SSI peripheral and flushes the receive
 * FIFO just in case it has anything in it. Additionally, since there's no
 * other obvious place to put it, we enable the interrupt for the GPIO that
 * the radio uses to signal reception of data here too.
 */
#define SPI_RATE 1000000



#define SPI_IS_INITIALIZED()  (HWREG(SSI0_BASE + SSI_O_CR1) & SSI_CR1_SSE)


#endif

/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

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

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
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
 *   SPI interface code for Radio FamRadios: CC1100, CC1101, CC2500
 *   SPI interface code.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_SPI_H
#define MRFI_SPI_H

/* ------------------------------------------------------------------------------------------------
 *                                         Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "stdint.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"



/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */

/* configuration registers */
#define IOCFG2      0x00      /*  IOCFG2   - GDO2 output pin configuration  */
#define IOCFG1      0x01      /*  IOCFG1   - GDO1 output pin configuration  */
#define IOCFG0      0x02      /*  IOCFG1   - GDO0 output pin configuration  */
#define FIFOTHR     0x03      /*  FIFOTHR  - RX FIFO and TX FIFO thresholds */
#define SYNC1       0x04      /*  SYNC1    - Sync word, high byte */
#define SYNC0       0x05      /*  SYNC0    - Sync word, low byte */
#define PKTLEN      0x06      /*  PKTLEN   - Packet length */
#define PKTCTRL1    0x07      /*  PKTCTRL1 - Packet automation control */
#define PKTCTRL0    0x08      /*  PKTCTRL0 - Packet automation control */
#define ADDR        0x09      /*  ADDR     - Device address */
#define CHANNR      0x0A      /*  CHANNR   - Channel number */
#define FSCTRL1     0x0B      /*  FSCTRL1  - Frequency synthesizer control */
#define FSCTRL0     0x0C      /*  FSCTRL0  - Frequency synthesizer control */
#define FREQ2       0x0D      /*  FREQ2    - Frequency control word, high byte */
#define FREQ1       0x0E      /*  FREQ1    - Frequency control word, middle byte */
#define FREQ0       0x0F      /*  FREQ0    - Frequency control word, low byte */
#define MDMCFG4     0x10      /*  MDMCFG4  - Modem configuration */
#define MDMCFG3     0x11      /*  MDMCFG3  - Modem configuration */
#define MDMCFG2     0x12      /*  MDMCFG2  - Modem configuration */
#define MDMCFG1     0x13      /*  MDMCFG1  - Modem configuration */
#define MDMCFG0     0x14      /*  MDMCFG0  - Modem configuration */
#define DEVIATN     0x15      /*  DEVIATN  - Modem deviation setting */
#define MCSM2       0x16      /*  MCSM2    - Main Radio Control State Machine configuration */
#define MCSM1       0x17      /*  MCSM1    - Main Radio Control State Machine configuration */
#define MCSM0       0x18      /*  MCSM0    - Main Radio Control State Machine configuration */
#define FOCCFG      0x19      /*  FOCCFG   - Frequency Offset Compensation configuration */
#define BSCFG       0x1A      /*  BSCFG    - Bit Synchronization configuration */
#define AGCCTRL2    0x1B      /*  AGCCTRL2 - AGC control */
#define AGCCTRL1    0x1C      /*  AGCCTRL1 - AGC control */
#define AGCCTRL0    0x1D      /*  AGCCTRL0 - AGC control */
#define WOREVT1     0x1E      /*  WOREVT1  - High byte Event0 timeout */
#define WOREVT0     0x1F      /*  WOREVT0  - Low byte Event0 timeout */
#define WORCTRL     0x20      /*  WORCTRL  - Wake On Radio control */
#define FREND1      0x21      /*  FREND1   - Front end RX configuration */
#define FREND0      0x22      /*  FREDN0   - Front end TX configuration */
#define FSCAL3      0x23      /*  FSCAL3   - Frequency synthesizer calibration */
#define FSCAL2      0x24      /*  FSCAL2   - Frequency synthesizer calibration */
#define FSCAL1      0x25      /*  FSCAL1   - Frequency synthesizer calibration */
#define FSCAL0      0x26      /*  FSCAL0   - Frequency synthesizer calibration */
#define RCCTRL1     0x27      /*  RCCTRL1  - RC oscillator configuration */
#define RCCTRL0     0x28      /*  RCCTRL0  - RC oscillator configuration */
#define FSTEST      0x29      /*  FSTEST   - Frequency synthesizer calibration control */
#define PTEST       0x2A      /*  PTEST    - Production test */
#define AGCTEST     0x2B      /*  AGCTEST  - AGC test */
#define TEST2       0x2C      /*  TEST2    - Various test settings */
#define TEST1       0x2D      /*  TEST1    - Various test settings */
#define TEST0       0x2E      /*  TEST0    - Various test settings */

/* status registers */
#define PARTNUM     0x30      /*  PARTNUM    - Chip ID */
#define VERSION     0x31      /*  VERSION    - Chip ID */
#define FREQEST     0x32      /*  FREQEST    – Frequency Offset Estimate from demodulator */
#define LQI         0x33      /*  LQI        – Demodulator estimate for Link Quality */
#define RSSI        0x34      /*  RSSI       – Received signal strength indication */
#define MARCSTATE   0x35      /*  MARCSTATE  – Main Radio Control State Machine state */
#define WORTIME1    0x36      /*  WORTIME1   – High byte of WOR time */
#define WORTIME0    0x37      /*  WORTIME0   – Low byte of WOR time */
#define PKTSTATUS   0x38      /*  PKTSTATUS  – Current GDOx status and packet status */
#define VCO_VC_DAC  0x39      /*  VCO_VC_DAC – Current setting from PLL calibration module */
#define TXBYTES     0x3A      /*  TXBYTES    – Underflow and number of bytes */
#define RXBYTES     0x3B      /*  RXBYTES    – Overflow and number of bytes */

/* burst write registers */
#define PA_TABLE0   0x3E      /*  PA_TABLE0 - PA control settings table */
#define TXFIFO      0x3F      /*  TXFIFO  - Transmit FIFO */
#define RXFIFO      0x3F      /*  RXFIFO  - Receive FIFO */
// Definitions for burst/single access to registers
#define CC1100_WRITE_BURST      0x40
#define CC1100_READ_SINGLE      0x80
#define CC1100_READ_BURST       0xC0

/* command strobe registers */
#define RF_SRES        0x30      /*  SRES    - Reset chip. */
#define RF_SFSTXON     0x31      /*  SFSTXON - Enable and calibrate frequency synthesizer. */
#define RF_SXOFF       0x32      /*  SXOFF   - Turn off crystal oscillator. */
#define RF_SCAL        0x33      /*  SCAL    - Calibrate frequency synthesizer and turn it off. */
#define RF_SRX         0x34      /*  SRX     - Enable RX. Perform calibration if enabled. */
#define RF_STX         0x35      /*  STX     - Enable TX. If in RX state, only enable TX if CCA passes. */
#define RF_SIDLE       0x36      /*  SIDLE   - Exit RX / TX, turn off frequency synthesizer. */
#define RF_SRSVD       0x37      /*  SRVSD   - Reserved.  Do not use. */
#define RF_SWOR        0x38      /*  SWOR    - Start automatic RX polling sequence (Wake-on-Radio) */
#define RF_SPWD        0x39      /*  SPWD    - Enter power down mode when CSn goes high. */
#define RF_SFRX        0x3A      /*  SFRX    - Flush the RX FIFO buffer. */
#define RF_SFTX        0x3B      /*  SFTX    - Flush the TX FIFO buffer. */
#define RF_SWORRST     0x3C      /*  SWORRST - Reset real time clock. */
#define RF_SNOP        0x3D      /*  SNOP    - No operation. Returns status byte. */


/*
 *  GDO functionality
 */


#define CC1101_GDO_LNA_PD         0x1c  /* low when receive is active, low during sleep */
#define CC1101_GDO_RXFHIGH             0x00
#define CC1101_GDO_RXFHIGH_PKTDONE     0x01
#define CC1101_GDO_TXFHIGH             0x02
#define CC1101_GDO_TXFFULL             0x03
#define CC1101_GDO_RXFOVER             0x04
#define CC1101_GDO_TXFUNDER            0x05
#define CC1101_GDO_SYNCDETECT          0x06
#define CC1101_GDO_PKT_CRCOK           0x07
#define CC1101_GDO_PQT_REACHED         0x08
#define CC1101_GDO_CCAOK               0x09
#define CC1101_GDO_PLL_LOCK            0x0A
#define CC1101_GDO_SCLK                0x0B
#define CC1101_GDO_SSDO                0x0C
#define CC1101_GDO_ASDO                0x0D
#define CC1101_GDO_CSOK                0x0E
#define CC1101_GDO_CRCOK               0x0F
#define CC1101_GDO_RX_HARD_DATA1       0x16
#define CC1101_GDO_RX_HARD_DATA0       0x17
#define CC1101_GDO_PA_PD               0x1B
#define CC1101_GDO_LNA_PD              0x1C
#define CC1101_GDO_RX_SYMBOL_TICK      0x1D
#define CC1101_GDO_WOR_EVNT0           0x24
#define CC1101_GDO_WOR_EVNT1           0x25
#define CC1101_GDO_CLK_32k             0x27
#define CC1101_GDO_CHIP_RDYn           0x29
#define CC1101_GDO_XOSC_STABLE         0x2B
#define CC1101_GDO_GDO0_Z_EN_N         0x2D
#define CC1101_GDO_HIZ                 0x2E
#define CC1101_GDO_HW0                 0x2F
#define CC1101_GDO_CLK_XOSC_D_1        0x30
#define CC1101_GDO_CLK_XOSC_D_1_5      0x31
#define CC1101_GDO_CLK_XOSC_D_2        0x32
#define CC1101_GDO_CLK_XOSC_D_3        0x33
#define CC1101_GDO_CLK_XOSC_D_4        0x34
#define CC1101_GDO_CLK_XOSC_D_6        0x35
#define CC1101_GDO_CLK_XOSC_D_8        0x36
#define CC1101_GDO_CLK_XOSC_D_12       0x37
#define CC1101_GDO_CLK_XOSC_D_16       0x38
#define CC1101_GDO_CLK_XOSC_D_24       0x39
#define CC1101_GDO_CLK_XOSC_D_32       0x3A
#define CC1101_GDO_CLK_XOSC_D_48       0x3B
#define CC1101_GDO_CLK_XOSC_D_64       0x3C
#define CC1101_GDO_CLK_XOSC_D_96       0x3D
#define CC1101_GDO_CLK_XOSC_D_128      0x3E
#define CC1101_GDO_CLK_XOSC_D_192      0x3F


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void SpiInit(void);
void radioSelect();
void radioDeselect();
uint8_t SendByte(uint8_t data);


uint8_t Strobe(uint8_t addr);
void SpiCC1101Reset();
uint8_t ReadSingleReg(uint8_t addr);
void ReadBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len);
void WriteSingleReg(uint8_t addr, uint8_t value);
void WriteBurstReg(uint8_t regAddr, uint8_t* buffer, uint8_t len);

void SpiWriteTxFifo(uint8_t * pWriteData, uint8_t len);
void SpiReadRxFifo(uint8_t * pReadData, uint8_t len);


/**************************************************************************************************
 */
#endif

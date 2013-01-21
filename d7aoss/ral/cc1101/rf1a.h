/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

//
// *************************************************************************************************
// Radio core access functions. Taken from TI reference code for CC430.
// *************************************************************************************************
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "../../types.h"


#define RADIO_INST_WRITE_WAIT()   CC1101_waitForIDle();
#define CLOCKS_PER_1us	20



// *************************************************************************************************
// Prototype section
unsigned char Strobe(unsigned char strobe);
unsigned char ReadSingleReg(unsigned char addr);
void WriteSingleReg(unsigned char addr, unsigned char value);
void ReadBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count);
void WriteBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count);
void ResetRadioCore (void);
void WritePATable(unsigned char value);
void WaitForXT2(void);


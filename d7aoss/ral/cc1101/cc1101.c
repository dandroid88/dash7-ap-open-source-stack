/*
 * cc1101.c
 *
 *  Created on: Dec 1, 2012
 *      Author: armin
 */
#include <stdbool.h>
#include <string.h>

#include "stdint.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"

#include "cc1101.h"
#include "cc1101_stellaris_spi.h"
#include "cc1101_constants.h"

uint8_t CC1101_TxBuf[CC1101_TX_BUFFER_SIZE];
uint8_t CC1101_RxBuf[CC1101_RX_BUFFER_SIZE];

volatile uint32_t CC1101_isr_count = 0;
volatile RXTX_flags flags;
// TODO Join CC1101_ReceiveMode with RXTXFLAGS
volatile bool CC1101_ReceiveMode = false;

void CC1101_Reset() {
//power on reset:
	// Toggle chip select signal
	radioDeselect();
	delayuS(30);
	radioSelect();
	delayuS(30);
	radioDeselect();
	delayuS(45);
	// Send SRES command
	CC1101_writestrobe(CC1101_SRES);
}


uint8_t CC1101_writestrobe(uint8_t data) {
	radioSelect();
	uint8_t result = spiSendByte(data & 0x3f);
	radioDeselect();
	return result;
}

void CC1101_writereg(uint8_t regAddr, uint8_t value) {
	radioSelect();
	spiSendByte((regAddr & 0x3F));
	spiSendByte(value);
	radioDeselect();
}

uint8_t CC1101_readreg(uint8_t regAddr) {
	radioSelect();
	spiSendByte((regAddr & 0x3F) | CC1101_READ_SINGLE);
	uint8_t val = spiSendByte(0); // send dummy byte to receive reply
	radioDeselect();
	return val;
}

uint8_t CC1101_readstatus(uint8_t regAddr) {
	uint8_t ret, retCheck, data, data2;
	uint8_t addr = (regAddr & 0x3F) | CC1101_READ_BURST;
	radioSelect();
	ret = spiSendByte(addr);
	data = spiSendByte(0); // send dummy byte to receive reply

	//See CC1101's Errata for SPI read errors
	while (true) {
		retCheck = spiSendByte(addr);
		data2 = spiSendByte(0);
		if (ret == retCheck && data == data2)
			break;
		else {
			ret = retCheck;
			data = data2;
		}
	}
	radioDeselect();
	return data;
}

void CC1101_burstwrite(uint8_t regAddr, uint8_t* buffer, uint8_t len) {
	uint8_t addr = (regAddr & 0x3F) | CC1101_WRITE_BURST;
	radioSelect();
	spiSendByte(addr);
	for (int i = 0; i < len; i++) {
		spiSendByte(buffer[i]);
	}
	radioDeselect();
}

void CC1101_burstread(uint8_t regAddr, uint8_t* buffer, uint8_t len) {
	uint8_t addr = (regAddr & 0x3F) | CC1101_READ_BURST;
	radioSelect();
	spiSendByte(addr);
	for (int i = 0; i < len; i++) {
		buffer[i] = spiSendByte(0); // send dummy byte to receive reply
	}
	radioDeselect();
}

uint8_t CC1101_init(void) {
	spiInit();
	debugprint("Start init radio\n");

	uint8_t tmp;
	uint8_t counter = 0;

	// TODO Only reset radio when needed (i.e. when marcstate is incorrect)
	CC1101_Reset();
//	CC1101_waitForIdle();

	CC1101_writestrobe(CC1101_SRES);
	tmp = CC1101_readstatus(CC1101_RXBYTES);
	if (tmp != 0) {
		CC1101_writestrobe(CC1101_SFRX);
	}
	tmp = CC1101_readstatus(CC1101_TXBYTES);
	if (tmp != 0) {
		CC1101_writestrobe(CC1101_SFTX);
	}

	//Write RF settings
	uint8_t i = 0, reg = 0xff;
	reg = rfSettings[i];
	while (reg != 0xFF) {
		CC1101_writereg(reg, rfSettings[i + 1]);
		tmp = CC1101_readreg(reg);

		if (rfSettings[i + 1] != tmp) {
			debugprint("%X:, %X !=  %X\n", rfSettings[i], tmp, rfSettings[i + 1]);
			counter++;
		}
		i += 2;
		reg = rfSettings[i];
	}
	if (counter > 0) {
		debugprint("\nRegisters not written properly: %d\n", counter);
	}
	CC1101_writereg(CC1101_IOCFG0, 0b01101111);	//GDO0 high state
	CC1101_writereg(CC1101_PKTLEN, 126);			//126-bytes maximum packet length
	CC1101_writereg(CC1101_PKTCTRL1, 0b00000111);	//APPEND_STATUS, ADR_CHK enable (broadcast addresses 0 and 255)
	CC1101_writereg(CC1101_PKTCTRL0, 0b00000101);	//CRC, variable packet length, format, whitening
	CC1101_writereg(CC1101_ADDR, 0x01);	//Set device's address
	CC1101_writereg(CC1101_MCSM1, 0b00000000);	//Disable CCA, RXOFF_MODE, TXOFF_MODE

//	//Set output power +10dbm
//	CC1101_writereg(CC1101_PATABLE, 0xc0);

//	Set output power -30dbm
//	CC1101_writereg(CC1101_PATABLE, 0x04);

	CC1101_writereg(CC1101_PATABLE, 0x20);

//	Set output power 0dbm
//	CC1101_writereg(CC1101_PATABLE, 0x50);

	CC1101_writestrobe(CC1101_SCAL);					//Need to calibrate before initial use
	radioConfigureInterrupt();

	return counter;
}

void CC1101_waitForIdle(void) {
	uint8_t tmp = 0;
#ifdef DEBUG
	uint8_t old = 1;
#endif
	while (tmp != 1) {								//Wait for CC1100 go to IDLE state
		tmp = CC1101_readstatus(CC1101_MARCSTATE);
#ifdef DEBUG
		if (old != tmp) {
			debugprint("MARCSTATE: %X\n", tmp);
			old = tmp;
		}
#endif
	}
}

void CC1101_IdleMode(void) {
	debugprint("CC1101_IdleMode\n");
	radioDisableInterrupt();
	CC1101_writestrobe(CC1101_SIDLE);
}

void CC1101_RxMode(void) {
	CC1101_ReceiveMode = true;

	CC1101_waitForIdle();
//	debugprint("Idle mode reached, start receiving\n");

	CC1101_writereg(CC1101_IOCFG0, 0b01000001);		//GDO0 associated to the RX FIFO
	CC1101_writestrobe(CC1101_SRX);
//#ifdef DEBUG
//	uint8_t marcState;
//	debugprint("MARCSTATE:");
//	for (int i = 0; i < 50; i++) {
//		if ((i % 25) == 0) {
//			debugprint("\n");
//		}
//		CC1101_readstatus(CC1101_MARCSTATE, &marcState);
//		debugprint(" %X", marcState);
////		_delay_us(200);
//	}
//	debugprint("\n");
//#endif

	radioEnableInterrupt();
}

void CC1101_TxMode(void) {
	CC1101_ReceiveMode = false;
	debugprint("CC1101_TxMode\n");

	CC1101_waitForIdle();
	CC1101_writereg(CC1101_IOCFG0, 0b00000010);		//GDO0 associated to the TX FIFO
	CC1101_writestrobe(CC1101_STX);
//#ifdef DEBUG
//	uint8_t marcState;
//	debugprint("MARCSTATE:");
//	for (int i = 0; i < 50; i++) {
//		if ((i % 25) == 0) {
//			debugprint("\n");
//		}
//		marcState = CC1101_readstatus(CC1101_MARCSTATE);
//		debugprint(" %X", marcState);
////		_delay_us(200);
//	}
//	debugprint("\n");
//#endif
	flags.RF_TX_DRDY = true;						//Tell transmit module, that data is ready
	radioEnableInterrupt();
}

void CC1101_SendPacket(uint8_t * buffer, uint8_t size, uint8_t sender, uint8_t receiver) {
	CC1101_TxBuf[0] = size + 2;
	CC1101_TxBuf[1] = receiver;
	CC1101_TxBuf[2] = sender;
	memcpy(CC1101_TxBuf + 3, buffer, size);
	CC1101_TxMode();
//	 stm32 doesn't have level interrupts, only edge. If the level is already low, we trigger a software interrupt
//	if (!(CC1101_GDO0_GPIO_PORT ->IDR & CC1101_GDO0_GPIO_PIN)) {
//		EXTI->SWIER &= CC1101_GDO0_EXTI_LINE;
//	}
//	debugprint("exti pending: %x\n", EXTI->PR);
//
	while (flags.RF_TX_DRDY)
		;
}


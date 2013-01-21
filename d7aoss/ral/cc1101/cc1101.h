/*
 * cc1101.h
 *
 *  Created on: Dec 1, 2012
 *      Author: armin
 */

#ifndef CC1101_H_
#define CC1101_H_
#include <stdbool.h>
#include <stdint.h>


//Global variables
#define CC1101_TX_BUFFER_SIZE	128
#define CC1101_RX_BUFFER_SIZE	128

extern uint8_t CC1101_TxBuf[CC1101_TX_BUFFER_SIZE];
extern uint8_t CC1101_RxBuf[CC1101_RX_BUFFER_SIZE];

typedef struct {
	bool RF_RX_DRDY :1;
	bool RF_TX_DRDY :1;
} RXTX_flags;

extern volatile RXTX_flags flags;

extern volatile uint32_t CC1101_isr_count;

uint8_t CC1101_writestrobe(uint8_t data);

void CC1101_writereg(uint8_t regAddr, uint8_t value);
uint8_t CC1101_readreg(uint8_t regAddr);

void CC1101_burstwrite(uint8_t regAddr, uint8_t* buffer, uint8_t len);
void CC1101_burstread(uint8_t regAddr, uint8_t* buffer, uint8_t len);

uint8_t CC1101_readstatus(uint8_t regAddr);
void CC1101_Reset(void);

uint8_t CC1101_init(void);

void CC1101_IdleMode(void);
void CC1101_RxMode(void);
void CC1101_TxMode(void);

void CC1101_waitForIdle(void);
void CC1101_SendPacket(uint8_t * buffer, uint8_t size, uint8_t sender, uint8_t receiver);
#endif /* CC1101_H_ */

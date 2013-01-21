/*
 * cc1101_isr.c
 *
 *  Created on: Dec 2, 2012
 *      Author: armin
 */
#include <stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_exti.h>
#include <stm32l1xx_syscfg.h>
#include <misc.h>

#include "debugprint.h"
#include "cc1101.h"
#include "cc1101_constants.h"
#include "led.h"
#include "radio_spi_hw.h"

extern volatile bool CC1101_ReceiveMode;
extern volatile RXTX_flags flags;

void GDO0_IRQHandler(void) {
	if (EXTI_GetITStatus(GDO0_EXTI_LINE ) != RESET) {
		CC1101_isr_count++;
		led_green_on();

		static uint8_t counter = 0;

		if (CC1101_ReceiveMode) {
			if (!flags.RF_RX_DRDY) {
				uint8_t rxbytes;
				uint8_t marc_state;

				rxbytes = CC1101_readstatus(CC1101_RXBYTES);

				//Check max length to fit RX buffer
				if ((rxbytes + counter + 2) > CC1101_RX_BUFFER_SIZE) {
					flags.RF_RX_DRDY = false;
					led_green_off();
					radioDisableInterrupt(); 				//disable interrupt
					CC1101_IdleMode();
				}

				if (rxbytes & 0x80) { //RXFIFO Overflow
#ifdef	DEBUG
					//				debugprint("RF RX Buffer\noverflowed!");
#else
					CC1101_writestrobe(CC1101_SFRX);	//Flush buffer
					counter = 0;//Flush counter
					CC1101_RxMode();//Go back into RX mode
					return;
#endif
				}

				marc_state = CC1101_readstatus(CC1101_MARCSTATE);
				if ((marc_state & 0x0f) == 0x01) { 		//Read all
					CC1101_burstread(CC1101_RXFIFO, CC1101_RxBuf + counter + 1, rxbytes);
					counter += rxbytes;
					CC1101_RxBuf[0] = counter;
					counter = 0;
					flags.RF_RX_DRDY = true;
					led_green_off();
					radioDisableInterrupt(); 				//disable interrupt
				}
				else {
					CC1101_burstread(CC1101_RXFIFO, CC1101_RxBuf + counter + 1, rxbytes - 1);
					counter += rxbytes - 1;
				}
			}
		}
		else {
			if (flags.RF_TX_DRDY) {
				uint8_t txbytes;
				uint8_t to_transmit;

				txbytes = CC1101_readstatus(CC1101_TXBYTES);
				if (txbytes & 0x80) {						//Tx FIFO underflow
#ifdef	DEBUG
					debugprint("RF TX Buffer\nunderflowed");
#else
					CC1101_writestrobe(CC1101_SFTX);	//Flush buffer
					counter = 0;//Flush counter
					CC1101_TxMode();//Try again
					return;
#endif
				}
				txbytes = 64 - txbytes;
				to_transmit = CC1101_TxBuf[0] - counter + 1;
				if (txbytes < to_transmit) {
					CC1101_burstwrite(CC1101_TXFIFO, CC1101_TxBuf + counter, txbytes);
					counter += txbytes;
				}
				else { //Completing transmission
					CC1101_burstwrite(CC1101_TXFIFO, CC1101_TxBuf + counter, to_transmit);
					counter = 0; 						//flush bytes counter
					flags.RF_TX_DRDY = false;			//Tell main module that transmission is finished
					led_green_off();
					radioDisableInterrupt(); 				//disable interrupt
				}
			}
		}
		// FIXME we should be able to clear the line, the radio should assert it -> or only in RX mode???
#ifdef RECEIVER
		/* Clear the  EXTI line pending bit */
		EXTI ->PR = GDO0_EXTI_LINE;
#endif
	}

}


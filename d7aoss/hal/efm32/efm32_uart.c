/*!
 *
 * \copyright (C) Copyright 2013 University of Antwerp (http://www.cosys-lab.be) and others.\n
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.\n
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * Contributors:
 * 		maarten.weyn@uantwerpen.be
 *
 */
 
#include "driverlib/EFM32/efm32.h"
#include "driverlib/efm32lib/efm32_gpio.h"
#include "driverlib/efm32lib/efm32_usart.h"
#include "driverlib/efm32lib/efm32_cmu.h"


#include "../uart.h"

void uart_init()
{
  /* Initialize USART */
    USART_TypeDef *usart = USART1;
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

    //CMU_ClockEnable(cmuClock_HFPER, true);
    //CMU_ClockEnable(cmuClock_USART1, true);

    /* Use default location 0: TX - Pin C0, RX - Pin C1 */
    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 1);
    /* Define input, no filtering */
    GPIO_PinModeSet(gpioPortC, 1, gpioModeInput, 0);

    /* Enable pins at default location */
    usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

    /* Configure USART for basic async operation */
    init.enable = usartDisable;
    USART_InitAsync(usart, &init);
    
     /* Clear previous RX interrupts */
    USART_IntClear((USART_TypeDef *) cmuClock_USART1, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ( USART1_RX_IRQn);

    /* Finally enable it */
    USART_Enable(usart, usartEnable);
}

void uart_enable_interrupt();

void uart_transmit_data(unsigned char data)
{
  USART_Tx(USART1, data);
}

void uart_transmit_message(unsigned char *data, unsigned char length)
{
  for (int i=0;i<length;i++)
    USART_Tx(USART1, *(data++));
}

unsigned char uart_tx_ready();

unsigned char uart_receive_data();
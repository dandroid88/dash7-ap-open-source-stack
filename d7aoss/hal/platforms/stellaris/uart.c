/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

#include "uart.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"
//#include "inc/hw_hibernate.h"
//#include "inc/hw_uart.h"
#include "inc/hw_ints.h"
//#include "driverlib/fpu.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
//#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
//#include "driverlib/hibernate.h"
#include "driverlib/uart.h"
//#include <msp430.h>


void uart_init()
{
	//
	// Initialize the UART and write status.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable port PA1 for UART0 U0TX
    //
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    //
    // Enable port PA0 for UART0 U0RX
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	//UARTStdioInit(0);
}

void uart_enable_interrupt()
{    

	  IntEnable(INT_UART0);                        // Enable USCI_A0 RX interrupt
}

void uart_disable_interrupt()
{
	  IntDisable(INT_UART1);
}


void uart_transmit_data(unsigned char data)
{
	while(!uart_tx_ready());

	UARTCharPutNonBlocking(UART0_BASE,data);
}


void uart_transmit_message(unsigned char *data, unsigned char length)
{
    unsigned char i=0;
    for (; i<length; i++)
    {
        uart_transmit_data(data[i]);
    }

}

unsigned char uart_tx_ready()
{
   // return UCA0IFG&UCTXIFG;
    return !UARTBusy(UART0_BASE);
}

unsigned char uart_receive_data()
{
    return UARTCharGetNonBlocking(UART0_BASE);
;
}

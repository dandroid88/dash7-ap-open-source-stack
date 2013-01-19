/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

#ifndef STELLARIS_H_
#define STELLARIS_H_

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
//#include "../addresses.h"

#define OUTPUT1_GPIO_PERIPH		SYSCTL_PERIPH_GPIOF
#define OUTPUT1_BASEADDRESS 	GPIO_PORTF_BASE
#define OUTPUT1_PORT			GPIO_PORTF_BASE
#define OUTPUT1_PIN 			GPIO_PIN_1

#define OUTPUT2_GPIO_PERIPH		SYSCTL_PERIPH_GPIOF
#define OUTPUT2_BASEADDRESS 	GPIO_PORTF_BASE
#define OUTPUT2_PORT 			GPIO_PORTF_BASE
#define OUTPUT2_PIN 			GPIO_PIN_2

#define OUTPUT3_GPIO_PERIPH		SYSCTL_PERIPH_GPIOF
#define OUTPUT3_BASEADDRESS  	GPIO_PORTF_BASE
#define OUTPUT3_PORT 			GPIO_PORTF_BASE
#define OUTPUT3_PIN 			GPIO_PIN_3

#define INPUT1_BASEADDRESS		GPIO_PORTF_BASE
#define INPUT1_GPIO_PERIPH		SYSCTL_PERIPH_GPIOF
#define INPUT1_PORT				GPIO_PORTF_BASE
#define INPUT1_PIN				GPIO_PIN_0

#define INPUT2_BASEADDRESS		GPIO_PORTF_BASE
#define INPUT2_GPIO_PERIPH		SYSCTL_PERIPH_GPIOF
#define INPUT2_PORT				GPIO_PORTF_BASE
#define INPUT2_PIN				GPIO_PIN_4


#define ALL_BUTTONS             (INPUT1_PIN | INPUT2_PIN)
// Stellaris does not have third button
//#define INPUT3_BASEADDRESS		__MSP430_BASEADDRESS_PORT1_R__
//#define INPUT3_PORT				GPIO_PORTF_BASE
//#define INPUT3_PIN				GPIO_PIN4

#endif /* WIZZIMOTE_H_ */

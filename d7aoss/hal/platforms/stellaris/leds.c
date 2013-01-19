/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

//#include "addresses.h"
#include "leds.h"
#include "../stellaris.h"
//#include "platforms/artesis.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
//#include "driverlib/watchdog.h"

void led_init()
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
   /* GPIO_setAsOutputPin(OUTPUT1_BASEADDRESS, OUTPUT1_PORT, OUTPUT1_PIN);
    GPIO_setAsOutputPin(OUTPUT2_BASEADDRESS, OUTPUT2_PORT, OUTPUT2_PIN);
    GPIO_setAsOutputPin(OUTPUT3_BASEADDRESS, OUTPUT3_PORT, OUTPUT3_PIN);*/
//    GPIODirModeSet(OUTPUT1_PORT, OUTPUT1_PIN,GPIO_DIR_MODE_OUT);
 //   GPIODirModeSet(OUTPUT2_PORT, OUTPUT2_PIN,GPIO_DIR_MODE_OUT);
  //  GPIODirModeSet(OUTPUT3_PORT, OUTPUT3_PIN,GPIO_DIR_MODE_OUT);

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, OUTPUT1_PIN|OUTPUT2_PIN|OUTPUT3_PIN);

}


void led_on(unsigned char led_nr)
{
    switch (led_nr)
    {
        case 1:
            //GPIO_setOutputHighOnPin(OUTPUT1_BASEADDRESS, OUTPUT1_PORT, OUTPUT1_PIN);
            GPIOPinWrite(OUTPUT1_PORT, OUTPUT1_PIN ,OUTPUT1_PIN);
            break;
        case 2:
            //GPIO_setOutputHighOnPin(OUTPUT2_BASEADDRESS, OUTPUT2_PORT, OUTPUT2_PIN);
            GPIOPinWrite(OUTPUT2_PORT, OUTPUT2_PIN ,OUTPUT2_PIN);
            break;
        case 3:
            //GPIO_setOutputHighOnPin(OUTPUT3_BASEADDRESS, OUTPUT3_PORT, OUTPUT3_PIN);
            GPIOPinWrite(OUTPUT3_PORT, OUTPUT3_PIN ,OUTPUT3_PIN);
            break;
    }
}

void led_off(unsigned char led_nr) {
	switch (led_nr) {
	case 1:
		//GPIO_setOutputHighOnPin(OUTPUT1_BASEADDRESS, OUTPUT1_PORT, OUTPUT1_PIN);
		GPIOPinWrite(OUTPUT1_PORT, OUTPUT1_PIN, !OUTPUT1_PIN);
		break;
	case 2:
		//GPIO_setOutputHighOnPin(OUTPUT2_BASEADDRESS, OUTPUT2_PORT, OUTPUT2_PIN);
		GPIOPinWrite(OUTPUT2_PORT, OUTPUT2_PIN, !OUTPUT2_PIN);
		break;
	case 3:
		//GPIO_setOutputHighOnPin(OUTPUT3_BASEADDRESS, OUTPUT3_PORT, OUTPUT3_PIN);
		GPIOPinWrite(OUTPUT3_PORT, OUTPUT3_PIN, !OUTPUT3_PIN);
		break;
	}
}

void led_toggle(unsigned char led_nr)
{
	//long led1val;
    switch (led_nr)
    {
	case 1:
		//GPIO_setOutputHighOnPin(OUTPUT1_BASEADDRESS, OUTPUT1_PORT, OUTPUT1_PIN);
	/*	led1val = GPIOPinRead(OUTPUT1_PORT, OUTPUT1_PIN);
		led1val = (led1val ^ OUTPUT1_PIN);
*/
		ROM_GPIOPinWrite(OUTPUT1_PORT, OUTPUT1_PIN, (GPIOPinRead(OUTPUT1_PORT, OUTPUT1_PIN) ^ OUTPUT1_PIN) );

		break;
	case 2:
		//GPIO_setOutputHighOnPin(OUTPUT2_BASEADDRESS, OUTPUT2_PORT, OUTPUT2_PIN);
		GPIOPinWrite(OUTPUT2_PORT, OUTPUT2_PIN,  (GPIOPinRead(OUTPUT2_PORT, OUTPUT2_PIN) ^ OUTPUT2_PIN) );
		break;
	case 3:
		//GPIO_setOutputHighOnPin(OUTPUT3_BASEADDRESS, OUTPUT3_PORT, OUTPUT3_PIN);
		GPIOPinWrite(OUTPUT3_PORT, OUTPUT3_PIN,  (GPIOPinRead(OUTPUT3_PORT, OUTPUT3_PIN) ^ OUTPUT3_PIN));
		break;
    }
}

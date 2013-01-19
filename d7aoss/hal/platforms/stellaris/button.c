/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

#include "button.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
//#include "driverlib/watchdog.h"

#include "../stellaris.h"

//#include "addresses.h"


void button_init()
{
/*
    GPIO_setAsInputPin(INPUT1_BASEADDRESS, INPUT1_PORT, INPUT1_PIN);
    GPIO_setAsInputPin(INPUT2_BASEADDRESS, INPUT2_PORT, INPUT2_PIN);
   // GPIO_setAsInputPin(INPUT2_BASEADDRESS, INPUT3_PORT, INPUT3_PIN);
*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	 //
	    // Unlock PF0 so we can change it to a GPIO input
	    // Once we have enabled (unlocked) the commit register then re-lock it
	    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
	    //
	HWREG(INPUT1_BASEADDRESS + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
	HWREG(INPUT1_BASEADDRESS + GPIO_O_CR) |= 0x01;
	HWREG(INPUT1_BASEADDRESS + GPIO_O_LOCK) = 0;

	    //
	    // Set each of the button GPIO pins as an input with a pull-up.
	    //


    GPIODirModeSet(INPUT1_PORT, INPUT1_PIN,GPIO_DIR_MODE_IN);
    GPIODirModeSet(INPUT2_PORT, INPUT2_PIN,GPIO_DIR_MODE_IN);

	//GPIODirModeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(INPUT1_BASEADDRESS, ALL_BUTTONS,
	                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);



}

void button_enable_interrupts()
{
  // GPIO_enableInterrupt(INPUT1_BASEADDRESS, INPUT1_PORT, INPUT1_PIN);
	IntEnable(INT_GPIOF);
	GPIOIntTypeSet(INPUT1_PORT, ALL_BUTTONS, GPIO_FALLING_EDGE);
    GPIOPinIntEnable(INPUT1_PORT, INPUT1_PIN);
    GPIOPinIntEnable(INPUT2_PORT, INPUT2_PIN);

  //  GPIO_enableInterrupt(INPUT2_BASEADDRESS, INPUT2_PORT, INPUT2_PIN);
    //GPIO_enableInterrupt(INPUT3_BASEADDRESS, INPUT3_PORT, INPUT3_PIN);

   /* GPIO_interruptEdgeSelect(INPUT1_BASEADDRESS, INPUT1_PORT, INPUT1_PIN,
        GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_interruptEdgeSelect(INPUT2_BASEADDRESS, INPUT2_PORT, INPUT2_PIN,
            GPIO_HIGH_TO_LOW_TRANSITION);
  */  //GPIO_interruptEdgeSelect(INPUT3_BASEADDRESS, INPUT3_PORT, INPUT3_PIN,
      //      GPIO_HIGH_TO_LOW_TRANSITION);

    button_clear_interrupt_flag();
}

void button_disable_interrupts()
{

   /* GPIO_disableInterrupt(INPUT1_BASEADDRESS, INPUT1_PORT, INPUT1_PIN);
    GPIO_disableInterrupt(INPUT2_BASEADDRESS, INPUT2_PORT, INPUT2_PIN);
  *///  GPIO_disableInterrupt(INPUT3_BASEADDRESS, INPUT3_PORT, INPUT3_PIN);
    GPIOPinIntDisable(INPUT1_PORT, INPUT1_PIN);
    GPIOPinIntDisable(INPUT2_PORT, INPUT2_PIN);
}

void button_clear_interrupt_flag()
{
    /*GPIO_clearInterruptFlag(INPUT1_BASEADDRESS, INPUT1_PORT, INPUT1_PIN);
	GPIO_clearInterruptFlag(INPUT2_BASEADDRESS, INPUT2_PORT, INPUT2_PIN);
	*///GPIO_clearInterruptFlag(INPUT3_BASEADDRESS, INPUT3_PORT, INPUT3_PIN);
	 GPIOPinIntClear(INPUT1_PORT, INPUT1_PIN);
	 GPIOPinIntClear(INPUT2_PORT, INPUT2_PIN);
}

unsigned char button_is_active(unsigned char button_nr)
{
    switch (button_nr)
    {
        case 1:
            return (!INPUT1_PIN == GPIOPinRead(INPUT1_PORT, INPUT1_PIN));
        case 2:
            return (!INPUT2_PIN  == GPIOPinRead(INPUT2_PORT, INPUT2_PIN));
       // case 3:
         //   return (GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(INPUT3_BASEADDRESS, INPUT3_PORT, INPUT3_PIN));
        default:
        	return false;
    }

}

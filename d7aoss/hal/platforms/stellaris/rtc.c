/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

//#include "addresses.h"
#include "rtc.h"
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


// Currrently 1 sec intervals
void rtc_init_counter_mode()//void (*timerhandler)(void))
{
	    //
	    // Enable the peripherals used by this example.
	    //
	    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

/*	    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet();
	//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	*/
	    ROM_IntEnable(INT_TIMER0A);

	   // ROM_TimerEnable(TIMER0_BASE, TIMER_A);

	    //
	    // Enable port PB6 for TIMER0 T0CCP0
	    //
	  //  GPIOPinConfigure(GPIO_PB6_T0CCP0);
	  //  GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_DIR_MODE_HW) ;
	  //  GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

	    //
	    // Enable port PB7 for TIMER0 T0CCP1
	    //
//	    GPIOPinConfigure(GPIO_PB7_T0CCP1);
//	    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7);

	    //
	    // Enable processor interrupts.
	/*    //
	    IntMasterEnable();


	    //
	    // Configure the two 32-bit periodic timers.
	    //
	    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_32_RTC);
	    ROM_TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
	  //  TimerLoadSet(TIMER0_BASE, TIMER_A, 1);

	  //  TimerIntRegister(TIMER0_BASE , TIMER_A,
	                               //  timerhandler);
	   // TimerMatchSet(TIMER0_BASE , TIMER_A, 2);

	    TimerRTCDisable(TIMER0_BASE);
	    TimerEnable(TIMER0_BASE, TIMER_A);

	    TimerRTCEnable(TIMER0_BASE);
*/

	     SysTickPeriodSet(SysCtlClockGet()/1000);

	     //
	     // Enable interrupts to the processor.
	     //
	     IntMasterEnable();

	     //
	     // Enable the SysTick Interrupt.
	     //
	     SysTickIntEnable();

	     //
	     // Enable SysTick.
	     //
	     SysTickEnable();



}

void Rtc_EnableInterrupt()
{
    //Enable interrupt for counter overflow
 /*   RTC_enableInterrupt(__MSP430_BASEADDRESS_RTC__,
        RTCTEVIE);*/

    //
	    // Setup the interrupts for the timer timeouts.
	    //



	SysTickIntEnable();
}

void rtc_disable_interrupt()
{
    //Enable interrupt for counter overflow
 /*   RTC_disableInterrupt(__MSP430_BASEADDRESS_RTC__,
        RTCTEVIE);*/
	//TimerIntDisable(TIMER0_BASE,TIMER_RTC_MATCH);
	SysTickIntDisable();
}

void rtc_start()
{
    //
    // Enable the timers.
    //
 //  TimerEnable(TIMER0_BASE, TIMER_A);
   //TimerRTCEnable(TIMER0_BASE);
    /*//Start RTC Clock
    RTC_startClock(__MSP430_BASEADDRESS_RTC__);*/

	SysTickEnable();
}

void rtc_stop()
{
   //Start RTC Clock
  //  RTC_holdClock(__MSP430_BASEADDRESS_RTC__);
    //
    // Enable the timers.
    //
//   TimerDisable(TIMER0_BASE,TIMER_A);
	SysTickDisable();
}


/*
 *  Created on: Nov 22, 2012
 *  Authors:
 * 		maarten.weyn@artesis.be
 *  	glenn.ergeerts@artesis.be
 */

#include "system.h"
#include "leds.h"
#include "button.h"
#include "uart.h"

//#include "inc/hw_memmap.h"
//#include "driverlib/5xx_6xx/wdt.h"

//#include "cc430_registers.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/watchdog.h"

u8 tag_id[8];

void PMM_SetStdSVSM(unsigned short svsmh_cfg, u8 Von, u8 Voffon) {
	/*    unsigned short svsmh_reg;
	 unsigned short svsml_reg;

	 PMMCTL0_H    = 0xA5;
	 svsmh_reg       = svsmh_cfg | ((unsigned short)Von << 8) | (unsigned short)Voffon;
	 svsml_reg       = SVSMLCTL & 0x070F;
	 svsml_reg      |= svsmh_reg & 0x08C0;   // Always disable SVML (useless)
	 PMMIFG        = 0;
	 PMMRIE        = 0;
	 SVSMHCTL   = svsmh_reg;
	 SVSMLCTL   = svsml_reg;
	 while ((PMMIFG & (SVSMLDLYIFG+SVSMHDLYIFG)) != (SVSMLDLYIFG+SVSMHDLYIFG));

	 PMMIFG        = 0;
	 PMMRIE        = 0x0130;               //Always enable SVSL reset, SVMH interrupt, SVS/MH Delayed interrupt
	 PMMCTL0_H    = 0x00;*/
}

void SetVCoreUp(unsigned char level) // Note: change level by one step only
{
	/*  PMMCTL0_H = 0xA5;                         // Open PMM module registers for write access

	 SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;     // Set SVS/M high side to new level

	 SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;     // Set SVM new Level
	 while ((PMMIFG & SVSMLDLYIFG) == 0);      // Wait till SVM is settled (Delay)
	 PMMCTL0_L = PMMCOREV0 * level;            // Set VCore to x
	 PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);        // Clear already set flags
	 if ((PMMIFG & SVMLIFG))
	 while ((PMMIFG & SVMLVLRIFG) == 0);     // Wait till level is reached

	 SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;     // Set SVS/M Low side to new level
	 PMMCTL0_H = 0x00;      */ // Lock PMM module registers for write access
}

void SetVCoreDown(unsigned char level) {
	/* PMMCTL0_H = 0xA5;                         // Open PMM module registers for write access
	 SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;     // Set SVS/M Low side to new level
	 //Wait until SVM high side and SVM low side is settled
	 while (PMMIFG & SVSMLDLYIFG == 0) ;
	 //while ((PMM->IFG & SVSMLDLYIFG) == 0);      // Wait till SVM is settled (Delay)
	 PMMCTL0_L = (level * PMMCOREV0);          // Set VCore to 1.85 V for Max Speed.
	 PMMCTL0_H = 0x00;  */ // Lock PMM module registers for write access
}

void PMM_SetVCore(u8 level) {
	/*   unsigned char actLevel;

	 // Note: change level by one step only
	 do {
	 actLevel = PMMCTL0_L & PMMCOREV_3;
	 if (actLevel < level)
	 SetVCoreUp(++actLevel);               // Set VCore (step by step)
	 if (actLevel > level)
	 SetVCoreDown(--actLevel);             // Set VCore (step by step)
	 } while (actLevel != level);*/
}

void system_init() {
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	SysCtlClockSet(
			SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN
					| SYSCTL_XTAL_16MHZ);
	//
	//peripherals are enabled within drivers
//
	system_watchdog_timer_init();
	system_watchdog_timer_stop();

	/*//PMM_setVCore(PMMCOREV_2);
	PMM_SetVCore(2);

	PMM_SetStdSVSM(0x8088, 2, 4);
*/
	led_init();
	button_init();
	uart_init();

	system_get_unique_id(tag_id);

}

void system_watchdog_timer_init() {

	//
	// Enable the peripherals used by this example.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

	//
	// Set the period of the watchdog timer to 1
	// second.
	//
	WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet());

	//
	// Enable reset generation from the
	// watchdog timer.
	//
	WatchdogResetEnable(WATCHDOG0_BASE);

	//
	// Enable the watchdog timer.
	//
	WatchdogResetDisable(WATCHDOG0_BASE);

	//
	// Enable processor interrupts.
	//
	IntMasterEnable();

	//
	// Register the watchdog interrupt handler
	//
	WatchdogIntRegister(WATCHDOG0_BASE, WatchdogIntHandler);

	//
	// Enable the watchdog interrupt.
	//
	IntEnable(INT_WATCHDOG);
}
void system_watchdog_timer_stop() {

	WatchdogResetDisable(WATCHDOG0_BASE);

}

void system_watchdog_timer_start() {

	WatchdogEnable(WATCHDOG0_BASE);
}

void
WatchdogIntHandler(void)
{
    //
    // Clear the watchdog interrupt.
    //
    WatchdogIntClear(WATCHDOG0_BASE);
}

void system_lowpower_mode(unsigned char mode, unsigned char enableInterrupts) {
	/* unsigned char registerSetting = 0;
	 switch (mode)
	 {
	 case 1:
	 registerSetting = LPM1_bits;
	 break;
	 case 2:
	 registerSetting = LPM2_bits;
	 break;
	 case 3:
	 registerSetting = LPM3_bits;
	 break;
	 case 4:
	 registerSetting = LPM4_bits;
	 break;
	 case 0:
	 default:
	 registerSetting = LPM0_bits;
	 break;
	 }

	 if (enableInterrupts)
	 registerSetting += GIE;

	 __bis_SR_register(registerSetting);*/
}

void system_get_unique_id(unsigned char *tagId) {

	//find something that will work for stellaris

	 tagId[0] = 0x00;
	 tagId[1] = 0x00;
	 tagId[2] = 0x00;
	 tagId[3] = 0x00;
	 tagId[4] = 0x00;
	 tagId[5] = 0x00;
	 tagId[6] = 0x00;
	 tagId[7] = 0x01;
}

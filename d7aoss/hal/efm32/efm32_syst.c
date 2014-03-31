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

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/EFM32/efm32.h"
#include "driverlib/efm32lib/efm32_chip.h"
#include "driverlib/efm32lib/efm32_emu.h"

#include "../system.h"
#include "../leds.h"

uint8_t device_id[8]; // TODO: keep this as global?
uint8_t virtual_id[2];

void PMM_SetVCore(uint8_t level) {

}

void system_init() {
        CHIP_Init();
  
        /* Ensure core frequency has been updated */
        SystemCoreClockUpdate();
        /* Setup SysTick Timer for 1 msec interrupts  */
        
        if (SysTick_Config(SystemCoreClock / 1000)) while (1) ;

        system_get_unique_id(device_id);
        
  
	//TODO: correct way to find virtual_id -> set by app layer
	virtual_id[0] = device_id[4] ^ device_id[5];
	virtual_id[1] = device_id[6] ^ device_id[7];

	
        /* Initialize LED driver */
        led_init();
        
	//button_init();
	//uart_init();
}

void system_watchdog_timer_stop() {

}

void system_watchdog_timer_start() {

}

void system_watchdog_timer_reset() {

}

void system_watchdog_timer_enable_interrupt() {

}

void system_watchdog_timer_init(unsigned char clockSelect,
		unsigned char clockDivider) {

}

void system_watchdog_init(unsigned char clockSelect, unsigned char clockDivider) {

}

void system_lowpower_mode(unsigned char mode, unsigned char enableInterrupts) {
	switch (mode) {
	case 4:
		//Only a power on reset or external reset pin can wake the device from EM4.
                EMU_EnterEM4();
		break;
	case 3:
              /*When entering EM3, the high frequency clocks are disabled by HW, ie HFXO,
               *   HFRCO and AUXHFRCO (for AUXHFRCO, see exception note below). In addition,
               *   the low frequency clocks, ie LFXO and LFRCO are disabled by SW. When
               *   re-entering EM0, HFRCO is re-enabled and the core will be clocked by the
               *   configured HFRCO band. This ensures a quick wakeup from EM3.
              */
		EMU_EnterEM3(true);
		break;
	case 2:
                /*
               * When entering EM2, the high frequency clocks are disabled, ie HFXO, HFRCO
               *   and AUXHFRCO (for AUXHFRCO, see exception note below). When re-entering
               *   EM0, HFRCO is re-enabled and the core will be clocked by the configured
               *   HFRCO band. This ensures a quick wakeup from EM2.
                */
		EMU_EnterEM2(true);
		break;
	case 1:
		EMU_EnterEM1();
		break;
	case 0:
		EMU_EnterEM1();
	default:
		break;
	}

}

void system_get_unique_id(unsigned char *tagId) {
        //TODO get device id (should be from filesystem)
	unsigned char i;
	for (i = 0; i < 8; i++) {
		tagId[i] = i;
	}

}

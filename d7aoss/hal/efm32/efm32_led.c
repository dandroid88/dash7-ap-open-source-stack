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
#include "driverlib/efm32lib/efm32_cmu.h"


#include "../leds.h"

void led_init()
{
        /* Enable GPIO */
        CMU_ClockEnable(cmuClock_GPIO, true);

        /* Configure GPIO port C 0-3 as LED control outputs */
        /* Enable the LED by default */
        GPIO_PinModeSet(LEDPORT, 0, gpioModePushPull, 1);
        GPIO_PinModeSet(LEDPORT, 1, gpioModePushPull, 1);
        GPIO_PinModeSet(LEDPORT, 2, gpioModePushPull, 1);
        GPIO_PinModeSet(LEDPORT, 3, gpioModePushPull, 1);
}


void led_on(unsigned char index) {
	if (index < 4)
        {
          GPIO_PinOutSet(LEDPORT, index);
        }
}

void led_toggle(unsigned char index) {
	if (index < 4)
        {
          GPIO_PinOutToggle(LEDPORT, index);
        }
}

void led_off(unsigned char index) {
      if (index < 4)
      {
        GPIO_PinOutClear(LEDPORT, index);
      }
}

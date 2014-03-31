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

#include "../button.h"


void button_init()
{
    /* Enable GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);

    /* Configure GPIO port C 0-3 as LED control outputs */
    /* Enable the LED by default */
    GPIO_PinModeSet(INPUT1_PORT, INPUT1_PIN, gpioModeInput, 0);
    GPIO_PinModeSet(INPUT2_PORT, INPUT2_PIN, gpioModeInput, 0);
}

void button_enable_interrupts()
{
  GPIO_IntConfig(INPUT1_PORT, INPUT1_PIN, false, true, true);
  GPIO_IntConfig(INPUT2_PORT, INPUT2_PIN, false, true, true);

  button_clear_interrupt_flag();
}

void button_disable_interrupts()
{
  GPIO_IntConfig(INPUT1_PORT, INPUT1_PIN, false, true, false);
  GPIO_IntConfig(INPUT2_PORT, INPUT2_PIN, false, true, false);
}

void button_clear_interrupt_flag()
{
  GPIO_IntClear(0xFF); //TODO: clear all fields???
}

unsigned char button_is_active(unsigned char button_nr)
{
    switch (button_nr)
    {
        case 1:
            return (0 == GPIO_PinInGet(INPUT1_PORT, INPUT1_PIN));
        case 2:
            return (0 == GPIO_PinInGet(INPUT2_PORT, INPUT2_PIN));
        default:
        	return false;
    }

}

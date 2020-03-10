/*
 * leds.c
 *
 *  Created on:
 *      Author: 
 */

#include "leds.h"

void init_leds()
{
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortE);

	port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
			kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
			kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister, };

	PORT_SetPinConfig(PORTB, 21, &config_led);  //Blue
	PORT_SetPinConfig(PORTB, 22, &config_led);	//Red
	PORT_SetPinConfig(PORTE, 26, &config_led);	//Green

	gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };

	GPIO_PinInit(GPIOB, 21, &led_config_gpio);
	GPIO_PinInit(GPIOB, 22, &led_config_gpio);
	GPIO_PinInit(GPIOE, 26, &led_config_gpio);
}

void green_led()
{
	GPIO_PinWrite(GPIOE, 26, 0);
	GPIO_PinWrite(GPIOB, 22, 1);
	GPIO_PinWrite(GPIOB, 21, 1);
}

void red_led()
{
	GPIO_PinWrite(GPIOE, 26, 1);
	GPIO_PinWrite(GPIOB, 22, 0);
	GPIO_PinWrite(GPIOB, 21, 1);
}

void blue_led()
{
	GPIO_PinWrite(GPIOE, 26, 1);
	GPIO_PinWrite(GPIOB, 22, 1);
	GPIO_PinWrite(GPIOB, 21, 0);
}

void turnLedsOff()
{//turn all the Kinetis leds off
	GPIO_PinWrite(GPIOB, 21, 1);
	GPIO_PinWrite(GPIOB, 22, 1);
	GPIO_PinWrite(GPIOE, 26, 1);
}

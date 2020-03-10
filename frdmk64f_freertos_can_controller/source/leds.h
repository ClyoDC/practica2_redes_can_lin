/*
 * leds.h
 *
 *  Created on: 
 *      Author: 
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#ifndef LEDS_H_
#define LEDS_H_

void init_leds();

void green_led();

void red_led();

void blue_led();

void turnLedsOff();

#endif /* LEDS_H_ */

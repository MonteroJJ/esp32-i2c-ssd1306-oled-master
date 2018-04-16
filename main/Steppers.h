/*
 * Steppers.h
 *
 *  Created on: Mar 27, 2018
 *      Author: JJ
 */

#ifndef MAIN_STEPPERS_H_
#define MAIN_STEPPERS_H_





#endif /* MAIN_STEPPERS_H_ */
#include "Configuration.h"
#include "Definitions.h"
#include <stdint.h>
#include "esp_log.h" // functions for logging
#include "fonts.h"
#include "ssd1306.hpp"
#include "driver/gpio.h" //general pin intput output
#include "driver/adc.h" //analog digital converter
#include "driver/uart.h" //unsyncr serial
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/pcnt.h" //interrupts
#include "driver/timer.h" //timers
#include <cstdio>
#include <string>
#include <sstream>
#include <iostream>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_err.h"

#include "driver/mcpwm.h" // motor control
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

void positionControl();
void setSpeedS(int target_sx, int target_sy);
void setMotorXSpeed(int16_t tspeed, int16_t dt);
void setMotorYSpeed(int16_t tspeed,int16_t dt);
void setPosition(int target_x_mm_new, int target_y_mm_new);
void setPosition_mm10(int target_x_mm_new, int target_y_mm_new);
void setSpeedS(int target_sx, int target_sy);

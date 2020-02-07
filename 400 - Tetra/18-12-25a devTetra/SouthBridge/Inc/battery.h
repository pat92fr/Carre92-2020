/*
 * battery.h
 *
 *  Created on: 25 d�c. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_BATTERY_H_
#define APPLICATION_USER_HAL_BATTERY_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* HAL settings ------------------------------------------------------------------*/

enum VBATT_ID
{
	VBATT = 0,
	VBATT_BIS,
	VBATT_COUNT
};

static float const hal_battery_voltage_hw_ration[VBATT_COUNT] = {
	0.091f, // calibrated, measured on board
	0.091f
};

static float const hal_battery_voltage_low_threshold[VBATT_COUNT] = {
	11.4, // V
	11.4
};

/* HAL Public Data ------------------------------------------------------------------*/


/* HAL Functions ------------------------------------------------------------------*/

void HAL_Battery_Init(void);

bool HAL_Battery_Is_Low();

float HAL_Battery_Get(int id); // volt


#endif /* APPLICATION_USER_HAL_BATTERY_H_ */

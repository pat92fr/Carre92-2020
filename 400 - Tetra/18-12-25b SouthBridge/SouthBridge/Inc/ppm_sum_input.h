/*
 * ppm_sum_input.h
 *
 *  Created on: 18 févr. 2018
 *      Author: Patrick
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PPM_SUM_INPUT_H_
#define PPM_SUM_INPUT_H_


/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/
#define SBUS_CHANNEL_COUNT 9

typedef struct
{
	TIM_HandleTypeDef * htim;
	uint16_t times[SBUS_CHANNEL_COUNT];
	uint32_t time_index;
	uint32_t full;
	uint32_t full_timestamp;
} HAL_PPM_SUM_Input_HandleTypeDef;

/* APP Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_PPM_SUM_Input_Init(
		HAL_PPM_SUM_Input_HandleTypeDef * hppmi,
		TIM_HandleTypeDef * htim
);

int32_t HAL_PPM_SUM_Input_Get(
		HAL_PPM_SUM_Input_HandleTypeDef * hppmi,
		uint32_t channel,
		uint32_t * timestamp);


/* IRS ------------------------------------------------------------------*/

void HAL_PPM_SUM_Input_ISR(TIM_HandleTypeDef *htim); //! TIM_IC_CaptureCallback

#ifdef __cplusplus
}
#endif

#endif /* PPM_SUM_INPUT_H_ */


/*
 * ppm_sum_input.c
 *
 *  Created on: 18 févr. 2018
 *      Author: Patrick
 */

/* Includes ------------------------------------------------------------------*/
#include "ppm_sum_input.h"
#include <string.h>

/* HAL settings ------------------------------------------------------------------*/

/* HAL Private data --------------------------------------------------------------*/
#define HAL_PPM_SUM_Input_max_handles 3
static HAL_PPM_SUM_Input_HandleTypeDef * HAL_PPM_SUM_Input_handles[HAL_PPM_SUM_Input_max_handles];
static uint32_t HAL_PPM_SUM_Input_handles_count = 0;

/* HAL functions ---------------------------------------------------------*/

void HAL_PPM_SUM_Input_Init(
		HAL_PPM_SUM_Input_HandleTypeDef * hppmi,
		TIM_HandleTypeDef * htim
)
{
	hppmi->htim = htim;
	memset(hppmi->times,0,SBUS_CHANNEL_COUNT*sizeof(uint16_t));
	hppmi->time_index = 0;
	hppmi->full = 0;
	hppmi->full_timestamp = 0;
	HAL_TIM_IC_Start_IT(hppmi->htim,TIM_CHANNEL_1);
	HAL_PPM_SUM_Input_handles[HAL_PPM_SUM_Input_handles_count++]=hppmi;
}

int32_t HAL_PPM_SUM_Input_Get(
		HAL_PPM_SUM_Input_HandleTypeDef * hppmi,
		uint32_t channel,
		uint32_t * timestamp)
{
	if(hppmi->full)
	{
		if(timestamp)
		{
			* timestamp=hppmi->full_timestamp;
		}
		return hppmi->times[channel];
	}
	else
		return 0;
}

void HAL_PPM_SUM_Input_ISR(TIM_HandleTypeDef *htim)
{
	for(uint32_t index=0;index<HAL_PPM_SUM_Input_handles_count;++index)
	{
		HAL_PPM_SUM_Input_HandleTypeDef * ppmi = HAL_PPM_SUM_Input_handles[index];
		// PPM SUM decoding
		if(htim==ppmi->htim)
		{
			uint32_t width = __HAL_TIM_GET_COMPARE(ppmi->htim,TIM_CHANNEL_1); // 1000 .. 2000
			if(width>3000)
				ppmi->time_index = 0;
			ppmi->times[ppmi->time_index++] = width;
			ppmi->time_index%=SBUS_CHANNEL_COUNT;
			if(ppmi->time_index==0)
			{
				ppmi->full = 1;
				ppmi->full_timestamp = HAL_GetTick();
			}
		}
	}
}

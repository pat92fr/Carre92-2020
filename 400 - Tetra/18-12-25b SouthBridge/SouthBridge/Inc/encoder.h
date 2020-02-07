/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_Encoder_H
#define __HAL_Encoder_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* APP Public Data ------------------------------------------------------------------*/

typedef enum
{
  HAL_ENCODER_LEFT = 0,
  HAL_ENCODER_RIGHT,
  HAL_ENCODER_ALL
} HAL_ENCODER_NAME;

typedef struct
{
	TIM_HandleTypeDef * htim_right;
	TIM_HandleTypeDef * htim_left;
	uint32_t last[2];
} HAL_Encoder_HandleTypeDef;

/* APP Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_Encoder_Init(
		HAL_Encoder_HandleTypeDef * hencoder,
		TIM_HandleTypeDef * htim_right,
		TIM_HandleTypeDef * htim_left
);

void HAL_Encoder_Reset(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id );

int32_t HAL_Encoder_Get(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id);

int32_t HAL_Encoder_Delta(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id );

#ifdef __cplusplus
}
#endif

/* IRS ------------------------------------------------------------------*/


#endif /* __HAL_Encoder_H */




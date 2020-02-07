/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/* HAL settings ------------------------------------------------------------------*/

/* HAL Private data --------------------------------------------------------------*/


/* HAL functions ---------------------------------------------------------*/

void HAL_Encoder_Init(
		HAL_Encoder_HandleTypeDef * hencoder,
		TIM_HandleTypeDef * htim_right,
		TIM_HandleTypeDef * htim_left
)
{
	hencoder->htim_right = htim_right;
	hencoder->htim_left = htim_left;
	hencoder->last[HAL_ENCODER_LEFT] = 0;
	hencoder->last[HAL_ENCODER_RIGHT] = 0;
	HAL_Encoder_Reset(hencoder,HAL_ENCODER_ALL);
    HAL_TIM_Encoder_Start(hencoder->htim_right,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(hencoder->htim_left,TIM_CHANNEL_ALL);
}

void HAL_Encoder_Reset(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id
)
{
    switch(id)
    {
    case HAL_ENCODER_LEFT:
        {
            __HAL_TIM_GetCounter(hencoder->htim_left) = 0;
            hencoder->last[HAL_ENCODER_LEFT] = 0;
        }
        break;
    case HAL_ENCODER_RIGHT:
        {
        	__HAL_TIM_GetCounter(hencoder->htim_right) = 0;
        	hencoder->last[HAL_ENCODER_RIGHT] = 0;
        }
        break;
    case HAL_ENCODER_ALL :
        {
        	__HAL_TIM_GetCounter(hencoder->htim_left) = 0;
        	__HAL_TIM_GetCounter(hencoder->htim_right) = 0;
        	hencoder->last[HAL_ENCODER_LEFT] = 0;
        	hencoder->last[HAL_ENCODER_RIGHT] = 0;
        }
        break;
    }
}

int32_t HAL_Encoder_Get(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id)
{
    switch(id)
    {
    case HAL_ENCODER_LEFT:
        {
            return (int32_t)__HAL_TIM_GetCounter(hencoder->htim_left);
        }
        break;
    case HAL_ENCODER_RIGHT:
        {
            return (int32_t)__HAL_TIM_GetCounter(hencoder->htim_right);
        }
        break;
    case HAL_ENCODER_ALL :
    default:
        {
            return 0;
        }
        break;
    }
}
int32_t HAL_Encoder_Delta(
		HAL_Encoder_HandleTypeDef * hencoder,
		HAL_ENCODER_NAME id )
{
    switch(id)
    {
    case HAL_ENCODER_LEFT:
        {
            uint32_t tmp = (uint32_t)__HAL_TIM_GetCounter(hencoder->htim_left);
            int32_t delta = tmp - hencoder->last[HAL_ENCODER_LEFT];
            hencoder->last[HAL_ENCODER_LEFT] = tmp;
//            if(delta>32768/2-1)
//                return delta-32768;
//            else if(delta<-32768/2-1)
//                return delta+32768;
//            else
                return delta;
        }
        break;
    case HAL_ENCODER_RIGHT:
        {
            uint32_t tmp = (uint32_t)__HAL_TIM_GetCounter(hencoder->htim_right);
            int32_t delta = tmp - hencoder->last[HAL_ENCODER_RIGHT];
            hencoder->last[HAL_ENCODER_RIGHT] = tmp;
//            if(delta>32768/2-1)
//                return delta-32768;
//            else if(delta<-32768/2-1)
//                return delta+32768;
//            else
                return delta;
        }
        break;
    case HAL_ENCODER_ALL :
    default:
        {
            return 0;
        }
        break;
    }
}


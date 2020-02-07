/*
 * led.c
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */


/* Includes ------------------------------------------------------------------*/
#include "Led.h"
#include <string.h>

/* Private data --------------------------------------------------------------*/
#define HAL_LED_max_handles 6
static HAL_LED_HandleTypeDef * HAL_LED_handles[HAL_LED_max_handles];
static uint32_t HAL_LED_handles_count = 0;

/* HAL functions ---------------------------------------------------------*/

void HAL_Led_Init()
{

}

void HAL_Led_Add(
		HAL_LED_HandleTypeDef * hled,
		GPIO_TypeDef * port,
		uint16_t pin
)
{
	hled->port = port;
	hled->pin = pin;
	hled->state = LED_OFF;
	memset(hled->sequence,0,32);
	hled->sequence_length = 0;
	hled->sequence_current_index = 0;
	hled->sequence_last_time_index_change = 0;
	hled->sequence_repeat = false;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
	HAL_LED_handles[HAL_LED_handles_count++] = hled;
}

void HAL_Led_Set(HAL_LED_HandleTypeDef * hled)
{
	hled->state = LED_ON;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

void HAL_Led_Reset(HAL_LED_HandleTypeDef * hled)
{
	hled->state = LED_OFF;
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

void HAL_Led_Toggle(HAL_LED_HandleTypeDef * hled)
{
	if(hled->state == LED_OFF)
	{
		hled->state = LED_ON;
	}
	else
	{
		hled->state = LED_OFF;
	}
	HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
}

int HAL_Led_Get(HAL_LED_HandleTypeDef * hled)
{
	return hled->state;
}

void HAL_Led_Sequence(HAL_LED_HandleTypeDef * hled, char * sequence, bool repeat)
{
	uint32_t len = strlen(sequence);
	if(len<HAL_LED_max_sequence_len)
	{
		memset(hled->sequence,0,32);
		memcpy(hled->sequence,sequence,len);
		hled->sequence_length = len;
		hled->sequence_current_index = 0;
		hled->sequence_last_time_index_change = 0;
		hled->sequence_repeat = repeat;
	}
	else
	{
		memset(hled->sequence,0,32);
		hled->sequence_length = 0;
		hled->sequence_current_index = 0;
		hled->sequence_last_time_index_change = 0;
		hled->sequence_repeat = false;
	}
}

void HAL_Led_Process(void)
{
    uint32_t const current_time = HAL_GetTick();
    for(uint32_t index=0;index<HAL_LED_handles_count;++index)
    {
    	HAL_LED_HandleTypeDef * hled = HAL_LED_handles[index];
    	if(hled->sequence_length>0)
    	{
    		char current_event = hled->sequence[hled->sequence_current_index];

    		switch(current_event)
    		{
    		case ' ':
				{
					hled->state = LED_OFF;
					HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
					if(current_time>=hled->sequence_last_time_index_change+HAL_LED_blinking_period)
					{
						++hled->sequence_current_index;
						hled->sequence_last_time_index_change = current_time;
						if(hled->sequence[hled->sequence_current_index]==0) // end of sequence
						{
							hled->sequence_current_index = 0;
							if(!hled->sequence_repeat) // dont repeat
							{
								memset(hled->sequence,0,32);
								hled->sequence_length = 0;
								hled->sequence_last_time_index_change = 0;
								hled->sequence_repeat = false;
							}
						}
					}
				}
				break;
    		case '.':
				{
					hled->state = LED_ON;
					HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
					if(current_time>=hled->sequence_last_time_index_change+HAL_LED_blinking_period)
					{
						++hled->sequence_current_index;
						hled->sequence_last_time_index_change = current_time;
						if(hled->sequence[hled->sequence_current_index]==0) // end of sequence
						{
							hled->sequence_current_index = 0;
							if(!hled->sequence_repeat) // dont repeat
							{
								memset(hled->sequence,0,32);
								hled->sequence_length = 0;
								hled->sequence_last_time_index_change = 0;
								hled->sequence_repeat = false;
							}
						}
					}
				}
				break;
    		case '_':
				{
					hled->state = LED_ON;
					HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
					if(current_time>=hled->sequence_last_time_index_change+HAL_LED_blinking_period)
					{
						++hled->sequence_current_index;
						hled->sequence_last_time_index_change = current_time;
						if(hled->sequence[hled->sequence_current_index]==0) // end of sequence
						{
							hled->sequence_current_index = 0;
							if(!hled->sequence_repeat) // dont repeat
							{
								memset(hled->sequence,0,32);
								hled->sequence_length = 0;
								hled->sequence_last_time_index_change = 0;
								hled->sequence_repeat = false;
							}
						}
					}
				}
				break;
    		default:
				{
					hled->state = LED_OFF;
					HAL_GPIO_WritePin(hled->port,hled->pin,hled->state);
					if(current_time>=hled->sequence_last_time_index_change+HAL_LED_blinking_period)
					{
						++hled->sequence_current_index;
						hled->sequence_last_time_index_change = current_time;
						if(hled->sequence[hled->sequence_current_index]==0) // end of sequence
						{
							hled->sequence_current_index = 0;
							if(!hled->sequence_repeat) // dont repeat
							{
								memset(hled->sequence,0,32);
								hled->sequence_length = 0;
								hled->sequence_last_time_index_change = 0;
								hled->sequence_repeat = false;
							}
						}
					}
				}
				break;
    		}
        }
    }
}



/*
 * led.h
 *
 *  Created on: 25 déc. 2015
 *      Author: Patrick
 */

#ifndef APPLICATION_USER_HAL_LED_H_
#define APPLICATION_USER_HAL_LED_H_


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ' ' short OFF
// '.' shot ON
// '_' long ON


/* APP Public Data ------------------------------------------------------------------*/

#define HAL_LED_max_sequence_len 32
#define HAL_LED_blinking_period 100

typedef struct
{
	GPIO_TypeDef * port;
	uint16_t pin;
	GPIO_PinState state;
	// blinking from string
	char sequence[HAL_LED_max_sequence_len]; //! ' ' off, '.' on short duration, '_' on long duration
	uint32_t sequence_length;
	uint32_t sequence_current_index;
	uint32_t sequence_last_time_index_change;
	bool sequence_repeat;

} HAL_LED_HandleTypeDef;

/* HAL Public Data ------------------------------------------------------------------*/

enum LED_STATE
{
    LED_OFF = 0,
    LED_ON = 1
};

/* HAL Functions ------------------------------------------------------------------*/

void HAL_Led_Init();

void HAL_Led_Add(
		HAL_LED_HandleTypeDef * hled,
		GPIO_TypeDef * port,
		uint16_t pin
);

void HAL_Led_Process(void);

void HAL_Led_Set(HAL_LED_HandleTypeDef * hled);
void HAL_Led_Reset(HAL_LED_HandleTypeDef * hled);
void HAL_Led_Toggle(HAL_LED_HandleTypeDef * hled);

int HAL_Led_Get(HAL_LED_HandleTypeDef * hled);

void HAL_Led_Sequence(HAL_LED_HandleTypeDef * hled, char * sequence, bool repeat);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_HAL_LED_H_ */

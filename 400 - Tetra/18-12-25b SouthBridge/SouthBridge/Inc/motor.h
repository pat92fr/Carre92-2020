/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HAL_MOTOR_H
#define __HAL_MOTOR_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* HAL Public Data ------------------------------------------------------------------*/

static const uint32_t motor_speed_limiter_forward = 100; // %
static const uint32_t motor_speed_limiter_backward = 50; // %
static const uint32_t motor_min_speed_trigger_brake = 20; // PWM / 1000

typedef enum
{
  HAL_MOTOR_BRAKE,
//  HAL_MOTOR_CW,
//  HAL_MOTOR_CCW,
  HAL_MOTOR_AUTO
} HAL_MOTOR_STATE;

typedef enum
{
  HAL_MOTOR_LEFT = 0,
  HAL_MOTOR_RIGHT,
  HAL_MOTOR_ALL
} HAL_MOTOR_NAME;

enum MOTOR_CONTROLS_ID
{
    LEFT1 = 0,
    LEFT2,
    RIGHT1,
    RIGHT2,
};

// Control ports and pins to TB6612FNG : adjust according design and wiring
static uint16_t const motor_id_to_pin[4] = {
		M1_INA_Pin, // LDIR1
		M1_INB_Pin, // LDIR2
		M2_INB_Pin, // RDIR1
		M2_INA_Pin  // RDIR2
};

// Control ports and pins to TB6612FNG : adjust according design and wiring
static GPIO_TypeDef * const motor_id_to_port[4] = {
		M1_INA_GPIO_Port,
		M1_INB_GPIO_Port,
		M2_INB_GPIO_Port,
		M2_INA_GPIO_Port
};

/* HAL Functions ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_Motor_Init(void);
void HAL_Motor_Set(HAL_MOTOR_NAME name, HAL_MOTOR_STATE state, int speed);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_MOTOR_H */


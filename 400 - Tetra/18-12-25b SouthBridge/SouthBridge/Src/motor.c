/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "mymath.h"
#include "stm32f4xx_hal.h"

/* Private data --------------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;

static uint16_t const HAL_MOTOR_AUTORELOADREG = 999; // valeur du registre Auto Reload Register du Timers 1

/* Private Functions ------------------------------------------------------------------*/

static void brake_left_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[LEFT1], motor_id_to_pin[LEFT1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_id_to_port[LEFT2], motor_id_to_pin[LEFT2], GPIO_PIN_SET);
}

static void brake_right_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT1], motor_id_to_pin[RIGHT1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT2], motor_id_to_pin[RIGHT2], GPIO_PIN_SET);
}

static void cw_left_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[LEFT1], motor_id_to_pin[LEFT1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_id_to_port[LEFT2], motor_id_to_pin[LEFT2], GPIO_PIN_SET);
}

static void cw_right_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT1], motor_id_to_pin[RIGHT1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT2], motor_id_to_pin[RIGHT2], GPIO_PIN_RESET);
}

static void ccw_left_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[LEFT1], motor_id_to_pin[LEFT1], GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor_id_to_port[LEFT2], motor_id_to_pin[LEFT2], GPIO_PIN_RESET);
}

static void ccw_right_motor()
{
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT1], motor_id_to_pin[RIGHT1], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_id_to_port[RIGHT2], motor_id_to_pin[RIGHT2], GPIO_PIN_SET);
}

static void set_left_motor_speed(int speed)
{
	htim1.Instance->CCR2 = speed; // Here adjust the timer channel
}

static void set_right_motor_speed(int speed)
{
	htim1.Instance->CCR1 = speed; // Here adjust the timer channel
}

/* APP Functions ------------------------------------------------------------------*/

void HAL_Motor_Init(void)
{
	HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_BRAKE,0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void HAL_Motor_Set(HAL_MOTOR_NAME name, HAL_MOTOR_STATE state, int speed)
{
    switch(name)
    {
    case HAL_MOTOR_LEFT:
        {
            if(state==HAL_MOTOR_BRAKE)
            {
                // apply
                brake_left_motor();
                if(speed < 0)
                {
                    set_left_motor_speed(constrain(-speed, 0, HAL_MOTOR_AUTORELOADREG));
                }
                else
                {
                    set_left_motor_speed(constrain(speed, 0, HAL_MOTOR_AUTORELOADREG));
                }
            }
            else // AUTO
            {
                if(speed < 0)
                {
                    cw_left_motor();
                    set_left_motor_speed(constrain(-speed, 0, HAL_MOTOR_AUTORELOADREG*motor_speed_limiter_backward/100));
                }
                else
                {
                    ccw_left_motor();
                    set_left_motor_speed(constrain(speed, 0, HAL_MOTOR_AUTORELOADREG*motor_speed_limiter_forward/100));
                }
            }
        }
        break;
    case HAL_MOTOR_RIGHT:
        {
            if(state==HAL_MOTOR_BRAKE)
            {
                // apply
                brake_right_motor();
                if(speed < 0)
                {
                	set_right_motor_speed(constrain(-speed, 0, HAL_MOTOR_AUTORELOADREG));
                }
                else
                {
                	set_right_motor_speed(constrain(speed, 0, HAL_MOTOR_AUTORELOADREG));
                }
            }
            else // AUTO
            {
                if(speed < 0)
                {
                    ccw_right_motor();
                    set_right_motor_speed(constrain(-speed, 0, HAL_MOTOR_AUTORELOADREG*motor_speed_limiter_backward/100));
                }
                else
                {
                    cw_right_motor();
                    set_right_motor_speed(constrain(speed, 0, HAL_MOTOR_AUTORELOADREG*motor_speed_limiter_forward/100));
                }
            }
        }
        break;
    case HAL_MOTOR_ALL:
        {
            HAL_Motor_Set(HAL_MOTOR_LEFT,state,speed);
            HAL_Motor_Set(HAL_MOTOR_RIGHT,state,speed);
        }
        break;
    }
}




/*
 * tools.c
 *
 *  Created on: 5 janv. 2020
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "math.h"

/******************************************************************************
  * @brief  Set Led state
  * @param led_num led number
  * @param state 0 or 1
  */
void LedSwitch(int led_num, int state)
{
  GPIO_TypeDef * port;
  int pin;

  if(led_num == 2)
  {
    port=LED2_GPIO_Port;
    pin=LED2_Pin;
  }
  if(led_num == 3)
  {
    port=LED3_GPIO_Port;
    pin=LED3_Pin;
  }
  if(led_num == 4)
  {
    port=LED4_GPIO_Port;
    pin=LED4_Pin;
  }

  if(state)
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

/******************************************************************************
  * @brief  Square a value
  * @param x value to square
  * @retval x*x
  */
double Square(double x)
{
  return x*x;
}

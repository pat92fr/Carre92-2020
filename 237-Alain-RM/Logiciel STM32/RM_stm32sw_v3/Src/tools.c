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

  HAL_GPIO_WritePin(port, pin, state);
}

float sigmoid(float x, float lambda)
{
  float sig;
  sig=1/(1+exp(-lambda*x));
  return sig;
}

float sigmoid_reduced(float x, float lambda)
{
  float sigp5, sigm5;
  float a, b, g;
  sigp5=sigmoid(5, lambda);
  sigm5=sigmoid(-5, lambda);
  a=1/(sigp5-sigm5);
  b=-a*sigm5;
  g=-a*sigmoid(10*x-5, lambda)+b;
  return g;
}

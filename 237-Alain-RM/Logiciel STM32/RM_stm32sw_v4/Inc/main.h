/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USR_BUTT_Pin GPIO_PIN_13
#define USR_BUTT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_14
#define OSC_IN_GPIO_Port GPIOC
#define OSC_OUT_Pin GPIO_PIN_15
#define OSC_OUT_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOH
#define IO_IN_2_Pin GPIO_PIN_1
#define IO_IN_2_GPIO_Port GPIOH
#define IO_OUT_1_Pin GPIO_PIN_0
#define IO_OUT_1_GPIO_Port GPIOC
#define IO_OUT_2_Pin GPIO_PIN_3
#define IO_OUT_2_GPIO_Port GPIOC
#define S21i4_Pin GPIO_PIN_0
#define S21i4_GPIO_Port GPIOA
#define S04x4_Pin GPIO_PIN_1
#define S04x4_GPIO_Port GPIOA
#define UART2_TX_Pin GPIO_PIN_2
#define UART2_TX_GPIO_Port GPIOA
#define UART2_RX_Pin GPIO_PIN_3
#define UART2_RX_GPIO_Port GPIOA
#define ADC_IN_Pin GPIO_PIN_4
#define ADC_IN_GPIO_Port GPIOA
#define S05i5_Pin GPIO_PIN_6
#define S05i5_GPIO_Port GPIOA
#define S11_Pin GPIO_PIN_7
#define S11_GPIO_Port GPIOA
#define S10_Pin GPIO_PIN_0
#define S10_GPIO_Port GPIOB
#define S01_Pin GPIO_PIN_1
#define S01_GPIO_Port GPIOB
#define S02_Pin GPIO_PIN_2
#define S02_GPIO_Port GPIOB
#define S03_Pin GPIO_PIN_10
#define S03_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOB
#define S22i1_Pin GPIO_PIN_14
#define S22i1_GPIO_Port GPIOB
#define S20i6_Pin GPIO_PIN_6
#define S20i6_GPIO_Port GPIOC
#define S06x6_Pin GPIO_PIN_7
#define S06x6_GPIO_Port GPIOC
#define S07_Pin GPIO_PIN_8
#define S07_GPIO_Port GPIOC
#define S08_Pin GPIO_PIN_9
#define S08_GPIO_Port GPIOC
#define S15i2_Pin GPIO_PIN_8
#define S15i2_GPIO_Port GPIOA
#define S09x2_Pin GPIO_PIN_9
#define S09x2_GPIO_Port GPIOA
#define S18_Pin GPIO_PIN_10
#define S18_GPIO_Port GPIOA
#define S17_Pin GPIO_PIN_11
#define S17_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define S19i3_Pin GPIO_PIN_15
#define S19i3_GPIO_Port GPIOA
#define FTDI_TX_Pin GPIO_PIN_10
#define FTDI_TX_GPIO_Port GPIOC
#define FTDI_RX_Pin GPIO_PIN_11
#define FTDI_RX_GPIO_Port GPIOC
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define S14x3_Pin GPIO_PIN_3
#define S14x3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define S13x5_Pin GPIO_PIN_5
#define S13x5_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SCA_Pin GPIO_PIN_7
#define I2C_SCA_GPIO_Port GPIOB
#define S16_Pin GPIO_PIN_8
#define S16_GPIO_Port GPIOB
#define S12_Pin GPIO_PIN_9
#define S12_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

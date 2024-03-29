/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

// WIRING INPUT
// SERIAL3 = ------ LiD (UART3, RX only, DMA RX Normal, note : pin TX damaged)
// SERIAL4 = TX&RX from NVIDIA AI (UART4, DMA RX Circular, DMA TX normal)
// SERIAL5 = ------ LiG (UART5, RX only, DMA RX Normal)

// RC1 = n/c
// RC2 = DIR from RX (TIM3 CH1 PWM input capture)
// RC3 = Tachymeter  (TIM4 CH1 PWM input capture)
// RC4 = THR from RX (TIM12 CH1 PWM input capture)
// RC5 = n/c

// IO1 : ------ LiH (UART2, RX only, DMA RX normal)
// IO4 : START Push button (PC4)

// WIRING OUTPUT
// SERVO5 (HW=TIM8 CH1) = THR
// SERVO6 (HW=TIM8 CH2) = DIR

// MMI
// LED0 (HW=D2)
// LED1 (HW=D1) = MANUAL/AUTO state
// LED2 (HW=D3)
// LED3 (HW=D4) = GYRO state
// LED4 (HW=D5) = AI state
// LED5 (HW=D6) = RC state



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "serial.h"
#include "tfminiplus.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>     /* atoi */
#include "imu.h"
#include "timer_us.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
static HAL_Serial_Handler ai_com;
static uint32_t RC1_last_time = 0;
static uint32_t RC1_period = 0;
static uint32_t RC1_duty_cycle = 0;
static uint32_t RC2_last_time = 0;
static uint32_t RC2_period = 0;
static uint32_t RC2_duty_cycle = 0;
static uint32_t RC4_last_time = 0;
static uint32_t RC4_period = 0;
static uint32_t RC4_duty_cycle = 0;
static char com_line[32];
static uint32_t com_position = 0;
static uint32_t com_last_time = 0;
static uint32_t pwm_manual_thr = 1500;
static uint32_t pwm_manual_dir = 1500;
static uint32_t pwm_ai_thr = 1500;
static uint32_t pwm_ai_dir = 1500;
static uint32_t ai_mode = 0; // 0:idle, 1:running
static uint32_t pwm_auto_thr = 1500;
static uint32_t pwm_auto_dir = 1500;
enum {MAIN_STATE_MANUAL, MAIN_STATE_AUTO_REQUEST, MAIN_STATE_AUTO_STARTUP, MAIN_STATE_AUTO };
static uint32_t main_state = MAIN_STATE_MANUAL;
static uint32_t main_state_last = 0;
#define MANUAL_OVERRIDE_TIMEOUT 2000 //ms
enum {RC_STATE_NONE,RC_STATE_OK};
static uint32_t rc_state = RC_STATE_NONE;
#define RC_TIMEOUT 500 //ms
enum {AI_STATE_NONE,AI_STATE_OK};
static uint32_t ai_state = AI_STATE_NONE;
#define AI_TIMEOUT 100 //ms
enum {LIDAR_STATE_NONE,LIDAR_STATE_OK};
//static uint32_t lidar_state = LIDAR_STATE_NONE;
#define LIDAR_TIMEOUT 100 //ms
//static int32_t lidar_distance_gauche = -1; // cm
//static int32_t lidar_distance_droit = -1;
//static int32_t lidar_distance_haut = -1;
//static int32_t lidar_strength_gauche = -1;
//static int32_t lidar_strength_droit = -1;
//static int32_t lidar_strength_haut = -1;
//static int32_t lidar_temp_gauche = -1;
//static int32_t lidar_temp_droit = -1;
//static int32_t lidar_temp_haut = -1;
static uint32_t tachymeter_pulse_period_5us = 0xFFFF;
static uint64_t tachymeter_pulse_count = 0;
#define GYRO_PERIOD_US 2404 // GYRO ODR = 416Hz
#define GYRO_ODR 416 // GYRO ODR = 416Hz
static int32_t start_countdown = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==START_BUTTON_Pin)
	{
		main_state = MAIN_STATE_AUTO;
		start_countdown = 3;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // Callback for PWM input catpure
{
	if(htim==&htim3) // RC2 = DIR from RX
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			RC2_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			RC2_last_time = HAL_GetTick(); // timestamp last pulse
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			RC2_duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if(htim==&htim12) // RC4 = THR from RX
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			RC1_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			RC1_last_time = HAL_GetTick(); // timestamp last pulse
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			RC1_duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if(htim==&htim4) // RC3 = Tachymeter
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			tachymeter_pulse_count++;
			RC4_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			RC4_last_time = HAL_GetTick(); // timestamp last pulse
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			RC4_duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
}
uint32_t int_to_pwm(uint32_t value)
{
	if(value>=255) value = 255;
	return (uint32_t)( (float)value * 1000.0 / 255.0 + 1000.0 );
}
uint32_t pwm_to_int(uint32_t value)
{
	if(value<=1000) value = 1000;
	if(value>=2000) value = 2000;
	return (uint32_t)( ( (float)value - 1000.0) * 255.0 / 1000.0 );
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  timer_us_init();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); // Start PWM outputs
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500); // Set PWM outputs (middle position)
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500);
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,1500);
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1500);
  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,1500);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1); // Start PWM inputs (capture)
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim12,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
  HAL_Serial_Init(&huart4,&ai_com); // Start com port
  HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET); // Init LEDs
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET); // Init LEDs
  HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);
  uint32_t gyro_err = gyro_init();
  uint16_t gyro_last_time_us = timer_us_get();
  HAL_Serial_Print(&ai_com, "gyro error:%d\r\n",gyro_err);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_time_100Hz = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t current_time_ms = HAL_GetTick();
		uint16_t current_time_us = timer_us_get();
		switch(rc_state) // RC state machine
		{
		case RC_STATE_NONE: // No RC
			{
				HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET);
				pwm_manual_thr = 1500;
				pwm_manual_dir = 1500;
			    if((RC1_last_time+RC_TIMEOUT>current_time_ms) && (RC2_last_time+RC_TIMEOUT>current_time_ms))
			    {
			    	rc_state = RC_STATE_OK;
			    }
			}
			break;
		case RC_STATE_OK:
			{
				HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET);
				pwm_manual_thr = RC1_duty_cycle;
				pwm_manual_dir = RC2_duty_cycle;
			    if((RC1_last_time+RC_TIMEOUT<current_time_ms) || (RC2_last_time+RC_TIMEOUT<current_time_ms))
			    {
			    	rc_state = RC_STATE_NONE;
			    }
			}
			break;
		}
	    while(HAL_Serial_Available(&ai_com)) // Process ai_com port
		{
	    	char c = HAL_Serial_GetChar(&ai_com);
			if(c=='\n' || c=='\r') // eol detection
			{
				if(com_position!=0) // not empty recv buffer
				{
					com_line[com_position]=0; // force eol
				    char *tab_args[50];
				    int num_args;
				    int32_t data;

			        num_args = 0;
			        tab_args[num_args] = strtok(com_line, ";");
			        while(tab_args[num_args] != NULL)
			        {
			            tab_args[++num_args] = strtok(NULL, ";");
			        }
			        // reset inputs data
					pwm_ai_dir = 1500;
					pwm_ai_thr = 1500;
					ai_mode = 0;
					// parse DIRECTION
			        if(num_args>=1)
			        {
						data = 128; // default not line position value from AI
						data = atoi(tab_args[0]); // decode value
						if(data<=255 && data>=0)
						{
							pwm_ai_dir = int_to_pwm(data);
						}
			        }
			        // parse THROTTLE
			        if(num_args>=2)
			        {
						data = 128; // default not speed value from AI
						data = atoi(tab_args[1]); // decode value
						if(data<=255 && data>=0)
						{
							pwm_ai_thr = int_to_pwm(data);
						}
			        }
			        // parse MODE
			        if(num_args>=3)
			        {
						data = 128; // default not speed value from AI
						data = atoi(tab_args[2]); // decode value
						ai_mode = data;
						// AI frame complete, update AI health state and send back telemetry frame
						com_last_time = current_time_ms;
			        }
					com_position = 0; // reset recv buffer
				}
			}
			else // new char
			{
				com_line[com_position]=c;
				if(com_position<31) // handle end of recv buffer
				{
					++com_position;
				}
			}
		}
		switch(ai_state) // AI state machine
		{
		case AI_STATE_NONE: // No RC
			{
				HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
				pwm_ai_thr = 1500;
				pwm_ai_dir = 1500;
			    if(com_last_time+AI_TIMEOUT>current_time_ms)
			    {
			    	ai_state = AI_STATE_OK;
			    }
			}
			break;
		case AI_STATE_OK:
			{
				HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
				// values from ai_com process
			    if(com_last_time+AI_TIMEOUT<current_time_ms)
			    {
			    	ai_state = AI_STATE_NONE;
			    }
			}
			break;
		}
		pwm_auto_thr = pwm_ai_thr; // no other source of automatic control, then auto controller use AI
		pwm_auto_dir = pwm_ai_dir;
		// compute actual x speed
		//   RC4 max period is 325ms (0xffff * 5us)
		//   no pulse after 650ms means null speed (return 0xffff)
		if(RC4_last_time + 650 > HAL_GetTick())
			tachymeter_pulse_period_5us = RC4_period;
		else
			tachymeter_pulse_period_5us = 0xFFFF;
		// compute actual distance at 416Hz
		//    see TIMER IT Callback
		// compute actual heading and actual w speed using gyro
		uint16_t duration_us = current_time_us-gyro_last_time_us;
		if(duration_us>=GYRO_PERIOD_US)
		{
			gyro_last_time_us += GYRO_PERIOD_US;
			if( gyro_err == GYRO_OK)
			{
				HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET);
				float duration_s = (float)(duration_us)/1000000.0;
				if(tachymeter_pulse_period_5us == 0xFFFF)
				{
					// robot is idle, auto calibrate bias
					gyro_auto_calibrate(duration_s);
#ifdef IMU_TRACE
					static int counter = 0;
					if(((counter++)%GYRO_ODR)==0) // one trace per second
						HAL_Serial_Print(&ai_com,"duration_us=%d rate=%d mean=%d variance=%d bias=%d heading=%d dps=%d\r\n",
							duration_us,
							(int32_t)(gyro_get_rate()*1000.0),
							(int32_t)(gyro_get_mean()*1000.0),
							(int32_t)(gyro_get_variance()*1000.0),
							(int32_t)(gyro_get_bias()*1000.0),
							(int32_t)(gyro_get_heading()),
							(int32_t)(gyro_get_dps()*1000.0)
						);
#endif
				}
				else
				{
					// robot is running
					gyro_update(duration_s);
				}
			}
			else
			{
				HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
			}
		}
		// 100Hz communication to AI
			// return distance (total) ==> AI compute v
			// return heading (total) ==> AI compute w and dtheta
			// return current instant v,w (sampling by AI)
		uint32_t duration_ms = current_time_ms-last_time_100Hz;
		if(duration_ms>=10)
		{
			last_time_100Hz+=10;
			// query last lidar values
			//tfminiplus_getLastAcquisition(MINILIDAR_GAUCHE, &lidar_distance_gauche, &lidar_strength_gauche, &lidar_temp_gauche);
			//tfminiplus_getLastAcquisition(MINILIDAR_DROIT, &lidar_distance_droit, &lidar_strength_droit, &lidar_temp_droit);
			//tfminiplus_getLastAcquisition(MINILIDAR_HAUT, &lidar_distance_haut, &lidar_strength_haut, &lidar_temp_haut);

			// STM32 replies to AI by a frame that contains informations :
			//  - all PWM values [0..255] for DIR/THR from RC and from AI
			//  - modes AUTO/MANUAL from RC and from AI
			//  - start countdown
			//  - actual absolute/total distance in pulses (1/4 main gear rotation)
			//  - actual heading in 0.001 degrees
			//  - actual instant x speed in 5us period of last pulses (1/4 main gear rotation)
			//  - actual instant w speed in 0.001 dps
			//  - actual gyro bias in 0.001 dps
			static uint32_t rep_counter = 0;
			HAL_Serial_Print(&ai_com, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
				rep_counter++,
				pwm_to_int(pwm_manual_dir),
				pwm_to_int(pwm_manual_thr),
				pwm_to_int(pwm_auto_dir),
				pwm_to_int(pwm_auto_thr),
				main_state,
				ai_mode,
				start_countdown,
				(uint32_t)tachymeter_pulse_count,
				(int32_t)(gyro_get_heading()*1000.0),
				tachymeter_pulse_period_5us,
				(int32_t)(gyro_get_dps()*1000.0),
				(int32_t)(gyro_get_bias()*1000.0)
			);
			if(start_countdown>0)
				--start_countdown;
		}
		switch(main_state) // MAIN state machine
		{
		case MAIN_STATE_MANUAL:
			{
				// RC control servo
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_manual_thr); // RC always control THR at the moment
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,pwm_manual_dir); // RC control DIR
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1500);// default servo position
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,1500);

				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);

				if( (pwm_manual_dir > 1850) && (pwm_manual_thr > 1450) && (pwm_manual_thr < 1550) ) // activation condition
				{
					main_state_last = current_time_ms;
					main_state = MAIN_STATE_AUTO_REQUEST;
				}
			}
			break;
		case MAIN_STATE_AUTO_REQUEST:
			{
				// RC control servo
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_manual_thr); // RC always control THR at the moment
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,pwm_manual_dir); // RC control DIR
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1500);// default servo position
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,1500);

				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);

				if( (pwm_manual_dir > 1850) && (pwm_manual_thr > 1450) && (pwm_manual_thr < 1550) ) // activation condition
				{
					if(main_state_last+MANUAL_OVERRIDE_TIMEOUT<current_time_ms)
					{
						main_state = MAIN_STATE_AUTO_STARTUP;
					}
				}
				else
				{
					main_state = MAIN_STATE_MANUAL;
				}
			}
			break;
		case MAIN_STATE_AUTO_STARTUP:
			{
				// RC control servo
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_manual_thr); // RC always control THR at the moment
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,pwm_manual_dir); // RC control DIR
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1500);// default servo position
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,1500);

				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);

				if( (pwm_manual_dir > 1450) && (pwm_manual_dir < 1550) && (pwm_manual_thr > 1450) && (pwm_manual_thr < 1550) ) // activation condition
				{
						main_state = MAIN_STATE_AUTO;
				}
			}
			break;
		case MAIN_STATE_AUTO:
			{
				if(ai_mode==1)
				{
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,pwm_auto_thr); // AI control THR
						__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,pwm_auto_dir); // AI control DIR
				}
				else
				{
					__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1500); // default servo position
					__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,1500); // default servo position
				}
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,1500); // default servo position
				__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,1500);

				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,1500);
				__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,1500);

				HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);

				if( (pwm_manual_dir > 1650) || (pwm_manual_dir < 1350) || (pwm_manual_thr < 1350) ) // go back to MANUAL mode when DIR stick touched, when THR stick on brake/backward position
				{
					main_state = MAIN_STATE_MANUAL;
				}
			}
			break;
		}
		//HAL_Delay(1);
		timer_us_delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 3;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 3;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 167;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 167;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0xffff;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0xffff;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 3;
  if (HAL_TIM_SlaveConfigSynchro(&htim12, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED0_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED5_Pin */
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED0_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED0_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

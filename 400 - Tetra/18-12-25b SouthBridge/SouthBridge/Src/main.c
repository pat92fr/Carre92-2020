/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
//#define __Print_All_Raw_Sensor__
//#define __Print_All_Scaled_Sensor__
//#define __Print_AHRS_Orientation__
//#define __Print_MyAHRS_Orientation__
//#define __Print_MotionCal__
#define __Print_PID_Motor__

#include "battery.h"
#include "led.h"
#include "ppm_sum_input.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "serial.h"
#include "mymath.h"
#include "imu.h"
#include "LSM6DS33.h"
#include "LIS3MDL.h"
#include <math.h>
#include "myAHRS.h"
#include "speed.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
	RUNNING = 0,
	RUNNING_AND_RX,
	SHUTDOWN,
	POWER_STATUS_COUNT
} POWER_STATUS;

char * const power_status_led_sequence[] = {
		".             ", 	//!RUNNING
		"__        ", 		//!RUNNING_AND_RX
		". . . . . . . ", 	//!SHUTDOWN
};

POWER_STATUS power_status = RUNNING;
uint32_t rx_timestamp = 0;

HAL_LED_HandleTypeDef hled;

HAL_PPM_SUM_Input_HandleTypeDef hppmi;

HAL_Encoder_HandleTypeDef hencoder;

double xpid_kp = 4000.0; // 1000.0
double xpid_ki = 100.0; // 2.0
double xpid_kd = 0.0;
pid_win_handler xpid = {
		&xpid_kp,
		&xpid_ki,
		&xpid_kd,
	    -1000.0f,
		1000.0f,
	    100,
	    0.5f
};
double wpid_kp = 20.0;
double wpid_ki = 0.4;
double wpid_kd = 0.0;
pid_win_handler wpid = {
		&wpid_kp,
		&wpid_ki,
		&wpid_kd,
	    -1000.0f,
		1000.0f,
	    100,
	    0.5f
};

HAL_Serial_Handler hcom;

IMU_HandleTypeDef imu;
static float const imu_sensor_offset[SENSOR_COUNT] = { 41.0f, -677.0f, -146.0f, 0.0f, 0.0f, 0.0f, 357.0f, -1296.0f, 5681.0f };
// first 3 value are provided by gyro calibration (bias)
// last 3 values are provided by hard iron calibration
static float const imu_sensor_scale_factor[SENSOR_COUNT] = {
		0.00875f*0.99f, 	// 8.75 mdps/LSB * sensitivity correction with turn table
		0.00875f*0.99f, 	// 8.75 mdps/LSB * sensitivity correction with turn table
		0.00875f*0.99f, 	// 8.75 mdps/LSB * sensitivity correction with turn table
		0.000061f, 		// 0,061 mg/LSB
		0.000061f, 		// 0,061 mg/LSB
		0.000061f, 		// 0,061 mg/LSB
		0.000146f, // 68xx LSB/gauss
		0.000146f,
		0.000146f
};
static float const imu_mag_softiron_matrix[3][3] = {
		{  1.0,  0.0,  0.0 },
        {  0.0,  1.0,  0.0 },
        {  0.0,  0.0,  1.0 } };	// all values from MotionCal
static HAL_Imu_ConfigurationElementTypeDef const imu_sensor_config[] = {
		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_WHO_AM_I, IMU_LSM6DS33_WHO_AM_I, IMU_READ_CHECK }, //! check sensor communication

		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL1_XL,	0x50, IMU_WRITE_AND_CHECK},
		// write 0101 0000 = 0x50
		// [7-4]: ODR_XL [3:0] = 0101b - ODR = 208 Hz
		// [3-2]: FS_XL [1:0] = 00b - Scale = ±2 g
		// [1-0]: BW_XL [1:0] = 00b - Filter = 400 Hz

		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL2_G,	0x50, IMU_WRITE_AND_CHECK},
		// write 0101 0000= 0x50
		// [7-4]: ODR_G [3:0] = 0101b - ODR = 208 Hz
		// [3-2]: FS_G [1:0] = 00b - Scale =  250 dps
		// [1]: FS_125 = 0b - Disable Gyroscope full-scale at 125 dps
		// [0]: unused

		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL3_C,	0x46, IMU_WRITE_AND_CHECK},
		// write 0100 0110= 0x46
		// [7]: BOOT = 0
		// [6]: BDU = 1 - output registers not updated until MSB and LSB have been read
		// [5]: H_LACTIVE = 0
		// [4]: PP_OD = 0
		// [3]: SIM = 0
		// [2]: IF_INC = 1 - Register address automatically incremented during a multiple byte access
		// [1]: BLE = 1 -  data MSB @ lower address
		// [0]: SW_RESET = 0

		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL9_XL,	0x38, IMU_WRITE_AND_CHECK},
		// write 0011 1000= 0x38
		// [7]: unused
		// [6]: unused
		// [5]: Zen_XL = 1 - Accelerometer Z-axis output enable
		// [4]: Yen_XL = 1 - Accelerometer Y-axis output enable
		// [3]: Xen_XL = 1 - Accelerometer X-axis output enable
		// [2]: unused
		// [1]: unused
		// [0]: unused

		{ IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_REGISTER_CTRL10_C,	0x38, IMU_WRITE_AND_CHECK},
		// write 0011 1000= 0x38
		// [7]: unused
		// [6]: unused
		// [5]: Zen_G = 1 - Gyroscope yaw axis (Z) output enable
		// [4]: Yen_G = 1 - Gyroscope roll axis (Y) output enable
		// [3]: Xen_G = 1 - Gyroscope pitch axis (X) output enable
		// [2]: FUNC_EN = 0
		// [1]: PEDO_RST_STEP = 0
		// [0]: SIGN_MOTION_EN = 0

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_WHO_AM_I, IMU_LIS3MDL_WHO_AM_I, IMU_READ_CHECK }, //! check sensor communication

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_CTRL1,	0x50, IMU_WRITE_AND_CHECK},
		// write 0101 0000= 0x50
		// [7]: TEMP_EN = 0
		// [6-5]: OM[1:0] = 10b - High-performance mode
		// [4-2]: DO[2:0] = 100b - 10Hz
		// [1]: FAST_ODR = 0
		// [0]: ST = 0

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_CTRL2,	0x00, IMU_WRITE_AND_CHECK},
		// write 0000 0000= 0x00
		// [7]: unused
		// [6-5]: FS[1:0] = 00b - ±4 gauss
		// [4]: unused
		// [3]: REBOOT = 0b
		// [2]: SOFT_RST = 0b
		// [1]: unused
		// [0]: unused

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_CTRL3,	0x00, IMU_WRITE_AND_CHECK},
		// write 0000 0000= 0x03
		// [7]: unused
		// [6]: unused
		// [5]: LP = 0b
		// [4]: unused
		// [3]: unused
		// [2]: SIM = 0b
		// [1-0]: MD[1:0] = 00b -  Continuous-conversion mode

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_CTRL4,	0x0A, IMU_WRITE_AND_CHECK},
		// write 0000 1010 = 0x0A
		// [7]: unused
		// [6]: unused
		// [5]: unused
		// [4]: unused
		// [3-2]: OMZ[1:0] = 10b - High-performance mode
		// [1]: BLE = 1b - data MSb at lower address
		// [0]: unused

		{ IMU_LIS3MDL_ADDRESS, IMU_LIS3MDL_REGISTER_CTRL5,	0x40, IMU_WRITE_AND_CHECK},
		// write 0100 0000= 0x40
		// [7]: FAST_READ = 0b -  FAST_READ disabled
		// [6]: BDU = 1b -  output registers not updated until MSb and LSb have been read
		// [5]: unused
		// [4]: unused
		// [3]: unused
		// [2]: unused
		// [1]: unused
		// [0]: unused

		{0,0,0,0} //! end of config
};
uint16_t imu_sensor_device_addr[3] = { IMU_LSM6DS33_ADDRESS, IMU_LSM6DS33_ADDRESS, IMU_LIS3MDL_ADDRESS };
uint16_t imu_sensor_reg_addr[3] = { IMU_LSM6DS33_REGISTER_OUTX_L_G, IMU_LSM6DS33_REGISTER_OUTX_L_XL, IMU_LIS3MDL_REGISTER_OUT_X_L };

AHRS_HandleTypeDef ahrs;

float x = 0.0f;
float y = 0.0f;

typedef struct
{
	int32_t cmd;
	int32_t x;
	int32_t y;
	int32_t w;
	int32_t chksum;
} POD_Command;

uint32_t const command_fifo_size = 32;
POD_Command command_fifo[32];
uint32_t command_fifo_input = 0;
uint32_t command_fifo_current = 0;
POD_Command current_command = {0};

uint32_t current_command_step = 0;
float start_x = 0.0f;
float start_y = 0.0f;
float start_heading = 0.0f;
float const xspeed_max = 0.15f; 	// m.s
float const xacc_max = 0.10f; 		//m.s2
float const wspeed_max = 20.0f;	// dps
float const wacc_max = 30.0f;	// dps/s
float xspeed_current = 0.0f;
float xspeed_target = 0.0f;
float wspeed_current = 0.0f;
float wspeed_target = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HAL_PPM_SUM_Input_ISR(htim);
}

void HAL_SYSTICK_Callback(void)
{

}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_Battery_Init();
  HAL_Led_Init();
  HAL_Led_Add(&hled,LED_GPIO_Port,LED_Pin);
  HAL_Led_Set(&hled);
  HAL_PPM_SUM_Input_Init(&hppmi,&htim3);
  HAL_Encoder_Init(&hencoder,&htim5,&htim2);
  // RIGHT MOTOR <=> M1 PORT <=> TIM5, CW > positive counter
  // LEFT MOTOR <=> M2 PORT <=> TIM2, CW > positive counter
  HAL_Motor_Init();
  HAL_Motor_Set(HAL_MOTOR_ALL,HAL_MOTOR_AUTO,0);
  reset_pid_win(&xpid);
  reset_pid_win(&wpid);
  HAL_Serial_Init(&huart2,&hcom);
  HAL_Serial_Print(&hcom,"\n\n\nTETRA - SouthBridge v0.00\n");
  HAL_IMU_Init();
  HAL_IMU_Add(
		&imu,
		&hi2c1,
		imu_sensor_offset,
		imu_sensor_scale_factor,
		imu_mag_softiron_matrix,
		imu_sensor_config,
		imu_sensor_device_addr,
		imu_sensor_reg_addr
	);
	HAL_AHRS_Init(&ahrs,0.005,0.01);

  //uint32_t last_time = HAL_GetTick();
  HAL_TIM_Base_Start(&htim14);
  uint16_t last_time_us = __HAL_TIM_GET_COUNTER(&htim14);

  HAL_Led_Sequence(&hled,".         ",true);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //! Power and LED state
	  {
		  float power_level = HAL_Battery_Get(VBATT);
		  switch(power_status)
		  {
			case RUNNING:
				{
					if(power_level<(3.0*3.6))
					{
						power_status = SHUTDOWN;
						HAL_Led_Sequence(&hled,power_status_led_sequence[SHUTDOWN],true);
					}
					if(HAL_GetTick()<(rx_timestamp+1000))
					{
						power_status = RUNNING_AND_RX;
						HAL_Led_Sequence(&hled,power_status_led_sequence[RUNNING_AND_RX],true);
					}
				}
				break;
			case RUNNING_AND_RX:
				{
					if(power_level<(3.0*3.6))
					{
						power_status = SHUTDOWN;
						HAL_Led_Sequence(&hled,power_status_led_sequence[SHUTDOWN],true);
					}
					if(HAL_GetTick()>(rx_timestamp+1000))
					{
						power_status = RUNNING;
						HAL_Led_Sequence(&hled,power_status_led_sequence[RUNNING],true);
					}
				}
				break;
			case SHUTDOWN:
			default:
				{
					if(power_level>(3.0*3.8))
					{
						power_status = RUNNING;
						HAL_Led_Sequence(&hled,power_status_led_sequence[RUNNING],true);
					}
				}
				break;
		  }
		  HAL_Led_Process();
	  }

	  // COMMENT when using OpenMV
//	  // simple protocol command over the air
//	  enum
//	  {
//		  RC_WAITING = 0,
//		  RC_SOM,
//		  RC_CMD,
//		  RC_X,
//		  RC_Y,
//		  RC_W
//	  };
//	  static uint32_t remote_control_state = RC_WAITING;
//	  POD_Command remote_control;
//	  if(HAL_Serial_Available(&hcom))
//	  {
//		  uint8_t c = HAL_Serial_GetChar(&hcom);
//		  switch(remote_control_state)
//		  {
//		  case RC_WAITING:
//			  {
//				  if(c==255)
//				  {
//					  remote_control_state=RC_SOM;
//				  }
//			  }
//			  break;
//		  case RC_SOM:
//			  {
//				  remote_control.cmd = c;
//				  remote_control.chksum = c;
//				  remote_control_state=RC_CMD;
//			  }
//			  break;
//		  case RC_CMD:
//			  {
//				  remote_control.x = c;
//				  remote_control.chksum += c;
//				  remote_control_state=RC_X;
//			  }
//			  break;
//		  case RC_X:
//			  {
//				  remote_control.y = c;
//				  remote_control.chksum += c;
//				  remote_control_state=RC_Y;
//			  }
//			  break;
//		  case RC_Y:
//			  {
//				  remote_control.w = c;
//				  remote_control.chksum += c;
//				  remote_control_state=RC_W;
//			  }
//			  break;
//		  case RC_W:
//			  {
//				  if(remote_control.chksum%256 == c)
//				  {
//					  // post command into FIFO
//					  command_fifo[command_fifo_input] = remote_control;
//					  ++command_fifo_input;
//					  command_fifo_input=command_fifo_input%command_fifo_size;
//					  //HAL_Serial_Print(&hcom,"Command %d %d %d %d\n",remote_control.cmd,remote_control.x,remote_control.y,remote_control.w);
//				  }
//				  remote_control_state=RC_WAITING;
//			  }
//			  break;
//		  }
//		  //HAL_Serial_Print(&hcom,"RxD:%c...\n",HAL_Serial_GetChar(&hcom));
//	  }

	  // 200Hz processing
	  uint16_t current_time_us = __HAL_TIM_GET_COUNTER(&htim14);
	  uint16_t delta_time_us = current_time_us-last_time_us;
	  if(delta_time_us>=4808) //! 208Hz (ODR)
	  {
		  // period
		  float period_us = (float)(delta_time_us)/1000000.0f; //s
		  last_time_us = current_time_us;

		  // AHRS
		  HAL_IMU_Read_Sensors(&imu);
		  HAL_AHRS_Process(
				  &ahrs,
				  period_us,
				imu.scaled_sensor_data[GYR_X],
				imu.scaled_sensor_data[GYR_Y],
				imu.scaled_sensor_data[GYR_Z],
				imu.scaled_sensor_data[ACC_X],
				imu.scaled_sensor_data[ACC_Y],
				imu.scaled_sensor_data[ACC_Z],
				imu.scaled_sensor_data[MAG_X],
				imu.scaled_sensor_data[MAG_Y],
				imu.scaled_sensor_data[MAG_Z]
	   );

		  // Remote control (from PPM SUM/RX)
		  int32_t xspeed_rc = HAL_PPM_SUM_Input_Get(&hppmi,1,&rx_timestamp);
		  int32_t wspeed_rc = HAL_PPM_SUM_Input_Get(&hppmi,2,&rx_timestamp);
		  if(xspeed_rc > 950 && xspeed_rc < 2050 && wspeed_rc > 950 && wspeed_rc < 2050)
		  {
			  // deadband
			  int32_t const rc_dead_band = 25;
			  if(xspeed_rc<1500-rc_dead_band)
			  {
				  xspeed_rc += rc_dead_band;
			  }
			  else if(xspeed_rc>1500+rc_dead_band)
			  {
				  xspeed_rc -= rc_dead_band;
			  }
			  else
			  {
				  xspeed_rc = 1500;
			  }
			  if(wspeed_rc<1500-rc_dead_band)
			  {
				  wspeed_rc += rc_dead_band;
			  }
			  else if(wspeed_rc>1500+rc_dead_band)
			  {
				  wspeed_rc -= rc_dead_band;
			  }
			  else
			  {
				  wspeed_rc = 1500;
			  }
		  }
		  else
		  {
			  xspeed_rc = 1500;
			  wspeed_rc = 1500;
		  }
		  float xspeed_rc_scaled = (xspeed_rc-1500)*0.4/500; // m/s
		  //float wspeed_rc_scaled = (wspeed_rc-1500)*60.0/500; // dps
		  float wspeed_rc_scaled = 0.0f;
		  // let cam control direction


		  // decode OpenMV serial
		  static char serial_string[128];
		  static uint32_t serial_index = 0;
		  static float wspeed_cam = 0;
		  if(HAL_Serial_Available(&hcom))
		  {
				// accumulate input chars
				uint8_t c = HAL_Serial_GetChar(&hcom);
				serial_string[serial_index++]=c;
				serial_index %= 128;

				// decode when EOL
				if(c=='\n')
				{
					serial_index = 0;
					wspeed_cam = atoi(serial_string);
				}
		  }
		  wspeed_rc_scaled = -(float)wspeed_cam / 2.0;

		  // Remonte control (from NETWORK)
		  switch(current_command.cmd)
		  {
	  	  case 0:
			  {
				  // let PPM SUM/RX control the robot

				  // reset command state
				  current_command.cmd = 0;
				  current_command_step = 0;

				  // reset speed
				  xspeed_current = 0.0f;
				  xspeed_target = 0.0f;
				  wspeed_current = 0.0f;
				  wspeed_target = 0.0f;

				  // new command
				  if(command_fifo_input!=command_fifo_current)
				  {
					  current_command = command_fifo[command_fifo_current];
					  ++command_fifo_current;
					  command_fifo_current = command_fifo_current%command_fifo_size;
				  }
				  else
				  {
					  current_command.cmd = 0;
				  }
			  }
			  break;
	  	  case 1: // forward
			  {
				  switch (current_command_step)
				  {
				  case 0:
					  {
						  // store initial position
						  start_x = x;
						  start_y = y;
						  start_heading = ahrs.yaw_from_gyro; // TODO ne pas reprendre le cap courant, mais le cap absolu lié à une série de commande

						  // set speed
						  xspeed_target = xspeed_max;
						  xspeed_current = 0.0f;
						  wspeed_target = 0.0f;
						  wspeed_current = 0.0f;

						  // reset control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = 0.0f;

						  current_command_step = 1;
					  }
					  break;
				  case 1:
					  {
						  // update speed
						  xspeed_target = xspeed_max;
						  next_speed(&xspeed_current,xspeed_target,xacc_max,xacc_max,period_us*1000.0f);
						  wspeed_target = 0.0f;
						  wspeed_current = 0.0f;

						  // update control
						  xspeed_rc_scaled = xspeed_current;
						  wspeed_rc_scaled = 0.0f;

						  	float distance_done = sqrt((x-start_x)*(x-start_x) + (y-start_y)*(y-start_y));
						  	float distance_remaining = current_command.x*0.01f-distance_done;

						  	if(accelaration_until_distance_left(xspeed_current,0.0f,distance_remaining)<=-xacc_max)
						  	{
								// update speed
								xspeed_target = 0.01;
								next_speed(&xspeed_current,xspeed_target,xacc_max,xacc_max,period_us*1000.0f);
								wspeed_target = 0.0f;
								wspeed_current = 0.0f;

							  // update control
							  xspeed_rc_scaled = xspeed_current;
							  wspeed_rc_scaled = 0.0f;

						  		current_command_step = 2;
						  	}
					  }
					  break;
				  case 2:
					  {
						  // update speed
						  xspeed_target = 0.01;
						  next_speed(&xspeed_current,xspeed_target,xacc_max,xacc_max,period_us*1000.0f);
						  wspeed_target = 0.0f;
						  wspeed_current = 0.0f;

						  // update control
						  xspeed_rc_scaled = xspeed_current;
						  wspeed_rc_scaled = 0.0f;

						  	float distance_done = sqrt((x-start_x)*(x-start_x) + (y-start_y)*(y-start_y));
						  	float distance_remaining = current_command.x*0.01f-distance_done;

						  	if(distance_remaining<=0)
						  	{
								// update speed
								xspeed_target = 0.0f;
								xspeed_current = 0.0f;
								wspeed_target = 0.0f;
								wspeed_current = 0.0f;

								// update control
								xspeed_rc_scaled = 0.0f;
								wspeed_rc_scaled = 0.0f;

								current_command_step = 3;
						  	}


					  }
					  break;
				  case 3:
					  {
						  // update speed
						  xspeed_target = 0.0f;
						  xspeed_current = 0.0f;
						  wspeed_target = 0.0f;
						  wspeed_current = 0.0f;

						  // update control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = 0.0f;

						  // reset command state
						  current_command.cmd = 0;
						  current_command_step = 0;

						  // new command
						  if(command_fifo_input!=command_fifo_current)
						  {
							  current_command = command_fifo[command_fifo_current];
							  ++command_fifo_current;
							  command_fifo_current = command_fifo_current%command_fifo_size;
						  }
						  else
						  {
							  current_command.cmd = 0;
						  }

					  }
					  break;
				  }
			  }
			  break;
	  	  case 3: // turn right
			  {
				  switch (current_command_step)
				  {
				  case 0:
					  {
						  // store initial position
						  start_x = x;
						  start_y = y;
						  start_heading = ahrs.yaw_from_gyro;

						  // set speed
						  xspeed_target = 0.0f;
						  xspeed_current = 0.0f;
						  wspeed_target = wspeed_max;
						  wspeed_current = 0.0f;

						  // reset control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = 0.0f;

						  current_command_step = 1;
					  }
					  break;
				  case 1:
					  {
						  // update speed
						  xspeed_target = 0.0f;
						  xspeed_current = 0.0f;
						  wspeed_target = -wspeed_max;
						  next_speed(&wspeed_current,wspeed_target,wacc_max,wacc_max,period_us*1000.0f);

						  // update control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = wspeed_current;

						  float end_heading = start_heading-current_command.w;
						  if(end_heading>180.0) end_heading-=360.0f;
						  if(end_heading<-180.0) end_heading+=360.0f;
						  float current_heading = ahrs.yaw_from_gyro;
						  if(fabs(end_heading-current_heading)>180.0) current_heading+=360;
						  float angle_remaining = end_heading-current_heading;

						  	if(accelaration_until_angle_left(fabs(wspeed_current),0.0f,fabs(angle_remaining))<=-wacc_max)
						  	{
								// update speed
						  		xspeed_target = 0.0f;
						  		xspeed_current = 0.0f;
								wspeed_target = -2.0f;
								next_speed(&wspeed_current,wspeed_target,wacc_max,wacc_max,period_us*1000.0f);

								// update control
								xspeed_rc_scaled = 0.0f;
								wspeed_rc_scaled = wspeed_current;

						  		current_command_step = 2;
						  	}

					  }
					  break;
				  case 2:
					  {
						  // update speed
						  xspeed_target = 0.0f;
						  xspeed_current = 0.0f;
						  wspeed_target = -2.0f;
						  next_speed(&wspeed_current,wspeed_target,wacc_max,wacc_max,period_us*1000.0f);

						  // update control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = wspeed_current;

						  float end_heading = start_heading-current_command.w;
						  if(end_heading>180.0) end_heading-=360.0f;
						  if(end_heading<-180.0) end_heading+=360.0f;
						  float current_heading = ahrs.yaw_from_gyro;
						  if(fabs(end_heading-current_heading)>180.0) current_heading+=360;
						  float angle_remaining =end_heading-current_heading;

						  if(angle_remaining>=0)
						  	{
						  		// update speed
						  		xspeed_target = 0.0f;
						  		xspeed_current = 0.0f;
						  		wspeed_target = 0.0f;
						  		wspeed_current = 0.0f;

						  		// update control
							  xspeed_rc_scaled = 0.0f;
							  wspeed_rc_scaled = 0.0f;

						  		current_command_step = 3;
						  	}
					  }
					  break;
				  case 3:
					  {
							// update speed
							xspeed_target = 0.0f;
							xspeed_current = 0.0f;
							wspeed_target = 0.0f;
							wspeed_current = 0.0f;

						  // update control
						  xspeed_rc_scaled = 0.0f;
						  wspeed_rc_scaled = 0.0f;

						  // reset command state
						  current_command.cmd = 0;
						  current_command_step = 0;

						  // new command
						  if(command_fifo_input!=command_fifo_current)
						  {
							  current_command = command_fifo[command_fifo_current];
							  ++command_fifo_current;
							  command_fifo_current = command_fifo_current%command_fifo_size;
						  }
						  else
						  {
							  current_command.cmd = 0;
						  }

					  }
					  break;
				  }
			  }
			  break;

	  	  case 11: // forward continue
			  {
				  switch (current_command_step)
				  {
				  case 0:
					  {
						  start_x = x;
						  start_y = y;

						  xspeed_rc_scaled = 0.1f;
						  wspeed_rc_scaled = 0.0f;

						  current_command_step = 1;
					  }
					  break;
				  case 1:
					  {
						  xspeed_rc_scaled = 0.1f;
						  wspeed_rc_scaled = 0.0f;

							float distance = sqrt((x-start_x)*(x-start_x) + (y-start_y)*(y-start_y));
							if(distance>=current_command.x*0.01)
							{
								xspeed_rc_scaled = 0.0f;
								wspeed_rc_scaled = 0.0f;

								current_command_step = 2;
							}
					  }
					  break;
				  case 2:
					  {
						xspeed_rc_scaled = 0.1f;
						wspeed_rc_scaled = 0.0f;

						  // reset command state
						  current_command.cmd = 0;
						  current_command_step = 0;

						  // new command
						  if(command_fifo_input!=command_fifo_current)
						  {
							  current_command = command_fifo[command_fifo_current];
							  ++command_fifo_current;
							  command_fifo_current = command_fifo_current%command_fifo_size;
						  }
						  else
						  {
							  current_command.cmd = 0;
						  }

					  }
					  break;
				  }
			  }
			  break;
		  case 13: // right curve
			  {
				  switch (current_command_step)
				  {
				  case 0:
					  {
						  start_x = x;
						  start_y = y;
						  start_heading = ahrs.yaw_from_gyro;

						  xspeed_rc_scaled = 0.1f;
						  wspeed_rc_scaled = -20.0f;

						  current_command_step = 1;
					  }
					  break;
				  case 1:
					  {
						  xspeed_rc_scaled = 0.1f;
						  wspeed_rc_scaled = -20.0f;

						  float end_heading = start_heading-current_command.w;
						  if(end_heading>180.0) end_heading-=360.0f;
						  if(end_heading<-180.0) end_heading+=360.0f;
						  float current_heading = ahrs.yaw_from_gyro;
						  if(fabs(end_heading-current_heading)>180.0) current_heading+=360;

							if(current_heading<=end_heading)
							{
								xspeed_rc_scaled = 0.0f;
								wspeed_rc_scaled = 0.0f;

								current_command_step = 2;
							}
					  }
					  break;
				  case 2:
					  {
						xspeed_rc_scaled = 0.1f;
						wspeed_rc_scaled = 0.0f;

						  // reset command state
						  current_command.cmd = 0;
						  current_command_step = 0;

						  // new command
						  if(command_fifo_input!=command_fifo_current)
						  {
							  current_command = command_fifo[command_fifo_current];
							  ++command_fifo_current;
							  command_fifo_current = command_fifo_current%command_fifo_size;
						  }
						  else
						  {
							  current_command.cmd = 0;
						  }

					  }
					  break;
				  }
			  }
			  break;
	  	  default:
			  {
				  // reset command state
				  current_command.cmd = 0;
				  current_command_step = 0;
			  }
			  break;



		  }

		  // Attitude PID motor controler
		  float xspeed_actual = 0.0f;
		  float wspeed_actual = 0.0f;
		  float xpwm = 0.0f;
		  float wpwm = 0.0f;
		  {
			  int32_t const left_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_LEFT);
			  int32_t const right_count = HAL_Encoder_Delta(&hencoder,HAL_ENCODER_RIGHT);

			  float const x_delta_count = (float)(left_count+right_count)/2.0f;
			  static double const distance_per_pulse = (PI_FLOAT*0.144f)/4480.0f; // 0.1mm per pulse , 4480 pulses per revolution, 0.45m per revolution
			  float const x_distance = (float)(x_delta_count)*distance_per_pulse; // m
			  xspeed_actual = x_distance/period_us;

			  float const w_delta_count = (float)(right_count-left_count);
			  float const w_distance = (float)(w_delta_count)*distance_per_pulse; // m
			  float const w_angle = ToDeg(fastAtan2(w_distance,0.30)); // degree, wheel base = 30cm
			  wspeed_actual = w_angle/period_us;

			  float const xerror = xspeed_rc_scaled-xspeed_actual;
			  float const werror = wspeed_rc_scaled-wspeed_actual;

			  xpwm = process_pid_win(&xpid,xerror);
			  wpwm = process_pid_win(&wpid,werror);

			  HAL_Motor_Set(HAL_MOTOR_LEFT,HAL_MOTOR_AUTO,(int32_t)(xpwm+wpwm));
			  HAL_Motor_Set(HAL_MOTOR_RIGHT,HAL_MOTOR_AUTO,(int32_t)(xpwm-wpwm));

			  // odometry
			  x += x_distance * cos(ToRad(ahrs.yaw_from_gyro));
			  y += x_distance * sin(ToRad(ahrs.yaw_from_gyro));
		  }

		  // debug
		  static uint32_t counter = 0;
		  if(((counter++)%4)== 0)
		  {
			  // Output for battery test
//			  HAL_Serial_Print(&hcom,"Bat:%dmV\n",
//					  (int32_t)(HAL_Battery_Get(VBATT)*1000.0)
//					);


#ifdef __Print_All_Raw_Sensor__
			  // Output ALL
			  HAL_Serial_Print(&hcom,"%d %d %d %d %d %d %d %d %d\n",
  					  imu.raw_sensor_data[ACC_X], imu.raw_sensor_data[ACC_Y], imu.raw_sensor_data[ACC_Z],
					  imu.raw_sensor_data[GYR_X], imu.raw_sensor_data[GYR_Y], imu.raw_sensor_data[GYR_Z],
  					  imu.raw_sensor_data[MAG_X], imu.raw_sensor_data[MAG_Y], imu.raw_sensor_data[MAG_Z]
					);
#endif

#ifdef __Print_All_Scaled_Sensor__
			  // Output ALL scaled
			  HAL_Serial_Print(&hcom,"%d %d %d %d %d %d %d %d %d %d %d %d\n",
					  	  (int32_t)(imu.scaled_sensor_data[ACC_X]*1000.0),
					  	  (int32_t)(imu.scaled_sensor_data[ACC_Y]*1000.0),
						  (int32_t)(imu.scaled_sensor_data[ACC_Z]*1000.0),
						  (int32_t)(imu.scaled_sensor_data[GYR_X]*1000.0),
						  (int32_t)(imu.scaled_sensor_data[GYR_Y]*1000.0),
							(int32_t)(imu.scaled_sensor_data[GYR_Z]*1000.0),
							(int32_t)(imu.scaled_sensor_data[MAG_X]*1000.0),
							(int32_t)(imu.scaled_sensor_data[MAG_Y]*1000.0),
							(int32_t)(imu.scaled_sensor_data[MAG_Z]*1000.0),

							(int32_t)(imu.gyro_rate_deviation.pitch*1000.0),
							(int32_t)(imu.gyro_rate_deviation.roll*1000.0),
							(int32_t)(imu.gyro_rate_deviation.yaw*1000.0)

								);
#endif

#ifdef __Print_MotionCal__
			  HAL_Serial_Print(&hcom,"Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r",
  					  imu.raw_sensor_data[ACC_X], imu.raw_sensor_data[ACC_Y], imu.raw_sensor_data[ACC_Z],
					  imu.raw_sensor_data[GYR_X], imu.raw_sensor_data[GYR_Y], imu.raw_sensor_data[GYR_Z],
			  					  imu.raw_sensor_data[MAG_X], imu.raw_sensor_data[MAG_Y], imu.raw_sensor_data[MAG_Z]
					);
#endif

#ifdef __Print_MyAHRS_Orientation__
				 HAL_Serial_Print(&hcom,"Orientation: %d %d %d %d %d %d %d %d %d %d %d\n",
							(int32_t)(ToDeg(ahrs.angles_filtered.pitch)*1000.0f),
							(int32_t)(ToDeg(ahrs.angles_filtered.roll)*1000.0f),
							(int32_t)(ToDeg(ahrs.angles_filtered.yaw)*1000.0f),
							(int32_t)(ahrs.yaw_from_gyro*1000.0f),
							(int32_t)(ahrs.gravity*1000.0f),
							(int32_t)(ahrs.magnetic_field*1000.0f),
							(int32_t)(imu.gyro_rate_mean.yaw*1000.0f),
							(int32_t)(imu.gyro_rate_deviation.yaw*1000.0f),
							(int32_t)(imu.sensor_offset[GYR_Z]*1000.0f),
							(int32_t)(ToDeg(ahrs.magnetic_dip_angle)*1000.0f),
							(int32_t)(period_us*1000000.0f)
						);
#endif

#ifdef __Print_AHRS_Orientation__
				 HAL_Serial_Print(&hcom,"Orientation: %d %d %d\n",
							(int32_t)(ahrs.Pitch*1000.0f),
							(int32_t)(ahrs.Roll*1000.0f),
							(int32_t)(ahrs.Yaw*1000.0f)
						);
#endif


#ifdef __Print_PID_Motor__
				 HAL_Serial_Print(&hcom,"%d %d %d %d %d %d %d %d %d %d\n",
						 (int32_t)(xspeed_rc_scaled*1000.0f),
						 (int32_t)(xspeed_actual*1000.0f),
						 (int32_t)(xpwm),
						 (int32_t)(wspeed_rc_scaled*1000.0f),
						 (int32_t)(wspeed_actual*1000.0f),
						 (int32_t)(wpwm),
						 (int32_t)(ahrs.gyro_rate_tilt_compensated.yaw*1000.0f),
						 (int32_t)(ahrs.yaw_from_gyro*1000.0f),
						 (int32_t)(x*1000.0f),
						 (int32_t)(y*1000.0f)
						);
#endif
		  }
	  }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim3, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 83;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0xFFFF;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|M2_INB_Pin|M2_INA_Pin|M1_INB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_INA_GPIO_Port, M1_INA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin M2_INB_Pin M2_INA_Pin M1_INB_Pin */
  GPIO_InitStruct.Pin = LED_Pin|M2_INB_Pin|M2_INA_Pin|M1_INB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_INA_Pin */
  GPIO_InitStruct.Pin = M1_INA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_INA_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

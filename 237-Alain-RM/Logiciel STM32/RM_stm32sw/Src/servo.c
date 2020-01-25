/*
 * servo.c
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "math.h"
#include "servo.h"

// pulse values for min, ANGLE_MIN, 0, ANGLE_MAX and max
// for registered servos
int s00_limits[5]={1200,1300,1500,1700,1800}; // test servo

int s01_limits[5]={ 600,1000,1485,2000,2100}; // DH
int s02_limits[5]={ 850, 950,1375,1900,2400}; // AH
int s03_limits[5]={ 950,1000,1450,1900,2450}; // CH
int s04_limits[5]={1200,1300,1500,1700,1800};
int s05_limits[5]={ 700,1150,1475,1900,2050}; // DM
int s06_limits[5]={1200,1300,1500,1700,1800};
int s07_limits[5]={1200,1300,1500,1700,1800};
int s08_limits[5]={ 550, 975,1410,1945,2045}; // BH
int s09_limits[5]={ 700,1250,1725,2250,2750}; // AM
int s10_limits[5]={ 650,1500,2000,2500,2650}; // AL
int s11_limits[5]={ 650,1150,1600,2050,2350}; // CM
int s12_limits[5]={ 550, 950,1350,1850,2450}; // BM
int s13_limits[5]={ 550,1300,1750,2200,2250}; // DL
int s14_limits[5]={1200,1300,1500,1700,1800};
int s15_limits[5]={1200,1300,1500,1700,1800};
int s16_limits[5]={ 550,1350,1750,2200,2350}; // CL
int s17_limits[5]={1200,1300,1500,1700,1800};
int s18_limits[5]={1200,1300,1500,1700,1800};
int s19_limits[5]={ 750,1500,2075,2550,2650}; // BL

/**********************************************************
  * @brief
  * @param
  * @param
  * @param
  * @retval
  */
void Servo_Init(t_servo *hservo, TIM_HandleTypeDef *htim, int timer_channel)
{
  hservo->htim=htim;
  hservo->timer_channel=timer_channel;
  hservo->pulse=SERVO_PULSE_MEAN;
}

/**********************************************************
  * @brief
  * @param
  * @param
  * @retval
  */
void Servo_SetLimits(t_servo *hservo, int limits[5])
{
  hservo->bmin_pulse=limits[0];
  hservo->amin_pulse=limits[1];
  hservo->zero_pulse=limits[2];
  hservo->amax_pulse=limits[3];
  hservo->bmax_pulse=limits[4];
  hservo->slope=((float)(limits[3]-limits[1]))/(SERVO_ANGLE_MAX-SERVO_ANGLE_MIN);
}

/**********************************************************
  * @brief  Start a servo
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_Start(t_servo *hservo)
{
  HAL_TIM_PWM_Start(hservo->htim, hservo->timer_channel);
}

/**********************************************************
  * @brief  Stop a servo
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_Stop(t_servo *hservo)
{
  HAL_TIM_PWM_Stop(hservo->htim,hservo->timer_channel);
}

/**********************************************************
  * @brief  Set servo position
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_SetPulse(t_servo *hservo, int pulse)
{
  hservo->pulse=pulse;
  __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, pulse);
}

/*
 *  @brief Set servo position from angle
 *  @param hservo : servo handler
 *  @param angle : angle, in radians
 */
void Servo_SetAngle(t_servo * hservo, float angle)
{
  int pulse;
  pulse=round(hservo->amin_pulse+(angle-SERVO_ANGLE_MIN)*hservo->slope);
  hservo->pulse=pulse;
  __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, pulse);
}

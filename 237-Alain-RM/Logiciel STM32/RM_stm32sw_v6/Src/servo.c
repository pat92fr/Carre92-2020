/*
 * servo.c
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "math.h"
#include "serial.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "main.h"

// pulse values for min, ANGLE_MIN, 0, ANGLE_MAX and max
// for registered servos
int s00_limits[5]={1200,1300,1500,1700,1800}; // test servo

int s01_limits[5]={ 600,1150,1600,2075,2100}; // FLH
int s02_limits[5]={ 850,1050,1550,2050,2400}; // FRH
int s03_limits[5]={ 950,1125,1550,2000,2450}; // RLH
int s04_limits[5]={1200,1300,1500,1700,1800};
int s05_limits[5]={ 700, 725,1150,1550,2050}; // FLM 90°=1925
int s06_limits[5]={1200,1300,1500,1700,1800};
int s07_limits[5]={1200,1300,1500,1700,1800};
int s08_limits[5]={ 550, 950,1450,1950,2045}; // RRH
int s09_limits[5]={ 700, 800,1225,1700,2750}; // FRM 90°=2225
int s10_limits[5]={ 650,1000,1425,1950,2650}; // FRL
int s11_limits[5]={ 650, 750,1175,1575,2350}; // RLM 90°=2050
int s12_limits[5]={ 550,1100,1600,2100,2450}; // RRL
int s13_limits[5]={ 550,1075,1450,1900,2250}; // FLL
int s14_limits[5]={1200,1300,1500,1700,1800};
int s15_limits[5]={1200,1300,1500,1700,1800};
int s16_limits[5]={ 550,1125,1500,1950,2350}; // RLL
int s17_limits[5]={1200,1300,1500,1700,1800};
int s18_limits[5]={1200,1300,1500,1700,1800};
int s19_limits[5]={ 750,850,1350,1800,2650}; // RRM 90°=2300

int s21_limits[5]={ 720,1500,1550,1500,2360}; // *150°
int s22_limits[5]={ 520, 975,1450,1900,2510}; // RRL
int s23_limits[5]={ 650,1025,1525,1925,2580}; // FRM 90°=2525
int s24_limits[5]={ 580,1500,1550,1500,2290}; // *170°
int s25_limits[5]={ 410,1100,1575,2025,2390}; // FLH
int s26_limits[5]={ 560,1025,1450,1955,2490}; // RLM 90°=2400
int s27_limits[5]={ 545,1075,1500,1975,2480}; // RLH
int s28_limits[5]={ 520,1050,1575,2050,2440}; // FRH
int s29_limits[5]={ 420,1025,1475,1850,2370}; // RRM 90°=2350
int s30_limits[5]={ 420,1175,1575,2050,2500}; // RRH
int s31_limits[5]={ 680,1150,1600,2075,2650}; // FRL
int s32_limits[5]={ 420,1500,1600,1500,2330}; // *170
int s33_limits[5]={ 480,1200,1625,2125,2420}; // FLL
int s34_limits[5]={ 590,1100,1525,2050,2570}; // RLL
int s35_limits[5]={ 420, 775,1225,1650,2440}; // FLM 90=2125

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
  hservo->pulse_outlimits=0;
  hservo->angle=0;
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
  hservo->slope=((double)(limits[3]-limits[1]))/(SERVO_LIMANGLE_MAX-SERVO_LIMANGLE_MIN);
}

/**********************************************************
  * @brief  Start a servo
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_Activate(t_servo *hservo)
{
  HAL_TIM_PWM_Start(hservo->htim, hservo->timer_channel);
}

/**********************************************************
  * @brief  Stop a servo
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_Deactivate(t_servo *hservo)
{
  hservo->pulse=0;
  hservo->pulse_outlimits=0;
  __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, 0);
  HAL_TIM_PWM_Stop(hservo->htim,hservo->timer_channel);
}

/**********************************************************
  * @brief  Set servo position
  * @param  hservo : servo handle
  * @retval none
  */
void Servo_SetPulse(t_servo *hservo, int pulse)
{
  if (((pulse>hservo->bmin_pulse) && (pulse<hservo->bmax_pulse)) || robot.state == CONFIGURATION)
  {
    hservo->pulse=pulse;
    hservo->pulse_outlimits=0;
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, pulse);
  }
  else
  {
    hservo->pulse_outlimits=pulse;
  }
}

/**********************************************************
 *  @brief Get pulse for a given angle
 *  @param hservo : servo handler
 *  @param angle : angle, in radians
 *  @retval pulse
 */
int Servo_GetPulseAngle(t_servo * hservo, double angle)
{
  return(round(hservo->amin_pulse+(angle-SERVO_LIMANGLE_MIN)*hservo->slope));
}

/**********************************************************
 *  @brief Set servo position from angle
 *  @param hservo : servo handler
 *  @param angle : angle, in radians
 */
void Servo_SetAngle(t_servo * hservo, double angle)
{
  int pulse;
  pulse=round(hservo->amin_pulse+(angle-SERVO_LIMANGLE_MIN)*hservo->slope);
  if (((pulse>hservo->bmin_pulse) && (pulse<hservo->bmax_pulse)) || robot.state == CONFIGURATION)
  {
    hservo->pulse=pulse;
    hservo->pulse_outlimits=0;
    hservo->angle=angle;
    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, pulse);
  }
  else
  {
    hservo->pulse_outlimits=pulse;
    hservo->angle=angle;
  }
}

/**********************************************************
 *  @brief Set servo position in % between min and max angle
 *  @param hservo : servo handler
 *  @param angle : angle, in radians
 */
void Servo_SetRatio(t_servo * hservo, int ratio)
{
#define ANGLE_TEST
#ifdef PULSE_TEST
  int pulse;
  pulse=round(hservo->amin_pulse+(hservo->amax_pulse-hservo->amin_pulse)*ratio/100);
  hservo->pulse=pulse;
  __HAL_TIM_SET_COMPARE(hservo->htim, hservo->timer_channel, pulse);
#endif
#ifdef ANGLE_TEST
  double angle;
  angle=-SERVO_LIMANGLE_MIN+(SERVO_LIMANGLE_MAX-SERVO_LIMANGLE_MIN)*(double)ratio/100.;
  Servo_SetAngle(hservo, angle);
#endif
#undef ANGLE_TEST
}

/**********************************************************
 *
 */
void Servo_Autotest(t_servo * hservo)
{
  int i;
  int pulse;
  for (i=0;i<=100;i++)
  {
    pulse=hservo->amin_pulse+(hservo->amax_pulse-hservo->amin_pulse)*i/100;
    Servo_SetPulse(hservo, pulse);
    HAL_Delay(20);
  }
  for (i=100;i>=0;i--)
  {
    pulse=hservo->amin_pulse+(hservo->amax_pulse-hservo->amin_pulse)*i/100;
    Servo_SetPulse(hservo, pulse);
    HAL_Delay(20);
  }
}


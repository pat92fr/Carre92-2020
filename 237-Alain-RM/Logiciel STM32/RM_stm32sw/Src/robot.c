/*
 * robot.c
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "serial.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

t_servo servo01;
t_servo servo02;
t_servo servo03;
t_servo servo07;
t_servo servo08;
t_servo servo09;
t_servo servo10;
t_servo servo11;
t_servo servo12;
t_servo servo16;
t_servo servo17;
t_servo servo18;

t_leg legFR;
t_leg legFL;
t_leg legRR;
t_leg legRL;

t_robot robot;

int sens=1;
extern int on_off;
uint8_t RxBuffer[64];
int RxBuffer_pos=0;

char CmdBuffer[16+1];
int pulse=1500;

int ctr50Hz=0;

void Robot_Init()
{
  int i;
  t_leg *hleg_temp;

  Servo_Init(&servo01, &htim3,  TIM_CHANNEL_4); // Servo connector 01 : tim3, ch4  => RRL, 19
  Servo_Init(&servo02, &htim2,  TIM_CHANNEL_4); // Servo connector 02 : tim2, ch4  => RRM, 12
  Servo_Init(&servo03, &htim2,  TIM_CHANNEL_3); // Servo connector 03 : tim2, ch3  => RRH, 08
  // Servo connector 04 : tim5, ch2 => unused
  // Servo connector 05 : tim3, ch1 => unused
  // Servo connector 06 : tim8, ch2 => unused
  Servo_Init(&servo07, &htim8,  TIM_CHANNEL_3); // Servo connector 07 : tim8, ch3  => RLH, 03
  Servo_Init(&servo08, &htim8,  TIM_CHANNEL_4); // Servo connector 08 : tim8, ch4  => RLM, 11
  Servo_Init(&servo09, &htim1,  TIM_CHANNEL_2); // Servo connector 09 : tim1, ch2  => RLL, 16
  Servo_Init(&servo10, &htim3,  TIM_CHANNEL_3); // Servo connector 10 : tim3, ch3  => FLL, 13
  Servo_Init(&servo11, &htim14, TIM_CHANNEL_1); // Servo connector 11 : tim14, ch1 => FLM, 05
  Servo_Init(&servo12, &htim11, TIM_CHANNEL_1); // Servo connector 12 : tim11, ch1 => FLH, 01
  // Servo connector 13 : tim3, ch2 => unused
  // Servo connector 14 : tim2, ch2 => unused
  // Servo connector 15 : tim1, ch1 => unused
  Servo_Init(&servo16, &htim10, TIM_CHANNEL_1); // Servo connector 16 : tim10, ch1 => FRH, 02
  Servo_Init(&servo17, &htim1,  TIM_CHANNEL_4); // Servo connector 17 : tim1, ch4  => FRM, 09
  Servo_Init(&servo18, &htim1,  TIM_CHANNEL_3); // Servo connector 18 : tim1, ch3  => FRL, 10
  // Servo connector 19 : tim2, ch1 => unused
  // Servo connector 20 : tim8, ch1 => unused
  // Servo connector 21 : tim5, ch1 => unused

  Servo_SetLimits(&servo01, s19_limits);
  Servo_SetLimits(&servo02, s12_limits);
  Servo_SetLimits(&servo03, s08_limits);
  Servo_SetLimits(&servo07, s03_limits);
  Servo_SetLimits(&servo08, s11_limits);
  Servo_SetLimits(&servo09, s16_limits);
  Servo_SetLimits(&servo10, s13_limits);
  Servo_SetLimits(&servo11, s05_limits);
  Servo_SetLimits(&servo12, s01_limits);
  Servo_SetLimits(&servo16, s02_limits);
  Servo_SetLimits(&servo17, s09_limits);
  Servo_SetLimits(&servo18, s10_limits);

  robot.hservos[RRL]=&servo01;
  robot.hservos[RRM]=&servo02;
  robot.hservos[RRH]=&servo03;
  robot.hservos[RLH]=&servo07;
  robot.hservos[RLM]=&servo08;
  robot.hservos[RLL]=&servo09;
  robot.hservos[FLL]=&servo10;
  robot.hservos[FLM]=&servo11;
  robot.hservos[FLH]=&servo12;
  robot.hservos[FRH]=&servo16;
  robot.hservos[FRM]=&servo17;
  robot.hservos[FRL]=&servo18;

  // Initialise legs
  Leg_Init(&legFR, robot.hservos[FRH], robot.hservos[FRM], robot.hservos[FRL], 0);
  Leg_Init(&legFL, robot.hservos[FLH], robot.hservos[FLM], robot.hservos[FLL], 1);
  Leg_Init(&legRR, robot.hservos[RRH], robot.hservos[RRM], robot.hservos[RRL], 0);
  Leg_Init(&legRL, robot.hservos[RLH], robot.hservos[RLM], robot.hservos[RLL], 1);

  robot.hlegs[0]=&legFL;
  robot.hlegs[1]=&legRL;
  robot.hlegs[2]=&legFR;
  robot.hlegs[3]=&legRR;

  // set initial position for legs
  // at 1st update, the array will be rotated, and last leg shall be returning
  // we initialise the positions, and rotate the array in reverse order
  for(i=0;i<NBLEGS;i++)
    Leg_SetPos(robot.hlegs[i], robot.hlegs[i]->origin_x-(i/(NBLEGS-1.))*robot.hlegs[i]->x_amplitude, robot.hlegs[i]->origin_y, LEG_HEIGHT);
  hleg_temp=robot.hlegs[0];
  for(i=0;i<NBLEGS-1;i++)
    robot.hlegs[i]=robot.hlegs[i+1];
  robot.hlegs[NBLEGS-1]=hleg_temp;

  robot.robot_on=0;
  robot.robot_mode=Standard;
  robot.step_x=0;
  robot.step_y=0;

  pulse=1500;
}

void Robot_Start()
{
  int i;

  robot.speed=50.; // speed in mm/s

  for(i=0;i<3*NBLEGS;i++)
    Servo_Start(robot.hservos[i]);
}

void Robot_Stop()
{
  int i;

  robot.speed=0.; // speed in mm/s

  for(i=0;i<3*NBLEGS;i++)
    Servo_Stop(robot.hservos[i]);
}


/**********************************************************
  * @brief  Update legs position
  * This function is run every 20ms by Timer 13.
  */
void Robot_Update()
{

  if(robot.robot_on == 1)
  {
    int i;

    if(robot.remsteps_before_return == 0)// new sequence, update return leg
    {
      t_leg *hleg_temp;
      float return_x, return_y; // return vector
      float return_d; // return distance
      float step_dist; // distance at each step
      // rotate legs array so returning leg is at last position
      hleg_temp=robot.hlegs[NBLEGS-1];
      for(i=NBLEGS-2;i>=0;i--)
        robot.hlegs[i+1]=robot.hlegs[i];
      robot.hlegs[0]=hleg_temp;
      hleg_temp=robot.hlegs[NBLEGS-1];

      // Distance for returning leg
      return_x=hleg_temp->origin_x-hleg_temp->x;
      return_y=hleg_temp->origin_y-hleg_temp->y;
      return_d=sqrtf(return_x*return_x+return_x*return_x);
      step_dist=robot.speed*SERVO_PULSE_PERIOD;
      robot.steps_before_return=(int)floorf(hleg_temp->x_amplitude/((NBLEGS-1)*step_dist));
      robot.remsteps_before_return=robot.steps_before_return;
      robot.ret_step_x=return_x/robot.steps_before_return;
      robot.ret_step_y=return_y/robot.steps_before_return;

      // step distance for staight line only
      robot.step_x=-robot.speed*SERVO_PULSE_PERIOD; // legs are going backward if speed is > 0
      robot.step_y=0;
    }
    // Leg steps
    for(i=0;i<NBLEGS-1;i++)
      Leg_Move(robot.hlegs[i], robot.step_x, robot.step_y, 0);
    Leg_Move(robot.hlegs[NBLEGS-1], robot.ret_step_x, robot.ret_step_y, 0);
    robot.remsteps_before_return--;
  }

  // count execution time
  int ctr;
  ctr=__HAL_TIM_GetCounter(&htim13);
  robot.tim13_ctr[robot.tim13_nb]=ctr;
  robot.tim13_nb++;

}

/**********************************************************
  * @brief  Read data from Message UART buffer and interpret commands
  */
void Robot_ReadCommand()
{
  // check if new data available in UART2 buffer
  if(Message_ReceivedLength() > 0)
  {
    char cmd_s[8];
    char buf[64];
    char cmd_i;
    int cmd_v;

    Message_Get((uint8_t*)cmd_s);
    Message_Send((uint8_t *)buf, sprintf(buf, "NewData : %d bytes\r\n", uart2_RxBuffer_len));
    cmd_s[3]=0;
    cmd_i=(char)cmd_s[0];
    cmd_v=atoi(cmd_s+1);

    if(robot.robot_mode == Configuration || 1)
    {
      if(cmd_i=='+')
      {
        pulse+=cmd_v;
        sprintf(buf, "OK +%d %d\r\n", cmd_v, pulse);
      }
      else if(cmd_i=='-')
      {
        pulse-=cmd_v;
        sprintf(buf, "OK -%d %d\r\n", cmd_v, pulse);
      }
      else if(cmd_i=='p')
      {
//        uart2_StartReceiveServoConf();
//        Robot_ReadServoConf(cmd_v);
      }
      else
        sprintf(buf, "Unkwn cmd %s\r\n", cmd_s);
    }
    Message_Send((uint8_t *)buf, strlen(buf));

    Message_StartReceive(5);
    HAL_Delay(250);
  }
}


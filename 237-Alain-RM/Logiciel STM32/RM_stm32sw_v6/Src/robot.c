/*
 * robot.c
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "string.h"
//#include "stdlib.h"
//#include "stdio.h"
#include "math.h"
#include "serial.h"
#include "tools.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "robot_update.h"
#include <robot_commands.h>
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

const char *Robot_ServoId[]={"FRH", "FRM", "FRL", "RRH", "RRM", "RRL", "FLH", "FLM", "FLL", "RLH", "RLM", "RLL"};
const char *Robot_LegId[]={"FR", "RR", "FL", "RL"};

int sens=1;
extern int on_off;
uint8_t RxBuffer[64];
int RxBuffer_pos=0;

char CmdBuffer[16+1];
int pulse=1500;

int ctr50Hz=0;

/**********************************************************
 * @brief Initialise robot
 *
 */
void Robot_Init()
{
  int i;
  // mechanical precomputation
  double t1, t2;
  double a_min, a_max;
  double sin_alpha_max, cos_alpha_max;
  double r_min, r_max;
  double rho_min;

  t_leg *hleg_temp;

  //*********************************************
  // Servo and leg initialisation
  //*********************************************
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

#if defined(STBOARD_TOP_FRONT) // STM32 board located top front
  Servo_SetLimits(&servo01, s22_limits); // RRL
  Servo_SetLimits(&servo02, s29_limits); // RRM
  Servo_SetLimits(&servo03, s30_limits); // RRH
  Servo_SetLimits(&servo07, s27_limits); // RLH
  Servo_SetLimits(&servo08, s26_limits); // RLM
  Servo_SetLimits(&servo09, s34_limits); // RLL
  Servo_SetLimits(&servo10, s33_limits); // FLL
  Servo_SetLimits(&servo11, s35_limits); // FLM
  Servo_SetLimits(&servo12, s25_limits); // FLH
  Servo_SetLimits(&servo16, s28_limits); // FRH
  Servo_SetLimits(&servo17, s23_limits); // FRM
  Servo_SetLimits(&servo18, s31_limits); // FRL
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
#elif defined(STBOARD_BOT_MIDDLE) // STM32 board located bottom middle
  Servo_SetLimits(&servo01, s10_limits); // FRL
  Servo_SetLimits(&servo02, s09_limits); // FRM
  Servo_SetLimits(&servo03, s02_limits); // FRH
  Servo_SetLimits(&servo07, s01_limits); // FLH
  Servo_SetLimits(&servo08, s05_limits); // FLM
  Servo_SetLimits(&servo09, s13_limits); // FLL
  Servo_SetLimits(&servo10, s16_limits); // RLL
  Servo_SetLimits(&servo11, s11_limits); // RLM
  Servo_SetLimits(&servo12, s03_limits); // RLH
  Servo_SetLimits(&servo16, s08_limits); // RRH
  Servo_SetLimits(&servo17, s19_limits); // RRM
  Servo_SetLimits(&servo18, s12_limits); // RRL
  robot.hservos[FRL]=&servo01;
  robot.hservos[FRM]=&servo02;
  robot.hservos[FRH]=&servo03;
  robot.hservos[FLH]=&servo07;
  robot.hservos[FLM]=&servo08;
  robot.hservos[FLL]=&servo09;
  robot.hservos[RLL]=&servo10;
  robot.hservos[RLM]=&servo11;
  robot.hservos[RLH]=&servo12;
  robot.hservos[RRH]=&servo16;
  robot.hservos[RRM]=&servo17;
  robot.hservos[RRL]=&servo18;
#else
  ERROR : STM32 board position undefined
#endif

  // Initialise legs
  Leg_Init(&legFR, "FR", robot.hservos[FRH], robot.hservos[FRM], robot.hservos[FRL], 1, 1);
  Leg_Init(&legFL, "FL", robot.hservos[FLH], robot.hservos[FLM], robot.hservos[FLL], 1, 0);
  Leg_Init(&legRR, "RR", robot.hservos[RRH], robot.hservos[RRM], robot.hservos[RRL], 0, 1);
  Leg_Init(&legRL, "RL", robot.hservos[RLH], robot.hservos[RLM], robot.hservos[RLL], 0, 0);

  // set hlegs tab
  robot.hlegs[FR]=&legFR;
  robot.hlegs[RR]=&legRR;
  robot.hlegs[FL]=&legFL;
  robot.hlegs[RL]=&legRL;

  //*********************************************
  // Compute mechanical parameters
  //*********************************************
  // r_min r_max : min and max foot/base distance
  a_min=LEG_LENGTH_M/sqrt(2.);
  t1=3./2.*LEG_LENGTH_M*LEG_LENGTH_M;
  t2=(2.-sqrt(2.))*LEG_LENGTH_M*LEG_LENGTH_L;
  a_max=sqrt(t1+t2);
  r_min=LEG_LENGTH_H+a_min;
  robot.mech_r_min=r_min;
  r_max=LEG_LENGTH_H+a_max;
  robot.mech_r_max=r_max;
  // alpha_max = max angle for H servo
  robot.mech_alpha_max=asin((ROBOT_LEG_X_AMPLITUDE/2.)/r_min);
  sin_alpha_max=(ROBOT_LEG_X_AMPLITUDE/2.)/r_min;
  cos_alpha_max=cos(robot.mech_alpha_max);
  // B, C, G points coordinates
  robot.mech_Cx=r_min*sin_alpha_max;
  robot.mech_Cy=r_min*cos_alpha_max;
  robot.mech_Bx=robot.mech_Cx;
  robot.mech_By=sqrt(r_max*r_max-r_min*r_min*sin_alpha_max*sin_alpha_max);
  robot.mech_Gx=robot.mech_Cx;
  robot.mech_Gy=r_min;
  robot.mech_Ax=-robot.mech_Cx;
  robot.mech_Ay=(robot.mech_Gy+robot.mech_By)/2.;
  // minimal curvature radius
  t1=r_max*r_max-r_min*r_min*sin_alpha_max*sin_alpha_max;
  t2=r_min*cos_alpha_max+sqrt(t1);
  rho_min=(4*sin_alpha_max*sin_alpha_max+t2*t2/4.)/t2;
  robot.mech_r_curv_min=rho_min;
  robot.mech_curv_max=1/rho_min;

  // movement parameters
  robot.cycletravel_x=robot.mech_Gx-robot.mech_Ax; // x component of travel during a cycle
  robot.cycletravel_y=robot.mech_Gy-robot.mech_Ay; // y component of travel during a cycle
  robot.cycletravel_d=sqrt(robot.cycletravel_x*robot.cycletravel_x+robot.cycletravel_y*robot.cycletravel_y);

  // set rotating hlegs tab : RL FR RR FL
  robot.hlegs_rtab[0]=&legRL;
  robot.hlegs_rtab[1]=&legFR;
  robot.hlegs_rtab[2]=&legRR;
  robot.hlegs_rtab[3]=&legFL;

  // Set initial position of legs
  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_SetStartPos(robot.hlegs_rtab[i], robot.mech_Ax+(i/(ROBOT_NBLEGS-1.))*robot.cycletravel_x, robot.mech_Ay, ROBOT_H);

  // at 1st update, the array will be rotated, and last leg shall be returning
  // we initialise the positions, and rotate the array in reverse order
  hleg_temp=robot.hlegs_rtab[0];
  for(i=0;i<ROBOT_NBLEGS-1;i++)
    robot.hlegs_rtab[i]=robot.hlegs_rtab[i+1];
  robot.hlegs_rtab[ROBOT_NBLEGS-1]=hleg_temp;

  // set current servo and leg for configuration
  robot.current_servo_idx=-1;
  robot.current_leg_idx=-1;

  // set initial speed and curvature
  robot.current_curv=0.;
  robot.current_speed=15;

  robot.state=FOLDED;
}

/**********************************************************
 * @brief Activate all servos
 */
void Robot_Activate()
{
  int i;
  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_Activate(robot.hlegs[i]);
}

/**********************************************************
 * @brief Deactivate all servos
 */
void Robot_Deactivate()
{
  int i;
  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_Deactivate(robot.hlegs[i]);
}

/**********************************************************
 * @brief Get servo index from ID
 */
int Robot_ServoId2Idx(char * id)
{
  int i;

  for(i=0;i<3*ROBOT_NBLEGS;i++)
    if(strcmp(id, Robot_ServoId[i]) == 0)
      return i;
  return -1;
}

/**********************************************************
 * @brief Get leg index from ID
 */
int Robot_LegId2Idx(char * id)
{
  int i;

  for(i=0;i<ROBOT_NBLEGS;i++)
    if(strcmp(id, Robot_LegId[i]) == 0)
      return i;
  return -1;
}

/**********************************************************
 * @brief  Move legs to start position
 */
void Robot_Unfold()
{
  int i;

  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_Activate(robot.hlegs_rtab[i]);

  // TODO : unflod legs smoothly
  for(i=0;i<ROBOT_NBLEGS;i++)
  {
    Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->start_x, robot.hlegs_rtab[i]->start_y, robot.hlegs_rtab[i]->start_z);
  }
#ifdef DEBUG_200220
  Servo_SetAngle(robot.hservos[RRL], 45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[RRM], -45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[RRH], 0.);
  Servo_SetAngle(robot.hservos[RLL], 45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[RLM], -45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[RLH], 0.);
  Servo_SetAngle(robot.hservos[FRL], 45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[FRM], -45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[FRH], 0.);
  Servo_SetAngle(robot.hservos[FLL], 45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[FLM], -45.*M_PI/180.);
  Servo_SetAngle(robot.hservos[FLH], 0.);
  robot.remsteps_before_return = 0;
#endif
  }

/**********************************************************
 * @brief  Move legs to store position
 */
void Robot_Fold()
{
  int i;

  // TODO : fold legs smoothly

  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_Deactivate(robot.hlegs_rtab[i]);

}

/**********************************************************
 * @brief  Move legs to start position
 */
void Robot_MoveToStartPos()
{
  int i;

  for(i=0;i<ROBOT_NBLEGS;i++)
    Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->start_x, robot.hlegs_rtab[i]->start_y, robot.hlegs_rtab[i]->start_z);

}

/**********************************************************
 * @brief Return robot state as string
 */
char * Robot_GetState()
{
  if (robot.state == FOLDED)
    return "FOLDED";
  else if (robot.state == READY)
    return "READY";
  else if (robot.state == RUNNING)
    return "RUNNING";
  else if (robot.state == CONFIGURATION)
    return "CONFIG";
  else
    return "UNKNOWN";
}

/**********************************************************
 * @brief Print a string to BT and serial
 */

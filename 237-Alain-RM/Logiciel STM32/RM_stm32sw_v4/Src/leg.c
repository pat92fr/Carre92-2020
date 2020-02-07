/*
 * leg.c
 *
 *  Created on: 30 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "math.h"
#include "servo.h"
#include "leg.h"

/**********************************************************
 *
 */
void Leg_Init(t_leg *hleg, char *label, t_servo *hservoH, t_servo *hservoM, t_servo *hservoL, int reverse)
{
  double t1;

  hleg->label=label;
  hleg->hservoH=hservoH;
  hleg->hservoM=hservoM;
  hleg->hservoL=hservoL;
  hleg->reverse=reverse;

  // Set start position
  t1=(LEG_LENGTH_L-LEG_HEIGHT)*(LEG_LENGTH_L-LEG_HEIGHT);
  hleg->span_min=LEG_LENGTH_H+sqrt(LEG_LENGTH_M*LEG_LENGTH_M-t1);
  t1=(LEG_LENGTH_M+LEG_LENGTH_L)*(LEG_LENGTH_M+LEG_LENGTH_L);
  hleg->span_max=LEG_LENGTH_H+sqrt(t1-LEG_HEIGHT*LEG_HEIGHT);
//  hleg->origin_y=hleg->span_max*cos(LEG_ANGLE_H/2.);
//  hleg->origin_x=hleg->span_max*sin(LEG_ANGLE_H/2.);
  hleg->x_min=-hleg->origin_x;
  hleg->x_max=hleg->origin_x;
  hleg->x_amplitude=2*hleg->origin_x;
}

/**********************************************************
 * @brief Activate leg
 */
void Leg_Start(t_leg * hleg)
{
  Servo_Start(hleg->hservoH);
  Servo_Start(hleg->hservoM);
  Servo_Start(hleg->hservoL);
}

/**********************************************************
 * @brief Desactivate leg
 */
void Leg_Stop(t_leg * hleg)
{
  Servo_Stop(hleg->hservoH);
  Servo_Stop(hleg->hservoM);
  Servo_Stop(hleg->hservoL);
}

/**********************************************************
 *
 */
void Leg_Autotest(t_leg * hleg)
{
  int i;
  t_servo * hservoH;
  t_servo * hservoM;
  t_servo * hservoL;

  hservoH=hleg->hservoH;
  hservoM=hleg->hservoM;
  hservoL=hleg->hservoL;

  for(i=0;i<=100;i++)
  {
    Servo_SetRatio(hservoH, i);
    Servo_SetRatio(hservoM, i);
    Servo_SetRatio(hservoL, i);
    HAL_Delay(20);
  }
  for(i=100;i>=0;i--)
  {
    Servo_SetRatio(hservoH, i);
    Servo_SetRatio(hservoM, i);
    Servo_SetRatio(hservoL, i);
    HAL_Delay(20);
  }
}

/**********************************************************
 *
 */
void Leg_SetPos(t_leg *hleg, double  x, double  y, double  z)
{
  double rho;
  double a;
  double c;
  double alpha_h;
  double alpha_m;
  double alpha_l;
  double t1, t2;

  hleg->x=x;
  hleg->y=y;
  hleg->z=z;

  // rho and angle
  rho=sqrt(x*x+y*y);
  if(hleg->reverse)
    alpha_h=-atan(x/y);
  else
    alpha_h=atan(x/y);

  a=rho-LEG_LENGTH_H;
  c=sqrtf(a*a+z*z);
  t1=c*c-LEG_LENGTH_M*LEG_LENGTH_M-LEG_LENGTH_L*LEG_LENGTH_L;
  t2=2.*c*LEG_LENGTH_M;
  alpha_m=atan(z/rho)-acos(t1/t2);

  t2=2.*LEG_LENGTH_M*LEG_LENGTH_L;
  alpha_l=M_PI-acos(-t1/t2);

  Servo_SetAngle(hleg->hservoH, alpha_h);
  Servo_SetAngle(hleg->hservoM, -alpha_m);
  Servo_SetAngle(hleg->hservoL, -alpha_l);
}

/**********************************************************
 *
 */
void Leg_Move(t_leg *hleg, double  dx, double  dy, double  dz)
{
  hleg->x=hleg->x+dx;
  hleg->y=hleg->y+dy;
  hleg->z=hleg->z+dz;
  Leg_SetPos(hleg, hleg->x, hleg->y, hleg->z);
}

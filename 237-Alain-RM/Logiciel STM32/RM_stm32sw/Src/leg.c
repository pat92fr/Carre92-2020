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

void Leg_Init(t_leg *hleg, t_servo *hservoH, t_servo *hservoM, t_servo *hservoL, int reverse)
{
  float t1;

  hleg->hservoH=hservoH;
  hleg->hservoM=hservoM;
  hleg->hservoL=hservoL;
  hleg->reverse=reverse;

  // Set start position
  t1=(LEG_LENGTH_L-LEG_HEIGHT)*(LEG_LENGTH_L-LEG_HEIGHT);
  hleg->span_min=LEG_LENGTH_H+sqrtf(LEG_LENGTH_M*LEG_LENGTH_M-t1);
  t1=(LEG_LENGTH_M+LEG_LENGTH_L)*(LEG_LENGTH_M+LEG_LENGTH_L);
  hleg->span_max=LEG_LENGTH_H+sqrtf(t1-LEG_HEIGHT*LEG_HEIGHT);
  hleg->origin_y=hleg->span_max*cosf(SERVO_ANGLE_MAX);
  hleg->origin_x=hleg->span_max*sinf(SERVO_ANGLE_MAX);
  hleg->x_min=-hleg->origin_x;
  hleg->x_max=hleg->origin_x;
  hleg->x_amplitude=2*hleg->origin_x;
}

void Leg_SetPos(t_leg *hleg, float x, float y, float z)
{
  float rho;
  float c;
  float alpha_h;
  float alpha_m;
  float alpha_l;
  float t1, t2;

  hleg->x=x;
  hleg->y=y;
  hleg->z=z;

  // rho and angle
  rho=sqrtf(x*x+y*y);
  if(hleg->reverse)
    alpha_h=-atanf(x/y);
  else
    alpha_h=atanf(x/y);

  c=sqrtf(rho*rho+z*z);
  t1=c*c-LEG_LENGTH_M*LEG_LENGTH_M-LEG_LENGTH_L*LEG_LENGTH_L;
  t2=2.*c*LEG_LENGTH_M;
  alpha_m=atanf(z/rho)-acosf(t1/t2);

  t2=2.*LEG_LENGTH_M*LEG_LENGTH_L;
  alpha_l=M_PI-acos(-t1/t2);

  Servo_SetAngle(hleg->hservoH, alpha_h);
  Servo_SetAngle(hleg->hservoM, -alpha_m);
  Servo_SetAngle(hleg->hservoL, -alpha_l);
}

void Leg_Move(t_leg *hleg, float dx, float dy, float dz)
{
  float nx;
  float ny;
  float nz;

  nx=hleg->x+dx;
  ny=hleg->y+dy;
  nz=hleg->z+dz;
  Leg_SetPos(hleg, nx, ny, nz);
}

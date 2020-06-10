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

/******************************************************************************
  * @brief  Init leg
  * @param hleg leg handle
  * @param label leg's name
  * @param hservoH H servo handle
  * @param hservoM M servo handle
  * @param hservoL L servo handle
  * @param right flag for right leg
  * @param front flag for front leg
  */
void Leg_Init(t_leg *hleg, char *label, t_servo *hservoH, t_servo *hservoM, t_servo *hservoL, int front, int right)
{
  hleg->label=label;
  hleg->hservoH=hservoH;
  hleg->hservoM=hservoM;
  hleg->hservoL=hservoL;
  hleg->right_flag=right;
  hleg->front_flag=front;

  // Set compensation to 0
  hleg->compensation_x=0.;
  hleg->compensation_y=0.;
  hleg->compensation_z=0.;
}

/**********************************************************
 * @brief Set leg start position
 * @param x,y,z leg position
 *
 * This function is used to unfold leg
 */
void Leg_SetStartPos(t_leg * hleg, double  x, double  y, double  z)
{
  double aH, aM, aL;
  hleg->start_x=x;
  hleg->start_y=y;
  hleg->start_z=z;
  Leg_GetAnglePos(hleg, x, y, z, &aH, &aM, &aL);
  hleg->hservoH->start_pulse=Servo_GetPulseAngle(hleg->hservoH, aH);
  hleg->hservoM->start_pulse=Servo_GetPulseAngle(hleg->hservoM, aM);
  hleg->hservoL->start_pulse=Servo_GetPulseAngle(hleg->hservoL, aL);
}

/**********************************************************
 * @brief Activate leg
 */
void Leg_Activate(t_leg * hleg)
{
  Servo_Activate(hleg->hservoH);
  Servo_Activate(hleg->hservoM);
  Servo_Activate(hleg->hservoL);
}

/**********************************************************
 * @brief Desactivate leg
 */
void Leg_Deactivate(t_leg * hleg)
{
  Servo_Deactivate(hleg->hservoH);
  Servo_Deactivate(hleg->hservoM);
  Servo_Deactivate(hleg->hservoL);
}

/**********************************************************
 * @brief Get servo angles for a leg position
 */
void Leg_GetAnglePos(t_leg * hleg,  double  x, double  y, double  z, double *aH, double *aM, double *aL)
{
  double rho;
  double a;
  double c;
  double t1, t2, t3, t4;
  double alpha_h;
  double alpha_m;
  double alpha_l;

  // rho and angle
  rho=sqrt(x*x+y*y);
  if(hleg->right_flag)
    alpha_h=atan(x/y);
  else
    alpha_h=-atan(x/y);

  a=rho-LEG_LENGTH_H;
  c=sqrtf(a*a+z*z);
  t1=c*c+LEG_LENGTH_M*LEG_LENGTH_M-LEG_LENGTH_L*LEG_LENGTH_L;
  t2=2.*c*LEG_LENGTH_M;
  alpha_m=atan(z/a)-acos(t1/t2);

  t3=LEG_LENGTH_L*LEG_LENGTH_L+LEG_LENGTH_M*LEG_LENGTH_M-c*c;
  t4=2.*LEG_LENGTH_M*LEG_LENGTH_L;
  alpha_l=M_PI-acos(t3/t4);

  *aH=alpha_h;
  *aM=alpha_m;
  *aL=alpha_l;
}

/**********************************************************
 * @brief Set leg position
 */
void Leg_SetPos(t_leg *hleg, double  x, double  y, double  z)
{
  double alpha_h, alpha_m, alpha_l;
  hleg->current_x=x;
  hleg->current_y=y;
  hleg->current_z=z;
  Leg_GetAnglePos(hleg, x+hleg->compensation_x, y+hleg->compensation_y, z+hleg->compensation_z, &alpha_h, &alpha_m, &alpha_l);
  Servo_SetAngle(hleg->hservoH, -alpha_h);
  Servo_SetAngle(hleg->hservoM, alpha_m);
  Servo_SetAngle(hleg->hservoL, alpha_l-M_PI/2);
}


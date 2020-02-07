/*
 * leg.h
 *
 *  Created on: 30 déc. 2019
 *      Author: alain
 */

#ifndef LEG_H_
#define LEG_H_

#define LEG_LENGTH_H 50.
#define LEG_LENGTH_M 40.
#define LEG_LENGTH_L 70.
#define LEG_HEIGHT 80.
#define LEG_H_ANGLE M_PI/6. // Angular amplitude for H servo
#define LEG_M_ANGLE_MIN -M_PI/4
#define LEG_M_ANGLE_MAX M_PI/4
#define LEG_L_ANGLE_MIN -3*M_PI/4
#define LEG_L_ANGLE_MAX M_PI/4

typedef struct
{
  char *label;
  t_servo *hservoH;
  t_servo *hservoM;
  t_servo *hservoL;
  int reverse;
  double x, y, z;
  double origin_x, origin_y; // origin position
  double span_min, span_max;
  double x_min, x_max, x_amplitude;
} t_leg;

void Leg_Init(t_leg * hleg, char *label, t_servo *servoH, t_servo *servoM, t_servo *servoL, int reverse);
void Leg_Start(t_leg * hleg);
void Leg_Stop(t_leg * hleg);
void Leg_Autotest(t_leg * hleg);
void Leg_SetPos(t_leg * hleg, double x, double y, double z);
void Leg_Move(t_leg * hleg, double dx, double dy, double dz);

#endif /* LEG_H_ */

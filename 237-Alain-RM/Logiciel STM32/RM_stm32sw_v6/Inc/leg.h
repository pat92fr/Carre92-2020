/*
 * leg.h
 *
 *  Created on: 30 déc. 2019
 *      Author: alain
 */

#ifndef LEG_H_
#define LEG_H_

#define LEG_LENGTH_H 47.
#define LEG_LENGTH_M 40.
#define LEG_LENGTH_L 100.
//#define LEG_HEIGHT 80.
#define LEG_H_ANGLE_MIN -M_PI/4
#define LEG_H_ANGLE_MAX M_PI/4
#define LEG_M_ANGLE_MIN -M_PI/2
#define LEG_M_ANGLE_MAX M_PI/2
#define LEG_L_ANGLE_MIN -3*M_PI/4
#define LEG_L_ANGLE_MAX M_PI/4

typedef struct
{
  char *label;
  t_servo *hservoH;
  t_servo *hservoM;
  t_servo *hservoL;
  double compensation_x, compensation_y, compensation_z;

  int right_flag;
  int front_flag;

  // current position
  double current_x, current_y, current_z;
  double current_heading;

  // unfolded position
  double start_x, start_y, start_z;
} t_leg;

void Leg_Init(t_leg * hleg, char *label, t_servo *servoH, t_servo *servoM, t_servo *servoL, int front, int right);
void Leg_Activate(t_leg * hleg);
void Leg_Deactivate(t_leg * hleg);
void Leg_SetStartPos(t_leg * hleg, double x, double y, double z);
void Leg_GetAnglePos(t_leg * hleg,  double  x, double  y, double  z, double *aH, double *aM, double *aL);
void Leg_SetPos(t_leg * hleg, double x, double y, double z);

#endif /* LEG_H_ */

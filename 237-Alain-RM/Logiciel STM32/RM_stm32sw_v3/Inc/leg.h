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

typedef struct
{
  t_servo *hservoH;
  t_servo *hservoM;
  t_servo *hservoL;
  int reverse;
  float x, y, z;
  float origin_x, origin_y; // origin position
  float span_min, span_max;
  float x_min, x_max, x_amplitude;
} t_leg;

void Leg_Init(t_leg * hleg, t_servo *servoH, t_servo *servoM, t_servo *servoL, int reverse);
void Leg_Start(t_leg * hleg);
void Leg_Stop(t_leg * hleg);
void Leg_Autotest(t_leg * hleg);
void Leg_SetPos(t_leg * hleg, float x, float y, float z);
void Leg_Move(t_leg * hleg, float dx, float dy, float dz);

#endif /* LEG_H_ */

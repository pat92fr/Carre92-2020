/*
 * servo.h
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#ifndef SERVO_H_
#define SERVO_H_

// angle values for angular pulse value definitions
#define SERVO_LIMANGLE_MIN -M_PI/4.
#define SERVO_LIMANGLE_MAX M_PI/4.


#define SERVO_PULSE_PERIOD 0.02 // pulse period in sec
#define SERVO_PULSE_MIN 1300
#define SERVO_PULSE_MAX 1700
#define SERVO_PULSE_MEAN 1500

// pulse values for min, amin, 0, amax and max
// for registered servos
extern int s00_limits[5];
extern int s01_limits[5];
extern int s02_limits[5];
extern int s03_limits[5];
extern int s04_limits[5];
extern int s05_limits[5];
extern int s06_limits[5];
extern int s07_limits[5];
extern int s08_limits[5];
extern int s09_limits[5];
extern int s10_limits[5];
extern int s11_limits[5];
extern int s12_limits[5];
extern int s13_limits[5];
extern int s14_limits[5];
extern int s15_limits[5];
extern int s16_limits[5];
extern int s17_limits[5];
extern int s18_limits[5];
extern int s19_limits[5];

typedef struct
{
  TIM_HandleTypeDef *htim;
  int timer_channel;
  int pulse;
  double angle;
  int bmin_pulse;
  int amin_pulse;
  int zero_pulse;
  int amax_pulse;
  int bmax_pulse;
  double slope;
} t_servo;


void Servo_Init(t_servo *hservo, TIM_HandleTypeDef *htim, int timer_channel);
void Servo_SetLimits(t_servo *hservo, int limits[5]);
void Servo_Start(t_servo *hservo);
void Servo_Stop(t_servo *hservo);
void Servo_SetPulse(t_servo *hservo, int pulse);
void Servo_SetAngle(t_servo * hservo, double angle);
void Servo_SetRatio(t_servo * hservo, int ratio);

void Servo_Autotest(t_servo * hservo);

#endif /* SERVO_H_ */

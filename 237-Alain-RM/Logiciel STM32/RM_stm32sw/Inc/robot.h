/*
 * robot.h
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#define NBLEGS 4

// Servo indexes
#define FRH 0
#define FRM 1
#define FRL 2
#define RRH 3
#define RRM 4
#define RRL 5
#define FLH 6
#define FLM 7
#define FLL 8
#define RLH 9
#define RLM 10
#define RLL 11

typedef struct
{
  t_leg   *hlegs[NBLEGS];
  t_servo *hservos[3*NBLEGS];
  float step_x, step_y;
  float ret_step_x, ret_step_y;
  int steps_before_return, remsteps_before_return;
  float speed;
  int exec_ctr;
  enum {Standard, Configuration} robot_mode;
  int robot_on;
  int tim13_ctr[100]; // TIM 13 IT duration, for interrupt time measurement
  int tim13_nb;       // TIM 13 IT number
} t_robot;

extern t_robot robot;
extern int robot_on;

void Robot_Init();
void Robot_Start();
void Robot_Stop();
void Robot_Update();
void Robot_ReadCommand();


#endif /* ROBOT_H_ */

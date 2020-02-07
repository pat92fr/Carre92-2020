/*
 * robot.h
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#define NBLEGS 4
#define ROBOT_H 50. // Robot height in mm
#define ROBOT_SERVO_H_MAX_ANGLE M_PI/6.


// Servo indexes
typedef enum {FRH=0, FRM=1, FRL=2, RRH=3, RRM=4, RRL=5, FLH=6, FLM=7, FLL=8, RLH=9, RLM=10, RLL=11} t_Robot_ServoId;

typedef enum {FOLDED, WAITING, RUNNING, CONFIGURATION} t_Robot_State;

typedef struct
{
  t_Robot_State state;
  // servo and leg handlers
  int current_servo_idx;
  t_leg   *hlegs[NBLEGS];
  t_servo *hservos[3*NBLEGS];
  // inverse cinematic parameters
  double w_min, w_max; // min and max foot distance to center
  double Ax, Ay, Bx, By;
  // movement parameters
  double cycletravel_x, cycletravel_y, cycletravel_d;
  double step_x, step_y; // period step for forward legs
  double ret_step_x, ret_step_y; // period step for return leg
  int steps_before_return, remsteps_before_return;
  double speed; // running speed in mm/s
  // monitoring and control
  int robot_on;
  int exec_ctr;
  int tim13_ctr[100]; // TIM 13 IT duration, for interrupt time measurement
  int tim13_nb;       // TIM 13 IT number
  HAL_Serial_Handler *debug_hserial;
} t_robot;

extern t_robot robot;
extern int robot_on;

void Robot_Init();
void Robot_Unfold();
void Robot_Fold();
void Robot_Update();
void Robot_ReadCommand(HAL_Serial_Handler * hserial, char *command);
void Robot_ButtonPushed();
char * Robot_GetState();

void Robot_Autotest();


#endif /* ROBOT_H_ */

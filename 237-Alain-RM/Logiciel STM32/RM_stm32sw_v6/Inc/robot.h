/*
 * robot.h
 *
 *  Created on: 23 déc. 2019
 *      Author: alain
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#define ROBOT_NBLEGS 4
#define ROBOT_H 80.
#define ROBOT_LEG_X_AMPLITUDE 100.
#define ROBOT_LENGTH 162.
#define ROBOT_WIDTH  117.

#define STBOARD_BOT_MIDDLE

// Servo indexes
typedef enum {FRH=0, FRM=1, FRL=2, RRH=3, RRM=4, RRL=5, FLH=6, FLM=7, FLL=8, RLH=9, RLM=10, RLL=11} t_Robot_ServoId;
typedef enum {FR=0, RR=1, FL=2, RL=3} t_Robot_LegId;

typedef enum {FOLDED, READY, RUNNING, CONFIGURATION} t_Robot_State;

extern const char *Robot_ServoId[ROBOT_NBLEGS*3];
extern const char *Robot_LegId[ROBOT_NBLEGS];

typedef struct
{
  t_Robot_State state;
  // servo and leg handlers
  int current_servo_idx;
  int current_leg_idx;
  t_servo *hservos[3*ROBOT_NBLEGS];
  t_leg   *hlegs[ROBOT_NBLEGS];
  t_leg   *hlegs_rtab[ROBOT_NBLEGS]; // rotating legs tab
  // Mechanical parameters
  double mech_r_min, mech_r_max;
  double mech_alpha_max;
  double mech_Bx, mech_By;
  double mech_Cx, mech_Cy;
  double mech_Gx, mech_Gy;
  double mech_Ax, mech_Ay;
  double mech_r_curv_min;
  double mech_curv_max;

  // movement parameters
  double cycletravel_x, cycletravel_y, cycletravel_d;
  double step_x, step_y; // period step for forward legs
  double ret_step_x, ret_step_y; // period step for return leg
  int steps_before_return, remsteps_before_return;
  double current_speed; // running speed in mm/s
  double current_curv;  // curvature in mm^-1
  // monitoring and control
  int robot_on;
  int exec_ctr;
  int tim13_ctr[100]; // TIM 13 IT duration, for interrupt time measurement
  int tim13_nb;       // TIM 13 IT number
  HAL_Serial_Handler *debug_hserial;

  // Debug values
  double ret_dz; // 200521 : watch returning leg height
} t_robot;

extern t_robot robot;
extern int robot_on;

void Robot_Init();
void Robot_Unfold();
void Robot_Fold();
void Robot_MoveToStartPos();

//void Robot_ReadCommand(HAL_Serial_Handler * hserial, char *command);
int Robot_ServoId2Idx(char * id);
int Robot_LegId2Idx(char * id);

char * Robot_GetState();


#endif /* ROBOT_H_ */

/*
 * robot_update.c
 *
 *  Created on: 15 march 2020
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "math.h"
#include "serial.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "main.h"

int tim13_nb=0;

/**********************************************************
  * @brief  Update legs position
  * This function is run every 20ms by Timer 13.
  */
void Robot_Update()
{
  if((robot.state == RUNNING) && 1)
  {
    int i;

    double dr;
    double d;
    double u,v;

    double delta;
    double rho;
    double cos_delta, sin_delta;
    double dx, dy, dz;

    int nr;

    t_leg *hleg_returning;
    t_leg *hleg_next;
    t_leg *hleg_temp;

    hleg_returning=robot.hlegs_rtab[ROBOT_NBLEGS-1];
    hleg_next=robot.hlegs_rtab[ROBOT_NBLEGS-2];

    // Compute remaining distance for next returning leg
    dr=robot.mech_Bx-hleg_next->current_x;

    // Compute remaining number of steps
    d=robot.current_speed*SERVO_PULSE_PERIOD;
    if(dr>=0.)
      nr=(int)floor(dr/d);
    else
      nr=(int)floor(-dr/d);

    // update number of steps for cycle
    robot.steps_before_return=robot.steps_before_return+1+nr-robot.remsteps_before_return;
    robot.remsteps_before_return=nr;

    // new sequence ? update return leg
    if(nr == 0)
    {
      // rotate legs array so returning leg is at last position
      hleg_temp=robot.hlegs_rtab[ROBOT_NBLEGS-1];
      for(i=ROBOT_NBLEGS-2;i>=0;i--)
        robot.hlegs_rtab[i+1]=robot.hlegs_rtab[i];
      robot.hlegs_rtab[0]=hleg_temp;

      // recompute remaining distance and number of steps
      hleg_returning=robot.hlegs_rtab[ROBOT_NBLEGS-1];
      hleg_next=robot.hlegs_rtab[ROBOT_NBLEGS-2];
      dr=robot.mech_Bx-hleg_next->current_x;
      d=robot.current_speed*SERVO_PULSE_PERIOD;
      if(dr>=0.)
        nr=(int)floor(dr/d);
      else
        nr=(int)floor(-dr/d);
      robot.remsteps_before_return=nr;
      robot.steps_before_return=nr;
    }

    // move normal legs
    for(i=0;i<ROBOT_NBLEGS-1;i++)
    {
      hleg_temp=robot.hlegs_rtab[i];
      if(robot.current_curv == 0.)
      {
        Leg_SetPos(hleg_temp, hleg_temp->current_x+d, hleg_temp->current_y, hleg_temp->start_z);
      }
      else
      {
        delta = d*robot.current_curv;
        rho=1/robot.current_curv;
        cos_delta=cos(delta);
        sin_delta=sin(delta);
        if(robot.current_curv > 0) // right turn
        {
          if(hleg_temp->right_flag) // interior leg
          {
            if(hleg_temp->front_flag)
            {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else // rear leg
            {
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
          }
          else // exterior leg
          {
            if(hleg_temp->front_flag) // front leg
            {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else // rear leg
            {
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
          } // interior/exterior leg
        }
        else // left turn
        {
          if(!(hleg_temp->right_flag)) // interior leg
          {
            if(hleg_temp->front_flag)
            {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else // rear leg
            {
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            } // front/rear leg
          }
          else // exterior leg
          {
            if(hleg_temp->front_flag) // front leg
            {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else // rear leg
            {
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            } // front/rear leg
          } // interior/exterior leg
        } // right/left turn
        Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->current_x-dx, robot.hlegs_rtab[i]->current_y-dy, robot.hlegs_rtab[i]->start_z);
      } // stright line or curve
    } // leg iterator


    // move returning leg
    u=hleg_returning->current_x-robot.mech_Ax;
    v=hleg_returning->current_y-robot.mech_Ay;
    dz=30.0*sqrt(sin((1.*robot.remsteps_before_return)/(1.*robot.steps_before_return)*M_PI));
    // DEBUG 200521 : watch returning leg height
    robot.ret_dz=dz;
    // END DEBUG
    Leg_SetPos(hleg_returning, hleg_returning->current_x-u/robot.remsteps_before_return, hleg_returning->current_y-v/robot.remsteps_before_return, hleg_returning->start_z-dz);
  }

  if (tim13_nb >50)
  {
    tim13_nb=0;
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  }
  tim13_nb++;
}

/**********************************************************
  * @brief  Update legs position
  * This function is run every 20ms by Timer 13.
  */
void Robot_Update_old()
{
  if((robot.state == RUNNING) && 0) {
    int i;

    double dr;
    double d;
    double u,v;
    int nr;

    t_leg *hleg_returning;
    t_leg *hleg_next;
    t_leg *hleg_temp;

    hleg_returning=robot.hlegs_rtab[ROBOT_NBLEGS-1];
    hleg_next=robot.hlegs_rtab[ROBOT_NBLEGS-2];

    // Compute remaining distance for next returning leg
    dr=robot.mech_Bx-hleg_next->current_x;

    // Compute remaining number of steps
    d=robot.current_speed*SERVO_PULSE_PERIOD;
    nr=(int)ceil(dr/d);

    // update number of steps for cycle
    robot.steps_before_return=robot.steps_before_return+nr-robot.remsteps_before_return;
    robot.remsteps_before_return=nr;

    // new sequence ? update return leg
    if(nr == 0) {
      // rotate legs array so returning leg is at last position
      hleg_temp=robot.hlegs_rtab[ROBOT_NBLEGS-1];
      for(i=ROBOT_NBLEGS-2;i>=0;i--)
        robot.hlegs_rtab[i+1]=robot.hlegs_rtab[i];
      robot.hlegs_rtab[0]=hleg_temp;

      // recompute remaining distance and number of steps
      hleg_returning=robot.hlegs_rtab[ROBOT_NBLEGS-1];
      hleg_next=robot.hlegs_rtab[ROBOT_NBLEGS-2];
      dr=robot.mech_Bx-hleg_next->current_x;
      d=robot.current_speed*SERVO_PULSE_PERIOD;
      nr=ceil(dr/d);
      robot.remsteps_before_return=nr;
      robot.steps_before_return=nr;
    }


    double delta;
    double rho;
    double cos_delta, sin_delta;
    double dx, dy, dz;

    // move normal legs
    for(i=0;i<ROBOT_NBLEGS-1;i++) {
      hleg_temp=robot.hlegs_rtab[i];
      if(robot.current_curv == 0.) {
        Leg_SetPos(hleg_temp, hleg_temp->current_x-d, hleg_temp->current_y, hleg_temp->start_z);
      }
      else {
        delta = d*robot.current_curv;
        rho=1/robot.current_curv;
        cos_delta=cos(delta);
        sin_delta=sin(delta);
        if(robot.current_curv > 0) { // right turn
          if(hleg_temp->right_flag) { // interior leg
            if(hleg_temp->front_flag) {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else { // rear leg
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
          }
          else { // exterior leg
            if(hleg_temp->front_flag) { // front leg
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else { // rear leg
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
          } // interior/exterior leg
        }
        else { // left turn
          if(!(hleg_temp->right_flag)) { // interior leg
            if(hleg_temp->front_flag) {
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else { // rear leg
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho-ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho-ROBOT_WIDTH/2.)*(cos_delta-1);
            } // front/rear leg
          }
          else { // exterior leg
            if(hleg_temp->front_flag) { // front leg
              dx=ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            }
            else { // rear leg
              dx=-ROBOT_LENGTH/2.*(cos_delta-1.)-(rho+ROBOT_WIDTH/2.)*sin_delta;
              dy=-ROBOT_LENGTH/2.*sin_delta+(rho+ROBOT_WIDTH/2.)*(cos_delta-1);
            } // front/rear leg
          } // interior/exterior leg
        } // right/left turn
        Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->current_x+dx, robot.hlegs_rtab[i]->current_y+dy, robot.hlegs_rtab[i]->start_z);
      } // stright line or curve
    } // leg iterator


    // move returning leg
    u=hleg_returning->current_x-robot.mech_Ax;
    v=hleg_returning->current_y-robot.mech_Ay;
    dz=30.0*sqrt(sin((1.*robot.remsteps_before_return)/(1.*robot.steps_before_return)*M_PI));
    Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->current_x+u/robot.remsteps_before_return, robot.hlegs_rtab[i]->current_y+v/robot.remsteps_before_return, robot.hlegs_rtab[i]->start_z-dz);
  }

  robot.tim13_nb++;
  if (robot.tim13_nb >50) {
    if((robot.state == RUNNING) && 1) {
      HAL_Serial_Print(robot.debug_hserial, "remsteps=%d/%d\n", robot.remsteps_before_return, robot.steps_before_return);
      HAL_Serial_Print(robot.debug_hserial, "Leg 1 x=%f\n", robot.hlegs_rtab[1]->current_x);
      HAL_Serial_Print(robot.debug_hserial, "Servo 0 p=%d\n", robot.hservos[0]->pulse);
      HAL_Serial_Print(robot.debug_hserial, "Servo 0 a(deg)=%f\n", robot.hservos[0]->angle*180/M_PI);
      HAL_Serial_Print(robot.debug_hserial, "Rleg z=%f\n", robot.hlegs_rtab[ROBOT_NBLEGS-1]->current_z);
    }
    else {
      HAL_Serial_Print(robot.debug_hserial, "state=%s\n", Robot_GetState());
    }
    robot.tim13_nb=0;
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  }

#ifdef OLD_VERSION
    if(robot.remsteps_before_return == 0)// new sequence, update return leg
    {
      t_leg *hleg_temp;
      double return_x, return_y; // return vector
      //double return_d; // return distance
      double step_dist; // distance at each step

      // rotate legs array so returning leg is at last position
      hleg_temp=robot.hlegs_rtab[ROBOT_NBLEGS-1];
      for(i=ROBOT_NBLEGS-2;i>=0;i--)
        robot.hlegs_rtab[i+1]=robot.hlegs_rtab[i];
      robot.hlegs_rtab[0]=hleg_temp;
      hleg_temp=robot.hlegs_rtab[ROBOT_NBLEGS-1]; // returning leg

      // number of steps for cycle part
      step_dist=robot.current_speed*SERVO_PULSE_PERIOD; // distance at each step
      robot.steps_before_return=(int)floor(robot.cycletravel_d/((ROBOT_NBLEGS-1)*step_dist));
      robot.remsteps_before_return=robot.steps_before_return;

      // Distance for returning leg
      return_x=robot.mech_Ax-hleg_temp->current_x;
      return_y=robot.mech_Ay-hleg_temp->current_y;
      robot.ret_step_x=return_x/robot.steps_before_return;
      robot.ret_step_y=return_y/robot.steps_before_return;

      // Step coordinate
      if(robot.current_curv == 0.) // straight ahead
      {
        // step distance for straight line only
        robot.step_x=-robot.current_speed*SERVO_PULSE_PERIOD; // legs are going backward if speed is > 0
        robot.step_y=0;

      }
      else // curve step
      {
        double r_curv;
        r_curv=1/robot.current_curv;
      }




    }

    hleg_return=robot.hlegs_rtab[ROBOT_NBLEGS-1];
    hleg_next=robot.hlegs_rtab[ROBOT_NBLEGS-2];

    // update remaining steps



    // Leg steps for forwarding legs
    for(i=0;i<ROBOT_NBLEGS-1;i++)
      Leg_SetPos(robot.hlegs_rtab[i], robot.hlegs_rtab[i]->current_x+robot.step_x, robot.hlegs_rtab[i]->current_y+robot.step_y, robot.hlegs_rtab[i]->start_z);
    // leg step for returning leg
    dz=30.0*sqrt(sin((1.*robot.remsteps_before_return)/(1.*robot.steps_before_return)*M_PI));
    Leg_SetPos(robot.hlegs_rtab[ROBOT_NBLEGS-1], robot.hlegs_rtab[ROBOT_NBLEGS-1]->current_x+robot.ret_step_x, robot.hlegs_rtab[ROBOT_NBLEGS-1]->current_y+robot.ret_step_y, robot.hlegs_rtab[i]->start_z-dz);
    // update phase step index
    robot.remsteps_before_return--;

    /* Toggle LED 3 in running state*/
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
  else
  {
    /* Toggle LED 2 if not in running state */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }

  robot.tim13_nb++;
  if (robot.tim13_nb >50)
  {
    if((robot.state == RUNNING) && 0)
    {
      HAL_Serial_Print(robot.debug_hserial, "remsteps=%d/%d\n", robot.remsteps_before_return, robot.steps_before_return);
      HAL_Serial_Print(robot.debug_hserial, "Leg 1 x=%f\n", robot.hlegs_rtab[1]->current_x);
      HAL_Serial_Print(robot.debug_hserial, "Servo 0 p=%d\n", robot.hservos[0]->pulse);
      HAL_Serial_Print(robot.debug_hserial, "Servo 0 a(deg)=%f\n", robot.hservos[0]->angle*180/M_PI);
      HAL_Serial_Print(robot.debug_hserial, "Rleg z=%f\n", robot.hlegs_rtab[ROBOT_NBLEGS-1]->current_z);
    }
    robot.tim13_nb=0;
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  }
#endif

}


/*
 * robot_readcommand.c
 *
 *  Created on: 15 march 2020
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "tools.h"
#include "serial.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"

/**********************************************************
  * @brief Interpret a command - State FOLDED
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand_Folded(HAL_Serial_Handler * hserial, char *cmd)
{
  // Cmd unfold ***********
  if (strcmp(cmd, "unfold") == 0)
  {
    Robot_Unfold();
    robot.state=READY;
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  // Cmd config ***********
  else if (strcmp(cmd, "config") == 0)
  {
    robot.state=CONFIGURATION;
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  else
    HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state FOLDED\n", cmd);
}

/**********************************************************
  * @brief Interpret a command - State READY
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand_Ready(HAL_Serial_Handler * hserial, char *cmd)
{
  // Cmd run **************
  if (strcmp(cmd, "run") == 0)
  {
    // robot_update() move legs if in RUNNING state
    robot.state=RUNNING;
    LedSwitch(2, 0);
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  // cmd speed ***************
  else if (strncmp(cmd, "speed", 5) == 0)
  {
    int speed;
    if (sscanf(cmd, "speed%d", &speed) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"select<int>\"\n", cmd);
    }
    else
    {
      robot.current_speed=(double)speed;
      HAL_Serial_Print(hserial,"OK speed set to %d\n", (int)robot.current_speed);
    }
  }
  // Cmd fold *************
  else if (strcmp(cmd, "fold") == 0)
  {
    Robot_Fold();
    robot.state=FOLDED;
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  else if (strncmp(cmd, "curv", 4) == 0)
  {
    int curv;
    if (sscanf(cmd, "curv%d", &curv) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"curv<int>\"\n", cmd);
    }
    else
    {
      robot.current_curv=(double)curv/1000000.;
      HAL_Serial_Print(hserial,"OK curv set to %f\n", robot.current_curv);
    }
  }
  // Cmd config ***********
  else if (strcmp(cmd, "config") == 0)
  {
    robot.state=CONFIGURATION;
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  else
    HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state READY\n", cmd);
}

/**********************************************************
  * @brief Interpret a command - State RUNNING
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand_Running(HAL_Serial_Handler * hserial, char *cmd)
{
  if (strcmp(cmd, "stop") == 0)
  {
    // robot_update() stop moving legs if not in RUNNING state
    robot.state=READY;
    robot.current_speed=0;
    robot.current_curv=0.;
    LedSwitch(3, 0);
    HAL_Serial_Print(hserial,"OK %s\n", cmd);
  }
  else if (strncmp(cmd, "speed", 5) == 0)
  {
    int speed;
    if (sscanf(cmd, "speed%d", &speed) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"speed<int>\"\n", cmd);
    }
    else
    {
      robot.current_speed=(double)speed;
      HAL_Serial_Print(hserial,"OK speed set to %d\n", (int)robot.current_speed);
    }
  }
  else if (strncmp(cmd, "curv", 4) == 0)
  {
    int curv;
    if (sscanf(cmd, "curv%d", &curv) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"curv<int>\"\n", cmd);
    }
    else
    {
      robot.current_curv=(double)curv/1000000.;
      HAL_Serial_Print(hserial,"OK curv set to %f\n", robot.current_curv);
    }
  }
  else
    HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state RUNNING\n", cmd);
}

/**********************************************************
  * @brief Interpret a command - State CONFIG
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand_Config(HAL_Serial_Handler * hserial, char *cmd)
{
  // Cmd select servo *****
  if (strncmp(cmd, "servo", 5) == 0)
  {
    char servo_id[8];
    if (sscanf(cmd, "servo%s", servo_id) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"select<int>\"\n", cmd);
    }
    else
    {
      int servo_idx;
      HAL_Serial_Print(hserial, " servo \"%s\" selected\n", servo_id);
      servo_idx=Robot_ServoId2Idx(servo_id);
      if( servo_idx >= 0)
      {
        robot.current_servo_idx=servo_idx;
        HAL_Serial_Print(hserial,"OK servo %d selected\n", servo_idx);
      }
      else
        HAL_Serial_Print(hserial, "KO Wrong servo ID\"%s\"\n", servo_id);
    }
  }
  // Cmd set pulse ********
  else if(strncmp(cmd, "pulse", 5) == 0)
  {
    int pulse;
    if (sscanf(cmd, "pulse%d", &pulse) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"pulse<int>\"\n", cmd);
    }
    else if((pulse < SERVO_PULSE_MIN) && (pulse > SERVO_PULSE_MAX))
      HAL_Serial_Print(hserial, "KO Wrong pulse value \"%d\"\n", pulse);
    else
    {
      Servo_SetPulse(robot.hservos[robot.current_servo_idx], pulse);
      HAL_Serial_Print(hserial,"OK pulse set to %d\n", robot.hservos[robot.current_servo_idx]->pulse);
    }
  }
  // Cmd set compensation x ********
  else if(strncmp(cmd, "compensx", 8) == 0)
  {
    int compensation;
    if (sscanf(cmd, "compensx%d", &compensation) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"compensx<int>\"\n", cmd);
    }
    else
    {
      robot.hlegs[robot.current_leg_idx]->compensation_x=(double)compensation;
      HAL_Serial_Print(hserial, "%f;%f;%f;\n", robot.hlegs[robot.current_leg_idx]->current_x, robot.hlegs[robot.current_leg_idx]->current_y, robot.hlegs[robot.current_leg_idx]->current_z);
      Leg_SetPos(robot.hlegs[robot.current_leg_idx], robot.hlegs[robot.current_leg_idx]->current_x, robot.hlegs[robot.current_leg_idx]->current_y, robot.hlegs[robot.current_leg_idx]->current_z);
      HAL_Serial_Print(hserial,"OK compensation x set to %f\n", robot.hlegs[robot.current_leg_idx]->compensation_x);
    }
  }
  // Cmd set compensation y ********
  else if(strncmp(cmd, "compensy", 8) == 0)
  {
    int compensation;
    if (sscanf(cmd, "compensy%d", &compensation) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"compensy<int>\"\n", cmd);
    }
    else
    {
      robot.hlegs[robot.current_leg_idx]->compensation_y=(double)compensation;
      Leg_SetPos(robot.hlegs[robot.current_leg_idx], robot.hlegs[robot.current_leg_idx]->current_x, robot.hlegs[robot.current_leg_idx]->current_y, robot.hlegs[robot.current_leg_idx]->current_z);
      HAL_Serial_Print(hserial,"OK compensation y set to %f\n", robot.hlegs[robot.current_leg_idx]->compensation_y);
    }
  }
  // Cmd set compensation z ********
  else if(strncmp(cmd, "compensz", 8) == 0)
  {
    int compensation;
    if (sscanf(cmd, "compensz%d", &compensation) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"compensz<int>\"\n", cmd);
    }
    else
    {
      robot.hlegs[robot.current_leg_idx]->compensation_z=(double)compensation;
      Leg_SetPos(robot.hlegs[robot.current_leg_idx], robot.hlegs[robot.current_leg_idx]->current_x, robot.hlegs[robot.current_leg_idx]->current_y, robot.hlegs[robot.current_leg_idx]->current_z);
      HAL_Serial_Print(hserial,"OK compensation z set to %f\n", robot.hlegs[robot.current_leg_idx]->compensation_z);
    }
  }
  // Cmd set angle ********
  else if(strncmp(cmd, "angle", 5) == 0)
  {
    int angle;
    if (sscanf(cmd, "angle%d", &angle) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"angle<int>\"\n", cmd);
    }
    else
    {
      Servo_SetAngle(robot.hservos[robot.current_servo_idx], (double)angle*M_PI/180.);
      HAL_Serial_Print(hserial,"OK angle set to %f\n", robot.hservos[robot.current_servo_idx]->angle*180./M_PI);
      HAL_Serial_Print(hserial,"   pulse is %d\n", robot.hservos[robot.current_servo_idx]->pulse);
    }
  }
  // Cmd set pulse limits **************
  else if(strncmp(cmd, "limits", 6) == 0)
  {
    int limits[5];
    if (sscanf(cmd, "limits%d;%d;%d;%d;%d\n", limits, limits+1, limits+2, limits+3, limits+4) != 5)
      HAL_Serial_Print(hserial, "KO, Wrond command syntax \"%s\" for command \"limits<int>;<int>;<int>;<int>;<int>\"\n", cmd);
    else
      Servo_SetLimits(robot.hservos[robot.current_servo_idx], limits);
  }
  // Cmd select leg ****************
  else if (strncmp(cmd, "leg", 3) == 0)
  {
    char leg_id[8];
    if (sscanf(cmd, "leg%s", leg_id) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"select<int>\"\n", cmd);
    }
    else
    {
      int leg_idx;
      HAL_Serial_Print(hserial, " leg \"%s\" selected\n", leg_id);
      leg_idx=Robot_LegId2Idx(leg_id);
      if(leg_idx >= 0)
      {
        robot.current_leg_idx=leg_idx;
        HAL_Serial_Print(hserial,"OK leg %d selected\n", leg_idx);
      }
      else
        HAL_Serial_Print(hserial, "KO Wrong leg ID\"%s\"\n", leg_id);
    }
  }
  // Cmd x *****
  else if (strncmp(cmd, "x", 1) == 0)
  {
    int dx;
    if (sscanf(cmd, "x%d", &dx) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"x<int>\"\n", cmd);
    }
    else
    {
      double x;
      t_leg *hleg;
      hleg=robot.hlegs[robot.current_leg_idx];
      x=hleg->current_x;
      Leg_SetPos(hleg, x+(double)dx, hleg->current_y, hleg->current_z);
      HAL_Serial_Print(hserial, "Leg %d set to (%f,%f,%f)\n", robot.current_leg_idx, (hleg->current_x), (hleg->current_y), (hleg->current_z));
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%d, M=%d, L=%d\n", hleg->hservoH->pulse, hleg->hservoM->pulse, hleg->hservoL->pulse);
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%f°,M=%f°,L=%f°\n", (hleg->hservoH->angle*180/M_PI), (hleg->hservoM->angle*180/M_PI), (hleg->hservoL->angle*180/M_PI));
    }
  }
  // Cmd y *****
  else if (strncmp(cmd, "y", 1) == 0)
  {
    int dy;
    if (sscanf(cmd, "y%d", &dy) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"y<int>\"\n", cmd);
    }
    else
    {
      double y;
      t_leg *hleg;
      hleg=robot.hlegs[robot.current_leg_idx];
      y=hleg->current_y;
      Leg_SetPos(hleg, hleg->current_x, y+(double)dy, hleg->current_z);
      HAL_Serial_Print(hserial, "Leg %d set to (%f,%f,%f)\n", robot.current_leg_idx, (hleg->current_x), (hleg->current_y), (hleg->current_z));
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%d, M=%d, L=%d\n", hleg->hservoH->pulse, hleg->hservoM->pulse, hleg->hservoL->pulse);
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%f°,M=%f°,L=%f°\n", (hleg->hservoH->angle*180/M_PI), (hleg->hservoM->angle*180/M_PI), (hleg->hservoL->angle*180/M_PI));
    }
  }
  // Cmd z *****
  else if (strncmp(cmd, "z", 1) == 0)
  {
    int dz;
    if (sscanf(cmd, "z%d", &dz) != 1)
    {
      HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"z<int>\"\n", cmd);
    }
    else
    {
      double z;
      t_leg *hleg;
      hleg=robot.hlegs[robot.current_leg_idx];
      z=hleg->current_z;
      Leg_SetPos(hleg, hleg->current_x, hleg->current_y, z+(double)dz);
      HAL_Serial_Print(hserial, "Leg %d set to (%f,%f,%f)\n", robot.current_leg_idx, (hleg->current_x), (hleg->current_y), (hleg->current_z));
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%d, M=%d, L=%d\n", hleg->hservoH->pulse, hleg->hservoM->pulse, hleg->hservoL->pulse);
      HAL_Delay(20);
      HAL_Serial_Print(hserial, "       H=%f°,M=%f°,L=%f°\n", (hleg->hservoH->angle*180/M_PI), (hleg->hservoM->angle*180/M_PI), (hleg->hservoL->angle*180/M_PI));
    }
  }
  // Cmd getservo *****
  else if (strncmp(cmd, "getservo", 8) == 0)
  {
    if(robot.current_servo_idx <0)
      HAL_Serial_Print(hserial, "None\n");
    else
      HAL_Serial_Print(hserial, "%s\n", Robot_ServoId[robot.current_servo_idx]);
  }
  // Cmd getpulse *****
  else if (strncmp(cmd, "getpulse", 8) == 0)
  {
    if(robot.current_servo_idx <0)
      HAL_Serial_Print(hserial, "0\n");
    else
      HAL_Serial_Print(hserial, "%d\n", robot.hservos[robot.current_servo_idx]->pulse);
  }
  // Cmd getangle *****
  else if (strncmp(cmd, "getangle", 8) == 0)
  {
    if(robot.current_servo_idx <0)
      HAL_Serial_Print(hserial, "0\n");
    else
      HAL_Serial_Print(hserial, "%f\n", robot.hservos[robot.current_servo_idx]->angle*180/M_PI);
  }
  // Cmd getlimits *****
  else if (strncmp(cmd, "getlimits", 9) == 0)
  {
    if(robot.current_servo_idx <0)
      HAL_Serial_Print(hserial, "0;0;0;0;0\n");
    else
      HAL_Serial_Print(hserial, "%d;%d;%d;%d;%d\n",
        robot.hservos[robot.current_servo_idx]->bmin_pulse,
        robot.hservos[robot.current_servo_idx]->amin_pulse,
        robot.hservos[robot.current_servo_idx]->zero_pulse,
        robot.hservos[robot.current_servo_idx]->amax_pulse,
        robot.hservos[robot.current_servo_idx]->bmax_pulse);
  }
  // Cmd getleg *****
  else if (strncmp(cmd, "getleg", 6) == 0)
  {
    if(robot.current_leg_idx <0)
      HAL_Serial_Print(hserial, "None\n");
    else
      HAL_Serial_Print(hserial, "%s\n", Robot_LegId[robot.current_leg_idx]);
  }
  // Cmd getpos *****
  else if (strncmp(cmd, "getpos", 6) == 0)
  {
    if(robot.current_leg_idx <0)
      HAL_Serial_Print(hserial, "0;0;0\n");
    else
    {
      t_leg *hleg;
      hleg=robot.hlegs[robot.current_leg_idx];
      HAL_Serial_Print(hserial, "%f;%f;%f;\n", hleg->current_x, hleg->current_y, hleg->current_z);
    }
  }
  // Cmd exit/quit *****
  else if ((strncmp(cmd, "exit", 4) == 0) || (strncmp(cmd, "quit", 4) == 0))
  {
    Robot_Fold();
    robot.state=FOLDED;
  }
  // WRONG COMMAND ********
  else
    HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state CONFIGURATION\n", cmd);

}

/**********************************************************
  * @brief Interpret a command
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand(HAL_Serial_Handler * hserial, char *cmd)
{
  if (strcmp(cmd, "getstate") == 0)
  {
    HAL_Serial_Print(hserial, "State %s\n", Robot_GetState());
  }
  //* State FOLDED ****************************************
  else if (robot.state == FOLDED)
  {
    Robot_ReadCommand_Folded(hserial, cmd);
  }
  //* State READY ***************************************
  else if (robot.state == READY)
  {
    Robot_ReadCommand_Ready(hserial, cmd);
  }
  //* State RUNNING ***************************************
  else if (robot.state == RUNNING)
  {
    Robot_ReadCommand_Running(hserial, cmd);
  }
  //* State CONFIGURATION *********************************
  else if (robot.state == CONFIGURATION)
  {
    Robot_ReadCommand_Config(hserial, cmd);
  }
  else
  {
    HAL_Serial_Print(hserial, "KO Unknown state\n");
  }
}

/**********************************************************
 * @brief Interpret button push
 */
void Robot_ButtonPushed()
{
  if(robot.state == FOLDED)
  {
    Robot_Unfold();
    robot.state=READY;
  }
  else if (robot.state == READY)
  {
    // start run after 1sec
    HAL_Delay(1000);
    // robot_update() move legs if in RUNNING state
    robot.state=RUNNING;
  }
  else if (robot.state == RUNNING)
  {
    // robot_update() stop moving legs if not in RUNNING state
    robot.state=READY;
  }
  else if (robot.state == CONFIGURATION)
  {
    robot.state=READY;
  }
}

/**********************************************************
 * @brief Interpret button long push
 */
void Robot_ButtonLongPushed()
{
  if(robot.state == FOLDED)
  {
    Robot_Unfold();
    robot.state=READY;
  }
  else if (robot.state == READY)
  {
    Robot_Fold();
    robot.state=FOLDED;
  }
}



/*
 * robot.c
 *
 *  Created on: 23 d�c. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "serial.h"
#include "tools.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

t_servo servo01;
t_servo servo02;
t_servo servo03;
t_servo servo07;
t_servo servo08;
t_servo servo09;
t_servo servo10;
t_servo servo11;
t_servo servo12;
t_servo servo16;
t_servo servo17;
t_servo servo18;

t_leg legFR;
t_leg legFL;
t_leg legRR;
t_leg legRL;

t_robot robot;

const char *Robot_ServoId[]={"FRH", "FRM", "FRL", "RRH", "RRM", "RRL", "FLH", "FLM", "FLL", "RLH", "RLM", "RLL"};

int sens=1;
extern int on_off;
uint8_t RxBuffer[64];
int RxBuffer_pos=0;

char CmdBuffer[16+1];
int pulse=1500;

int ctr50Hz=0;

void Robot_Init()
{
  int i;
  t_leg *hleg_temp;

  Servo_Init(&servo01, &htim3,  TIM_CHANNEL_4); // Servo connector 01 : tim3, ch4  => RRL, 19
  Servo_Init(&servo02, &htim2,  TIM_CHANNEL_4); // Servo connector 02 : tim2, ch4  => RRM, 12
  Servo_Init(&servo03, &htim2,  TIM_CHANNEL_3); // Servo connector 03 : tim2, ch3  => RRH, 08
  // Servo connector 04 : tim5, ch2 => unused
  // Servo connector 05 : tim3, ch1 => unused
  // Servo connector 06 : tim8, ch2 => unused
  Servo_Init(&servo07, &htim8,  TIM_CHANNEL_3); // Servo connector 07 : tim8, ch3  => RLH, 03
  Servo_Init(&servo08, &htim8,  TIM_CHANNEL_4); // Servo connector 08 : tim8, ch4  => RLM, 11
  Servo_Init(&servo09, &htim1,  TIM_CHANNEL_2); // Servo connector 09 : tim1, ch2  => RLL, 16
  Servo_Init(&servo10, &htim3,  TIM_CHANNEL_3); // Servo connector 10 : tim3, ch3  => FLL, 13
  Servo_Init(&servo11, &htim14, TIM_CHANNEL_1); // Servo connector 11 : tim14, ch1 => FLM, 05
  Servo_Init(&servo12, &htim11, TIM_CHANNEL_1); // Servo connector 12 : tim11, ch1 => FLH, 01
  // Servo connector 13 : tim3, ch2 => unused
  // Servo connector 14 : tim2, ch2 => unused
  // Servo connector 15 : tim1, ch1 => unused
  Servo_Init(&servo16, &htim10, TIM_CHANNEL_1); // Servo connector 16 : tim10, ch1 => FRH, 02
  Servo_Init(&servo17, &htim1,  TIM_CHANNEL_4); // Servo connector 17 : tim1, ch4  => FRM, 09
  Servo_Init(&servo18, &htim1,  TIM_CHANNEL_3); // Servo connector 18 : tim1, ch3  => FRL, 10
  // Servo connector 19 : tim2, ch1 => unused
  // Servo connector 20 : tim8, ch1 => unused
  // Servo connector 21 : tim5, ch1 => unused

  Servo_SetLimits(&servo01, s19_limits);
  Servo_SetLimits(&servo02, s12_limits);
  Servo_SetLimits(&servo03, s08_limits);
  Servo_SetLimits(&servo07, s03_limits);
  Servo_SetLimits(&servo08, s11_limits);
  Servo_SetLimits(&servo09, s16_limits);
  Servo_SetLimits(&servo10, s13_limits);
  Servo_SetLimits(&servo11, s05_limits);
  Servo_SetLimits(&servo12, s01_limits);
  Servo_SetLimits(&servo16, s02_limits);
  Servo_SetLimits(&servo17, s09_limits);
  Servo_SetLimits(&servo18, s10_limits);

  robot.hservos[RRL]=&servo01;
  robot.hservos[RRM]=&servo02;
  robot.hservos[RRH]=&servo03;
  robot.hservos[RLH]=&servo07;
  robot.hservos[RLM]=&servo08;
  robot.hservos[RLL]=&servo09;
  robot.hservos[FLL]=&servo10;
  robot.hservos[FLM]=&servo11;
  robot.hservos[FLH]=&servo12;
  robot.hservos[FRH]=&servo16;
  robot.hservos[FRM]=&servo17;
  robot.hservos[FRL]=&servo18;

  // Initialise legs
  Leg_Init(&legFR, robot.hservos[FRH], robot.hservos[FRM], robot.hservos[FRL], 0);
  Leg_Init(&legFL, robot.hservos[FLH], robot.hservos[FLM], robot.hservos[FLL], 1);
  Leg_Init(&legRR, robot.hservos[RRH], robot.hservos[RRM], robot.hservos[RRL], 0);
  Leg_Init(&legRL, robot.hservos[RLH], robot.hservos[RLM], robot.hservos[RLL], 1);

  robot.hlegs[0]=&legFL;
  robot.hlegs[1]=&legRL;
  robot.hlegs[2]=&legFR;
  robot.hlegs[3]=&legRR;

  // set initial position for legs
  // at 1st update, the array will be rotated, and last leg shall be returning
  // we initialise the positions, and rotate the array in reverse order
  for(i=0;i<NBLEGS;i++)
    Leg_SetPos(robot.hlegs[i], robot.hlegs[i]->origin_x-(i/(NBLEGS-1.))*robot.hlegs[i]->x_amplitude, robot.hlegs[i]->origin_y, LEG_HEIGHT);
  hleg_temp=robot.hlegs[0];
  for(i=0;i<NBLEGS-1;i++)
    robot.hlegs[i]=robot.hlegs[i+1];
  robot.hlegs[NBLEGS-1]=hleg_temp;

  robot.state=FOLDED;
}

/**********************************************************
  * @brief Get servo index from ID
  */
int Robot_ServoId2Idx(char * id)
{
  int i;

  for(i=0;i<3*NBLEGS;i++)
    if(strcmp(id, Robot_ServoId[i]) == 0)
      return i;
  return -1;
}

/**********************************************************
  * @brief  Move legs to start position
  */
void Robot_Unfold()
{
  int i;

  robot.state=WAITING;
  for(i=0;i<NBLEGS;i++)
    Leg_Start(robot.hlegs[i]);

  // TODO : unflod legs
}

/**********************************************************
  * @brief  Move legs to store position
  */
void Robot_Fold()
{
  int i;

  robot.state=FOLDED;

  // TODO : fold legs

  for(i=0;i<NBLEGS;i++)
    Leg_Stop(robot.hlegs[i]);

}

/**********************************************************
  * @brief  Update legs position
  * This function is run every 20ms by Timer 13.
  */
void Robot_Update()
{

  if(robot.state == RUNNING)
  {
    int i;

    if(robot.remsteps_before_return == 0)// new sequence, update return leg
    {
      t_leg *hleg_temp;
      float return_x, return_y; // return vector
      float return_d; // return distance
      float step_dist; // distance at each step

      // rotate legs array so returning leg is at last position
      hleg_temp=robot.hlegs[NBLEGS-1];
      for(i=NBLEGS-2;i>=0;i--)
        robot.hlegs[i+1]=robot.hlegs[i];
      robot.hlegs[0]=hleg_temp;
      hleg_temp=robot.hlegs[NBLEGS-1];

      // Distance for returning leg
      return_x=hleg_temp->origin_x-hleg_temp->x;
      return_y=hleg_temp->origin_y-hleg_temp->y;
      return_d=sqrtf(return_x*return_x+return_x*return_x);
      step_dist=robot.speed*SERVO_PULSE_PERIOD;
      robot.steps_before_return=(int)floorf(hleg_temp->x_amplitude/((NBLEGS-1)*step_dist));
      robot.remsteps_before_return=robot.steps_before_return;
      robot.ret_step_x=return_x/robot.steps_before_return;
      robot.ret_step_y=return_y/robot.steps_before_return;

      // step distance for staight line only
      robot.step_x=-robot.speed*SERVO_PULSE_PERIOD; // legs are going backward if speed is > 0
      robot.step_y=0;
    }
    // Leg steps for forwarding legs
    for(i=0;i<NBLEGS-1;i++)
      Leg_Move(robot.hlegs[i], robot.step_x, robot.step_y, 0);
    // leg step for returning leg
    Leg_Move(robot.hlegs[NBLEGS-1], robot.ret_step_x, robot.ret_step_y, 0);
    // update phase step index
    robot.remsteps_before_return--;

#ifdef TOTO
    // blink led 3
    robot.tim13_nb++;
    if(robot.tim13_nb >25 )
    {
      LedSwitch(3, GPIO_PIN_SET);
    }
    else if (robot.tim13_nb >50)
    {
      LedSwitch(3, GPIO_PIN_RESET);
      robot.tim13_nb=0;
    }
#endif
    /* Toggle LED */
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
  else
  {
    /* Toggle LED */
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }

}

/**********************************************************
  * @brief Interpret a command
  * @hserial serial handler to write status message
  * @param command command to interpret
  */
void Robot_ReadCommand(HAL_Serial_Handler * hserial, char *cmd)
{
  //* State FOLDED ****************************************
  if (robot.state == FOLDED)
  {
    // Cmd unfold ***********
    if (strcmp(cmd, "unfold") == 0)
    {
      Robot_Unfold();
      HAL_Serial_Print(hserial,"OK %s\n", cmd);
    }
    else
      HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state FOLDED\n", cmd);
  }
  //* State WAITING ***************************************
  else if (robot.state == WAITING)
  {
    // Cmd run **************
    if (strcmp(cmd, "run") == 0)
    {
      robot.state=RUNNING;
      HAL_Serial_Print(hserial,"OK %s\n", cmd);
    }
    // Cmd config ***********
    if (strcmp(cmd, "config") == 0)
    {
      robot.state=CONFIGURATION;
      HAL_Serial_Print(hserial,"OK %s\n", cmd);
    }
    // Cmd fold *************
    else if (strcmp(cmd, "fold") == 0)
    {
      robot.state=FOLDED;
      Robot_Fold();
      HAL_Serial_Print(hserial,"OK %s\n", cmd);
    }
    else
      HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state WAITING\n", cmd);
  }
  //* State RUNNING ***************************************
  else if (robot.state == RUNNING)
  {
    if (strncmp(cmd, "speed", 5) == 0)
    {
      int speed;
      if (sscanf(cmd, "speed%d", &speed) != 1)
      {
        HAL_Serial_Print(hserial, "KO Wrong command syntax \"%s\" for command \"select<int>\"\n", cmd);
      }
      else
      {
        robot.speed=(float)speed;
        HAL_Serial_Print(hserial,"OK speed set to %d\n", (int)robot.speed);
      }
    }
    else
      HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state RUNNING\n", cmd);
  }
  //* State CONFIGURATION *********************************
  else if (robot.state == CONFIGURATION)
  {
    // Cmd select servo *****
    if (strncmp(cmd, "select", 6) == 0)
    {
      char servo_id[8];
      if (sscanf(cmd, "select%s", servo_id) != 1)
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
        HAL_Serial_Print(hserial,"OK pulse st to %d\n", pulse);
      }
    }
    else
      HAL_Serial_Print(hserial, "KO Wrong command \"%s\" in state CONFIGURATION\n", cmd);

  }
  else
  {
    HAL_Serial_Print(hserial, "KO Unknown state\n");
  }
}

/**********************************************************
  * @brief Return robot state as string
  */
char * Robot_GetState()
{
  if (robot.state == FOLDED)
    return "FOLDED";
  else if (robot.state == WAITING)
    return "WAITING";
  else if (robot.state == RUNNING)
    return "RUNNING";
  else if (robot.state == CONFIGURATION)
    return "CONFIG";
  else
    return "UNKNOWN";
}
/**********************************************************
  * @brief Interpret button push
  */
void Robot_ButtonPushed()
{
  if(robot.state == FOLDED)
  {
    Robot_Unfold();
  }
  else if (robot.state == WAITING)
  {
    // wait 2 sec before run
    HAL_Delay(2000);
    robot.state=RUNNING;
  }
  else if (robot.state == RUNNING)
  {
    robot.state=WAITING;
  }
  else if (robot.state == CONFIGURATION)
  {
    robot.state=WAITING;
  }
}

void Robot_Autotest()
{
  int i;

  t_servo * hservoH0;
  t_servo * hservoM0;
  t_servo * hservoL0;
  t_servo * hservoH1;
  t_servo * hservoM1;
  t_servo * hservoL1;
  t_servo * hservoH2;
  t_servo * hservoM2;
  t_servo * hservoL2;
  t_servo * hservoH3;
  t_servo * hservoM3;
  t_servo * hservoL3;

  hservoH0=robot.hlegs[0]->hservoH;
  hservoM0=robot.hlegs[0]->hservoM;
  hservoL0=robot.hlegs[0]->hservoL;
  hservoH1=robot.hlegs[1]->hservoH;
  hservoM1=robot.hlegs[1]->hservoM;
  hservoL1=robot.hlegs[1]->hservoL;
  hservoH2=robot.hlegs[2]->hservoH;
  hservoM2=robot.hlegs[2]->hservoM;
  hservoL2=robot.hlegs[2]->hservoL;
  hservoH3=robot.hlegs[3]->hservoH;
  hservoM3=robot.hlegs[3]->hservoM;
  hservoL3=robot.hlegs[3]->hservoL;

  for(i=0;i<=100;i++)
  {
    Servo_SetRatio(hservoH0, i);
    Servo_SetRatio(hservoM0, i);
    Servo_SetRatio(hservoL0, i);
    Servo_SetRatio(hservoH1, i);
    Servo_SetRatio(hservoM1, i);
    Servo_SetRatio(hservoL1, i);
    Servo_SetRatio(hservoH2, i);
    Servo_SetRatio(hservoM2, i);
    Servo_SetRatio(hservoL2, i);
    Servo_SetRatio(hservoH3, i);
    Servo_SetRatio(hservoM3, i);
    Servo_SetRatio(hservoL3, i);
    HAL_Delay(20);
  }
  for(i=100;i>=0;i--)
  {
    Servo_SetRatio(hservoH0, i);
    Servo_SetRatio(hservoM0, i);
    Servo_SetRatio(hservoL0, i);
    Servo_SetRatio(hservoH1, i);
    Servo_SetRatio(hservoM1, i);
    Servo_SetRatio(hservoL1, i);
    Servo_SetRatio(hservoH2, i);
    Servo_SetRatio(hservoM2, i);
    Servo_SetRatio(hservoL2, i);
    Servo_SetRatio(hservoH3, i);
    Servo_SetRatio(hservoM3, i);
    Servo_SetRatio(hservoL3, i);
    HAL_Delay(20);
  }
}



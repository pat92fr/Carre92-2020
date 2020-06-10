#ifndef ROBOT_READCOMMAND_H
#define ROBOT_READCOMMAND_H

void Robot_ReadCommand(HAL_Serial_Handler * hserial, char *cmd);

void Robot_ButtonPushed();
void Robot_ButtonLongPushed();

#endif

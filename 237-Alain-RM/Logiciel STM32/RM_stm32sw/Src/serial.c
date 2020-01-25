/*
 * serial.c
 *
 *  Created on: 27 déc. 2019
 *      Author: alain
 */

#include "stm32f4xx_hal.h"
#include "tools.h"
#include "string.h"
#include "serial.h"
#include "servo.h"
#include "leg.h"
#include "robot.h"
#include "main.h"

__IO ITStatus uart2_ready = RESET;
__IO ITStatus uart4_ready = RESET;

uint8_t uart2_RxByte;
uint8_t uart2_RxBuffer[16];
int     uart2_RxBuffer_len=0;

uint8_t uart4_RxBuffer[16];
int     uart4_RxBuffer_len=0;

#define HUART_MSG huart2
#define UART_MSG_RXBUFFER uart4_RxBuffer
#define UART_MSG_RXBUFFER_LEN uart4_RxBuffer_len

/**********************************************************
  * @brief  Set UART receive IT for command (5 char)
  * @param  huart: UART handle
  * @param  buf : buffer
  * @param  len : buffer length
  * @retval None
  */
void uart2_StartReceiveCmd()
{
  HAL_UART_Receive_IT(&huart2, uart2_RxBuffer, 5);
}

/**********************************************************
  * @brief  Set UART receive IT for servo conf (8 char)
  * @param  huart: UART handle
  * @param  buf : buffer
  * @param  len : buffer length
  * @retval None
  */
void uart2_StartReceiveServoConf()
{
  HAL_UART_Receive_IT(&huart2, uart2_RxBuffer, 8);
}

/**********************************************************
  * @brief  UART2 reception buffer length
  * @retval Reception buffer length
  */
int uart2_ReceivedLength()
{
  return uart2_RxBuffer_len;
}

/**********************************************************
  * @brief  Get command from UART2 buffer
  * @param  buffer : destination buffer, must be >5 bytes long
  * @retval Number of byte reveived
  */
void uart2_GetCmd(uint8_t * dest_buffer)
{
  memcpy(dest_buffer, uart2_RxBuffer,   uart2_RxBuffer_len);
  uart2_RxBuffer_len=0;
  LedSwitch(3,GPIO_PIN_SET);
}

/**********************************************************
  * @brief  Send through UART2
  * @param  source_buffer : data to transmit
  * @param  len : buffer length
  * @retval None
  */
void uart2_Transmit(uint8_t * source_buffer, int len)
{
  HAL_UART_Transmit(&huart2, source_buffer, len, HAL_MAX_DELAY);
}

/**********************************************************
  * @brief  Send message through message UART
  * @param  source_buffer : data to transmit
  * @param  len : buffer length
  * @retval None
  */
void Message_Send(uint8_t * source_buffer, int len)
{
  HAL_UART_Transmit(&HUART_MSG, source_buffer, len, HAL_MAX_DELAY);
}

/**********************************************************
  * @brief  Prepare UART to receive n bytes
  * @param  buf : buffer
  * @param  n : number of bytes to receive
  * @retval None
  */
void Message_StartReceive(int n)
{
  HAL_UART_Receive_IT(&HUART_MSG, UART_MSG_RXBUFFER, n);

}

/**********************************************************
  * @brief  Length of UART received data
  * @retval Number of received bytes
  */
int Message_ReceivedLength()
{
  return UART_MSG_RXBUFFER_LEN;
}


/**********************************************************
  * @brief  Get UART received data
  * @param  buf : buffer
  * @param  n : number of bytes to receive
  * @retval None
  */
void Message_Get(uint8_t * buffer)
{
  memcpy(buffer, UART_MSG_RXBUFFER,   UART_MSG_RXBUFFER_LEN);
  UART_MSG_RXBUFFER_LEN=0;
  LedSwitch(3,GPIO_PIN_SET);
}


/**********************************************************
  * @brief  Period elapsed callback in non blocking mode
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart2_ready = SET;
    uart2_RxBuffer_len=5;
    LedSwitch(3,GPIO_PIN_RESET);
  }
  if (huart->Instance == UART4)
  {
    uart4_ready = SET;
    uart4_RxBuffer_len=5;
    LedSwitch(3,GPIO_PIN_RESET);
  }
}

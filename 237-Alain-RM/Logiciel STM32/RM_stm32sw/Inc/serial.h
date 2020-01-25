/*
 * serial.h
 *
 *  Created on: 27 déc. 2019
 *      Author: alain
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#define size_of_tx_pool 32
#define size_of_tx_buffer 256
#define size_of_rx_circular_buffer 2048

/*
#define size_of_tx_pool 32
#define size_of_tx_buffer 1600
#define size_of_rx_circular_buffer 32000
*/

typedef struct
{
  uint8_t data[size_of_tx_buffer];
  uint32_t length;
} tx_buffer;

typedef struct
{
    UART_HandleTypeDef * huart;

    tx_buffer tx_buffer_pool[size_of_tx_pool];
    uint32_t tx_head_position;
    uint32_t tx_tail_position;
    volatile uint32_t tx_dma;

    uint8_t rx_circular_buffer[size_of_rx_circular_buffer];
    uint8_t const * rx_tail_ptr;

} HAL_DMASerial_Handler;


extern __IO ITStatus uart2_ready;
extern uint8_t uart2_RxBuffer[16];
extern int     uart2_RxBuffer_len;

void uart2_StartReceiveCmd();
void uart2_StartReceiveServoConf();
int uart2_ReceivedLength();
void uart2_GetCmd(uint8_t * dest_buffer);
void uart2_Transmit(uint8_t * source_buffer, int len);

void Message_Send(uint8_t * source_buffer, int len);
void Message_StartReceive(int n);
int Message_ReceivedLength();
void Message_Get(uint8_t * buffer);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif /* SERIAL_H_ */

#include "uart_bsp.h"
#include "pc_mcu_uart.h"
#include "string.h"
#include "usart.h"
#include "dji_ndj_remote.h"

//#define SBUS_HEAD 0X0F
//#define SBUS_END 0X00
#define JOYSTICK_OFFSET 1024

uint8_t uart5_rx_buff[BUFF_SIZE];
extern dji_ndj6 dji_remote;
int16_t test1;

double received_value = 0.0; // Global variable to store received double from PC
uint8_t rx_buffer_uart10[sizeof(double)]; // Buffer for UART10 reception

//void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
//{
////	test1 = (buf[0] | buf[1] << 8) & 0x07FF;
////	test1 -= 1024;
//
////    if ((buf[0] != SBUS_HEAD) || (buf[17] != SBUS_END))
////        return;
////
////    if (buf[16] == 0x0C)
////        remoter->online = 0;
////    else
////        remoter->online = 1;
//	remoter->ndj6.right_x = (buf[0] | buf[1] << 8) & 0x07FF;
//	remoter->ndj6.right_x -= JOYSTICK_OFFSET;
//	remoter->ndj6.right_y = (buf[1] >> 3 | buf[2] << 5) & 0x07FF;
//	remoter->ndj6.right_y -= JOYSTICK_OFFSET;
//	remoter->ndj6.left_x = (buf[2] >> 6 | buf[3] << 2
//			| buf[4] << 10) & 0x07FF;
//	remoter->ndj6.left_x -= JOYSTICK_OFFSET;
//	remoter->ndj6.left_y = (buf[4] >> 1 | buf[5] << 7) & 0x07FF;
//	remoter->ndj6.left_y -= JOYSTICK_OFFSET;
//	//Left switch position
//	remoter->ndj6.left_switch = ((buf[5] >> 4) & 0x000C) >> 2;
//	remoter->ndj6.right_switch = (buf[5] >> 4) & 0x0003;
//	remoter->ndj6.mouse_x = ((int16_t) buf[6] | ((int16_t) buf[7] << 8));
//	remoter->ndj6.mouse_y = ((int16_t) buf[8] | ((int16_t) buf[9] << 8));
//	remoter->ndj6.mouse_z = ((int16_t) buf[10] | ((int16_t) buf[11] << 8));
//	remoter->ndj6.mouse_hori += remoter->ndj6.mouse_x;
//	remoter->ndj6.mouse_vert += remoter->ndj6.mouse_y;
//	remoter->ndj6.mouse_left = (buf[12]);
//	remoter->ndj6.mouse_right = (buf[13]);
//	remoter->ndj6.keyboard_keys = (buf[14]);
//	remoter->ndj6.side_dial = ((int16_t) buf[16]) | ((int16_t) buf[17] << 8);
//	remoter->ndj6.side_dial -= JOYSTICK_OFFSET;
//	remoter->ndj6.last_time = HAL_GetTick();
////    remoter->ndj6.right_x = ((buf[0] | buf[1] << 8) & 0x07FF);
////    remoter->ndj6.right_y = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
////    remoter->rc.ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
////    remoter->rc.ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
////    remoter->rc.ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
////    remoter->rc.ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
////    remoter->rc.ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
////    remoter->rc.ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
////    remoter->rc.ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
////    remoter->rc.ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		if (Size <= BUFF_SIZE)
		{
//			sbus_frame_parse(&remoter, uart5_rx_buff);
			DJI_NDJ_REMOTE_PROCESS(&dji_remote, uart5_rx_buff);
		}
		else  // If received data length is greater than BUFF_SIZE, clear buffer
		{	
			memset(uart5_rx_buff, 0, BUFF_SIZE);
		}
		// Always restart UART reception after processing data
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_buff, BUFF_SIZE*2);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		memset(uart5_rx_buff, 0, BUFF_SIZE);							   // Clear buffer
		// Restart UART reception after error
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_buff, BUFF_SIZE*2);
	}
	else if(huart->Instance == USART10)
	{
		// Clear any error flags
		__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
		
		// Re-arm the UART receive
		HAL_UART_Receive_IT(&huart10, rx_buffer, MCU_MSG_SIZE);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5)
    {
        // Process the received data for UART5
        DJI_NDJ_REMOTE_PROCESS(&dji_remote, uart5_rx_buff);

        // Clear the buffer
//        memset(uart5_rx_buff, 0, BUFF_SIZE);

        // Restart reception for UART5
//        HAL_UART_Receive_IT(&huart5, uart5_rx_buff, BUFF_SIZE);
    }
    else if(huart->Instance == USART10)
    {
        // Copy data to structure first
        memcpy(&armor_data, rx_buffer, NUM_FLOATS * 4);
        
        // Then copy to float array
        memcpy(pc_mcu_rx_data, rx_buffer, NUM_FLOATS * 4);
        
        // Re-arm receive AFTER processing the data
        HAL_UART_Receive_IT(&huart10, rx_buffer, MCU_MSG_SIZE);
    }
}

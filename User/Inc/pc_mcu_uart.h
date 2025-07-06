/*
 * pc_mcu_uart.h
 *
 *  Created on: Jun 7, 2025
 *      Author: YI MING
 */

#ifndef INC_PC_MCU_UART_H_
#define INC_PC_MCU_UART_H_

#include <stdint.h>  // For uint8_t, uint16_t types

#ifdef __cplusplus
extern "C" {
#endif

// PC sends 7 floats to MCU
#define PC_TO_MCU_FLOATS 7
#define PC_TO_MCU_MSG_SIZE (PC_TO_MCU_FLOATS * 4 + 2) // 7 floats + 2 bytes CRC = 30 bytes

// MCU sends 25 floats to PC
#define MCU_TO_PC_FLOATS 25
#define MCU_TO_PC_MSG_SIZE (MCU_TO_PC_FLOATS * 4 + 2) // 25 floats + 2 bytes CRC = 102 bytes

// *** From MCU's perspective ***
// pc_mcu_tx_data = Data that MCU TRANSMITS to PC (MCU → PC)
extern float pc_mcu_tx_data[MCU_TO_PC_FLOATS]; // MCU sends 25 floats to PC

// pc_mcu_rx_data = Data that MCU RECEIVES from PC (PC → MCU)
extern float pc_mcu_rx_data[PC_TO_MCU_FLOATS]; // MCU receives 7 floats from PC

// UART buffers (from MCU's perspective)
extern uint8_t tx_buffer[MCU_TO_PC_MSG_SIZE];  // MCU transmit buffer (102 bytes)
extern uint8_t rx_buffer[PC_TO_MCU_MSG_SIZE];  // MCU receive buffer (30 bytes)

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

void PC_MCU_UART_TASK(void);
void PC_MCU_UART_Process_Received_Data(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_PC_MCU_UART_H_ */

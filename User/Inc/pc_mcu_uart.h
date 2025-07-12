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

// Error handler structure
typedef struct {
    uint32_t total_received;        // Total messages received
    uint32_t crc_errors;           // Total CRC errors
    uint32_t consecutive_errors;   // Consecutive CRC errors
    uint32_t recovery_count;       // Number of times recovery was performed
    uint32_t last_receive_time;    // Last successful receive time
    uint8_t error_state;           // Error state flag
    uint32_t total_transmissions;  // Total transmission attempts
    uint32_t skipped_transmissions;// Transmissions skipped due to busy UART
} uart_error_handler_t;

// Function declarations
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

void PC_MCU_UART_TASK(void);
void PC_MCU_UART_Process_Received_Data(void);
void UART_Error_Recovery(void);
void Check_Receive_Timeout(void);

// UART Communication Status Functions (for buzzer alerts)
uint8_t PC_MCU_UART_Is_Connected(void);
uint32_t PC_MCU_UART_Get_Time_Since_Last_Receive(void);
uart_error_handler_t* PC_MCU_UART_Get_Error_Stats(void);

// UART connection timeout threshold (in milliseconds)
#define UART_CONNECTION_TIMEOUT_MS  500  // 3 seconds to declare disconnected

#ifdef __cplusplus
}
#endif
#endif /* INC_PC_MCU_UART_H_ */

/*
 * pc_mcu_uart.c
 *
 *  Created on: Jun 7, 2025
 *      Author: YI MING
 */
#include "pc_mcu_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"

// *** From MCU's perspective ***
// pc_mcu_tx_data = Data that MCU TRANSMITS to PC (MCU → PC)
float pc_mcu_tx_data[MCU_TO_PC_FLOATS] = {0};  // MCU sends 25 floats to PC

// pc_mcu_rx_data = Data that MCU RECEIVES from PC (PC → MCU)
float pc_mcu_rx_data[PC_TO_MCU_FLOATS] = {0};  // MCU receives 7 floats from PC

// UART buffers (from MCU's perspective)
uint8_t tx_buffer[MCU_TO_PC_MSG_SIZE];     // MCU transmit buffer: 102 bytes (25 floats + CRC)
uint8_t rx_buffer[PC_TO_MCU_MSG_SIZE];     // MCU receive buffer: 30 bytes (7 floats + CRC)


void MCU_TO_PC_DATA_ASSIGN(){
	// Add some specific test values to MCU transmission
	pc_mcu_tx_data[0] = 1.2f;
	pc_mcu_tx_data[1] += 0.001f;
	pc_mcu_tx_data[2] = 2.2f;
	pc_mcu_tx_data[3] += 0.001f;
	pc_mcu_tx_data[4] = 3.2f;
	pc_mcu_tx_data[5] += 0.001f;
	pc_mcu_tx_data[6] = 5.2f;
    pc_mcu_tx_data[7] += 0.001f;
    pc_mcu_tx_data[8] = 6.2f;
    pc_mcu_tx_data[9] += 0.001f;
    pc_mcu_tx_data[10] = 7.2f;
    pc_mcu_tx_data[11] += 0.001f;
    pc_mcu_tx_data[12] = 8.2f;
    pc_mcu_tx_data[13] += 0.001f;
    pc_mcu_tx_data[14] = 9.2f;
    pc_mcu_tx_data[15] += 0.001f;
    pc_mcu_tx_data[16] = 10.2f;
    pc_mcu_tx_data[17] += 0.001f;
    pc_mcu_tx_data[18] = 11.2f;
    pc_mcu_tx_data[19] += 0.001f;
    pc_mcu_tx_data[20] = 12.2f;
    pc_mcu_tx_data[21] += 0.001f;
    pc_mcu_tx_data[22] = 13.2f;
    pc_mcu_tx_data[23] += 0.001f;
    pc_mcu_tx_data[24] = 14.2f;
}

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        crc &= 0xFFFF;
    }
    return crc;
}

void PC_MCU_UART_Process_Received_Data(void) {
    // Process data that MCU RECEIVED from PC (7 floats)
    uint16_t received_crc = (uint16_t)rx_buffer[PC_TO_MCU_FLOATS * 4] | 
                           ((uint16_t)rx_buffer[PC_TO_MCU_FLOATS * 4 + 1] << 8);
    uint16_t calculated_crc = crc16_ccitt(rx_buffer, PC_TO_MCU_FLOATS * 4);
    
    if (received_crc == calculated_crc) {
        // CRC OK - Copy received floats to pc_mcu_rx_data[] (MCU received data)
        memcpy(pc_mcu_rx_data, rx_buffer, PC_TO_MCU_FLOATS * 4);
    }
    
    // Re-arm UART receive for next message from PC
    HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);
}

void PC_MCU_UART_TASK(void) {
    // Start first receive (expecting 7 floats from PC)
    HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);

    for (;;) {
        // Prepare data that MCU will TRANSMIT to PC (25 floats)
        MCU_TO_PC_DATA_ASSIGN();
        memcpy(tx_buffer, pc_mcu_tx_data, MCU_TO_PC_FLOATS * 4);
        uint16_t crc = crc16_ccitt(tx_buffer, MCU_TO_PC_FLOATS * 4);
        tx_buffer[MCU_TO_PC_FLOATS * 4] = crc & 0xFF;
        tx_buffer[MCU_TO_PC_FLOATS * 4 + 1] = (crc >> 8) & 0xFF;

        // MCU TRANSMITS message to PC (25 floats)
        HAL_UART_Transmit_IT(&huart10, tx_buffer, MCU_TO_PC_MSG_SIZE);

        // Re-arm UART receive periodically to ensure it's always active
        HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);

        osDelay(5); // Send every 5ms
    }
}

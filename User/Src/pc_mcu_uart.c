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

float pc_mcu_tx_data[NUM_FLOATS] = {0};
float pc_mcu_rx_data[NUM_FLOATS] = {0};
armorPlate_t armor_data = {0};  // Initialize armor data structure

// Use MCU_MSG_SIZE for buffer sizes
uint8_t tx_buffer[MCU_MSG_SIZE];
uint8_t rx_buffer[MCU_MSG_SIZE];

void MCU_TO_PC_DATA_ASSIGN(){
	pc_mcu_tx_data[0] = 1.2f;
	pc_mcu_tx_data[1] += 1.2f;
	pc_mcu_tx_data[2] = 2.2f;
	pc_mcu_tx_data[3] += 2.2f;
	pc_mcu_tx_data[4] = 3.2f;
	pc_mcu_tx_data[5] += 4.2f;
	pc_mcu_tx_data[6] = 5.2f;
	pc_mcu_tx_data[7] += 5.2f;
	pc_mcu_tx_data[8] = 6.2f;
	pc_mcu_tx_data[9] += 6.2f;
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

void PC_MCU_UART_TASK(void) {
    // Start first receive
    HAL_UART_Receive_IT(&huart10, rx_buffer, MCU_MSG_SIZE);

    for (;;) {
        // Pack floats into tx_buffer
        MCU_TO_PC_DATA_ASSIGN();
        memcpy(tx_buffer, pc_mcu_tx_data, NUM_FLOATS * 4);
        uint16_t crc = crc16_ccitt(tx_buffer, NUM_FLOATS * 4);
        tx_buffer[NUM_FLOATS * 4] = crc & 0xFF;
        tx_buffer[NUM_FLOATS * 4 + 1] = (crc >> 8) & 0xFF;

        // Send message
        HAL_UART_Transmit_IT(&huart10, tx_buffer, MCU_MSG_SIZE);

        osDelay(1000); // Send every 1 second
    }
}

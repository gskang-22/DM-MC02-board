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

// Armor plate data structure
typedef struct {
    float tvec_y;
    float tvec_z;
    float is_detected;
    float armor_length;
    float armor_width;
    float orientation;
    float delta_x;
    float delta_y;
} armorPlate_t;

#define NUM_FLOATS 8  // Updated to match armor plate structure
#define MCU_MSG_SIZE (NUM_FLOATS * 4) // 8 floats + 2 bytes CRC

extern float pc_mcu_tx_data[NUM_FLOATS]; // User: assign floats to send
extern float pc_mcu_rx_data[NUM_FLOATS]; // User: read floats received
extern armorPlate_t armor_data;  // Structure to store received armor data

// Add these declarations
extern uint8_t rx_buffer[MCU_MSG_SIZE];
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len);

void PC_MCU_UART_TASK(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_PC_MCU_UART_H_ */

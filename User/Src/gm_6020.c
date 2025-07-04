/*
 * gm_6020.c
 *
 *  Created on: Jun 11, 2025
 *      Author: YI MING
 */
#include "gm_6020.h"
#include <stdint.h>
#include "fdcan.h"
#include "bsp_fdcan.h"

gm_6020 GM_motor1;
gm_6020 GM_motor2;
gm_6020 GM_motor3;
gm_6020 GM_motor4;
gm_6020 GM_motor5;
gm_6020 GM_motor6;
gm_6020 GM_motor7;

void fbdata_gm6020(gm_6020 *can_motor_data, uint16_t motor_id, uint8_t *rx_buffer) {

//	uint16_t idnum = motor_id - 0x200;
		//convert the raw data back into the respective values
	can_motor_data->id = motor_id;
	can_motor_data->raw_angle[1] = can_motor_data->raw_angle[0];
	can_motor_data->raw_angle[0] = (rx_buffer[0] << 8) | rx_buffer[1];
	can_motor_data->raw_rpm = (rx_buffer[2] << 8) | rx_buffer[3];
	can_motor_data->torque = (rx_buffer[4] << 8) | rx_buffer[5];
	can_motor_data->temp = (rx_buffer[6]);
}

void send_can_gm_6020() {
    uint8_t tx_data_1ff[8] = {0};
    uint8_t tx_data_2ff[8] = {0};

    // Motors 1-4 (CAN ID 0x1FF)
    tx_data_1ff[0] = (GM_motor1.current_output >> 8) & 0xFF; // High byte
    tx_data_1ff[1] = GM_motor1.current_output & 0xFF;        // Low byte
    tx_data_1ff[2] = (GM_motor2.current_output >> 8) & 0xFF;
    tx_data_1ff[3] = GM_motor2.current_output & 0xFF;
    tx_data_1ff[4] = (GM_motor3.current_output >> 8) & 0xFF;
    tx_data_1ff[5] = GM_motor3.current_output & 0xFF;
    tx_data_1ff[6] = (GM_motor4.current_output >> 8) & 0xFF;
    tx_data_1ff[7] = GM_motor4.current_output & 0xFF;
    fdcanx_send_data(&hfdcan1, 0x1FF, tx_data_1ff, 8);

    // Motors 5-7 (CAN ID 0x2FF)
    tx_data_2ff[0] = (GM_motor5.current_output >> 8) & 0xFF;
    tx_data_2ff[1] = GM_motor5.current_output & 0xFF;
    tx_data_2ff[2] = (GM_motor6.current_output >> 8) & 0xFF;
    tx_data_2ff[3] = GM_motor6.current_output & 0xFF;
    tx_data_2ff[4] = (GM_motor7.current_output >> 8) & 0xFF;
    tx_data_2ff[5] = GM_motor7.current_output & 0xFF;
    tx_data_2ff[6] = 0; // Null
    tx_data_2ff[7] = 0; // Null
    fdcanx_send_data(&hfdcan1, 0x2FF, tx_data_2ff, 8);
}

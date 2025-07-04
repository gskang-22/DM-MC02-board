/*
 * gm_6020.h
 *
 *  Created on: Jun 11, 2025
 *      Author: YI MING
 */

#ifndef INC_GM_6020_H_
#define INC_GM_6020_H_

#include <stdint.h>

typedef struct {
	uint16_t id;
	uint8_t motor_id;
	int32_t raw_angle[2]; //[0]=latest [1]=last time
	int64_t rad;
	int32_t continuous_rad;
	int32_t zero_offset; //offset to set zero cause this motor cant set zero
	int16_t raw_rpm;
	int16_t raw_rad_per_sec;
	int16_t torque;
	uint8_t temp;
	int32_t current_output;
	int32_t voltage_output;
	uint8_t ping;
	uint8_t online;
} gm_6020;

void send_can_gm_6020();

#endif /* INC_GM_6020_H_ */

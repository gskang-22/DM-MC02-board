/*
 * j60_10.h
 *
 *  Created on: Jul 4, 2025
 *      Author: YI MING
 */

#ifndef INC_J60_10_H_
#define INC_J60_10_H_

#include <stdint.h>

#define MAP_F32_TO_U16(val, min, max) ((uint16_t)(((val) - (min)) * 65535.0f / ((max) - (min))))
#define MAP_F32_TO_U14(val, min, max) ((uint16_t)(((val) - (min)) * 16383.0f / ((max) - (min))))
#define MAP_F32_TO_U10(val, min, max) ((uint16_t)(((val) - (min)) * 1023.0f / ((max) - (min))))
#define MAP_F32_TO_U8(val, min, max)  ((uint8_t)(((val) - (min)) * 255.0f / ((max) - (min))))

// J60 Motor feedback data ranges (from documentation)
#define J60_POS_MIN     (-40.0f)        // -40 rad
#define J60_POS_MAX     (40.0f)         // +40 rad
#define J60_VEL_MIN     (-40.0f)        // -40 rad/s  
#define J60_VEL_MAX     (40.0f)         // +40 rad/s
#define J60_TOR_MIN     (-40.0f)        // -40 N·m
#define J60_TOR_MAX     (40.0f)         // +40 N·m
#define J60_TEMP_MIN    (-20.0f)        // -20°C
#define J60_TEMP_MAX    (200.0f)        // +200°C

// J60 Motor feedback data unpacking macros
#define MAP_U20_TO_F32(val, min, max) ((float)(val) * ((max) - (min)) / 1048575.0f + (min))
#define MAP_U16_TO_F32(val, min, max) ((float)(val) * ((max) - (min)) / 65535.0f + (min))
#define MAP_U7_TO_F32(val, min, max)  ((float)(val) * ((max) - (min)) / 127.0f + (min))


typedef struct
{
	int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	float temperature;
	float torque;
	float speed;
	float encoder_angle;
	float previous_angle; // Store the last encoder angle for this motor
	int rotations;
	int heartbeat;
	int ping;
	int online;
	uint8_t enable_failed;
	uint8_t safety_stop; // Safety stop flag when torque exceeds limit
	// Update frequency tracking
	uint32_t update_count;       // Count of updates received
	uint32_t last_update_time;   // Timestamp of last update (in ms)
	uint32_t update_interval_sum; // Sum of intervals between updates (in ms)
	float update_frequency;      // Calculated update frequency (Hz)
}motor_fbpara_t;

typedef struct
{
	float pos_set;
	float vel_set;
	float kp_set;
	float kd_set;
	float tor_set;
}motor_ctrl_t;

typedef struct
{
	int16_t id;
	uint8_t can_number;
	motor_fbpara_t para;
	motor_ctrl_t cmd;
	float rad_offset;
	int8_t direction;
	float pos_limit_min;   // Minimum position limit in radians
	float pos_limit_max;   // Maximum position limit in radians
	uint8_t limit_enabled; // Flag to enable/disable position limits
}motor_t;

void Enable_J60_Motor(motor_t *motor);
void Init_J60_Motor(motor_t *motor, int16_t motor_id, uint8_t can_channel, float rad_offset, int8_t direction, float pos_limit_min, float pos_limit_max);
void Send_J60_Motor_Command(motor_t *motor);
float Apply_Position_Limits(motor_t *motor, float position);
void J60_Process_Feedback(motor_t *motor, uint8_t *rx_data);
void J60_UintsToFloats(uint8_t *data_recv, float *float_data);
void J60_Enable_Feedback(motor_t *motor, uint8_t *rx_data);
void J60_Cmd_Clear(motor_t *motor);
uint8_t J60_Safety_Check(motor_t *motor, float torque_limit);
void J60_Safety_Reset(motor_t *motor);
void J60_Set_Position_Limits(motor_t *motor, float min_pos, float max_pos);
void J60_Enable_Position_Limits(motor_t *motor, uint8_t enable);

#endif /* INC_J60_10_H_ */

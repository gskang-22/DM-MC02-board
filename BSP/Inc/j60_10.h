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
	int8_t disconnect_time;
	int online;
	float pos_predict;

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
}motor_t;

void Enable_J60_Motor(motor_t *motor);
void Init_J60_Motor(motor_t *motor, int16_t motor_id, uint8_t can_channel);
void Send_J60_Motor_Command(motor_t *motor);
void J60_Process_Feedback(motor_t *motor, uint8_t *rx_data);
void J60_UintsToFloats(uint8_t *data_recv, float *float_data);

#endif /* INC_J60_10_H_ */

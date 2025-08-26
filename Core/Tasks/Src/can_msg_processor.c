/*
 * can_msg_processor.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "bsp_lk_motor.h"
#include "supercap_comm_task.h"

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;
#define ANGLE_LPF 0
#define SPEED_LPF 0

#ifndef CHASSIS_MCU
extern motor_data_t g_can_motors[24];

motor_map_t lk_motor_map[65];
motor_map_t dji_motor_map[25];
//where is this number from lmao
motor_map_t dm_motor_map[15];
extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

#else
motor_data_t g_can_motors[12];
#endif

void map_lk_motor(uint16_t motor_id, motor_data_t *motor_data) {
	if (motor_id > 0x140 && motor_id <= 0x160) {
		lk_motor_map[motor_id - 0x140].motor_data = motor_data;
		lk_motor_map[motor_id - 0x140].motor_id = motor_id;
	}
}

void map_dji_motor(uint16_t motor_id, motor_data_t *motor_data) {
	if (motor_id <= 24) {
		dji_motor_map[motor_id].motor_id = motor_id;
		dji_motor_map[motor_id].motor_data = motor_data;
	}
}

/**
 * CAN ISR function, triggered upon RX_FIFO0_MSG_PENDING
 * converts the raw can data to the motor_data struct form as well
 */
void can_ISR(FDCAN_HandleTypeDef *hfdcan) {
	FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[CAN_BUFFER_SIZE];

	/* CAN1 handling */
	if (hfdcan->Instance == FDCAN1) {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
				== HAL_OK) {
			if (RxHeader.Identifier >= 0x200 && RxHeader.Identifier <= 0x20E) {
				if (dji_motor_map[RxHeader.Identifier - 0x200].motor_data
						!= NULL) {
					convert_raw_can_data(
							dji_motor_map[RxHeader.Identifier - 0x200].motor_data,
							RxHeader.Identifier, RxData);
				}
			} else if (RxHeader.Identifier > 0x140
					&& RxHeader.Identifier <= 0x160) {
				if (lk_motor_map[RxHeader.Identifier - 0x140].motor_data != NULL) {
					process_lk_motor(RxData,
							lk_motor_map[RxHeader.Identifier - 0x140].motor_data);
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
							&xHigherPriorityTaskWoken);
				}
			}
#ifdef SUPERCAP_PRESENT
			else if (RxHeader.Identifier == DEVC_NODE_ID) {
				supercapISR(RxData);
			}
#endif
			else {
				int fb_id = (RxData[0]) & 0x0F;
				switch (fb_id) {
				case 1:
				case 5:
					dm4310_fbdata(&dm_pitch_motor, RxData);
					break;
				}
			}
		}
	}
	/* CAN2 handling */
	if (hfdcan->Instance == FDCAN2) {
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, RxData)
				== HAL_OK) {
			if (RxHeader.Identifier >= 0x200 && RxHeader.Identifier <= 0x210) {
				if (dji_motor_map[RxHeader.Identifier - 0x200 + 12].motor_data
						!= NULL) {
					convert_raw_can_data(
							dji_motor_map[RxHeader.Identifier - 0x200 + 12].motor_data,
							RxHeader.Identifier + 12, RxData);
				}
			} else if (RxHeader.Identifier > 0x140
					&& RxHeader.Identifier <= 0x160) {
				if (lk_motor_map[RxHeader.Identifier - 0x140].motor_data != NULL) {
					process_lk_motor(RxData,
							lk_motor_map[RxHeader.Identifier - 0x140].motor_data);
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
							&xHigherPriorityTaskWoken);
				}
			}
#ifdef SUPERCAP_PRESENT
			else if (RxHeader.Identifier == DEVC_NODE_ID) {
				supercapISR(RxData);
			}
#endif
			else {
				int fb_id = (RxData[0]) & 0x0F;
				switch (fb_id) {
				case 1:
					dm4310_fbdata(&dm_yaw_motor, RxData);
					break;
				}
			}
		}
	}
}
void map_dm_motor(uint16_t motor_id, motor_data_t *motor_data) {
	if (motor_id > 0x200 && motor_id <= 0x20E) {
		dm_motor_map[motor_id - 0x200].motor_id = motor_id;
		dm_motor_map[motor_id - 0x200].motor_data = motor_data;
	}
}

/*
 * Converts raw CAN data over to the motor_data_t struct
 * 7 bytes of CAN data is sent from the motors:
 * High byte for motor angle data
 * Low byte for motor angle data
 * High byte for RPM
 * Low byte for RPM
 * High byte for Torque
 * Low byte for Torque
 * 1 byte for temperature
 *
 * This function combines the respective high and low bytes into 1 single 16bit integer, then stores them
 * in the struct for the motor.
 *
 * For GM6020 motors, it recenters the motor angle data and converts it to radians.
 */

void convert_raw_can_data(motor_data_t *can_motor_data, uint16_t motor_id,
		uint8_t *rx_buffer) {
	uint16_t idnum = motor_id - 0x200;

	//if idnum > 24, it's not a DJI motor. Add in a seperate processing function if other CAN devices are added
	if (idnum > 24) {
		return;
	}
	motor_data_t *curr_motor = can_motor_data;
	//convert the raw data back into the respective values
	curr_motor->id = motor_id;
	curr_motor->raw_data.angle[1] = curr_motor->raw_data.angle[0];
	curr_motor->raw_data.angle[0] = (rx_buffer[0] << 8) | rx_buffer[1];
	int16_t temp_rpm = (rx_buffer[2] << 8) | rx_buffer[3];
	curr_motor->raw_data.rpm = curr_motor->raw_data.rpm * SPEED_LPF
			+ temp_rpm * (1 - SPEED_LPF);
	curr_motor->raw_data.torque = (rx_buffer[4] << 8) | rx_buffer[5];
	curr_motor->raw_data.temp = (rx_buffer[6]);
	curr_motor->last_time[1] = curr_motor->last_time[0];
	curr_motor->last_time[0] = get_microseconds();

	float rds_passed = (float) (curr_motor->raw_data.angle[0]
			- curr_motor->raw_data.angle[1]) / 8192;
	float time_diff = (float) (curr_motor->last_time[0]
			- curr_motor->last_time[1]) / (float) (TIMER_FREQ * 60);
	curr_motor->angle_data.hires_rpm = curr_motor->angle_data.hires_rpm * 0.95
			+ (rds_passed * time_diff * 0.05);
	//process the angle data differently depending on the motor type to get radians in the
	//adj_angle value

	//motor must be initialised in motor_config.c first
	if (curr_motor->motor_type > 0) {
		switch (curr_motor->motor_type) {
		case TYPE_GM6020:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		case TYPE_M2006:
		case TYPE_M3508:
			break;
		case TYPE_M2006_STEPS:
		case TYPE_M3508_STEPS:
//			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
//					curr_motor->last_time);
			break;
		case TYPE_M2006_ANGLE:
		case TYPE_M3508_ANGLE:
		case TYPE_GM6020_720:
			motor_calc_odometry(&curr_motor->raw_data, &curr_motor->angle_data,
					curr_motor->last_time);
			angle_offset(&curr_motor->raw_data, &curr_motor->angle_data);
			break;
		default:
			break;

		}

		//initialise task switching variables
		BaseType_t xHigherPriorityTaskWoken, xResult;
		xHigherPriorityTaskWoken = pdFALSE;

		//set event group bits so that the tasks and PIDs only trigger upon updated data
		//also checks if the respective tasks are set to ready
		switch (idnum) {
#ifndef CHASSIS_MCU
		case FR_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b1000,
					&xHigherPriorityTaskWoken);
			break;
		case FL_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0100,
					&xHigherPriorityTaskWoken);
			break;
		case BL_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0010,
					&xHigherPriorityTaskWoken);
			break;
		case BR_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(chassis_event_group, 0b0001,
					&xHigherPriorityTaskWoken);
			break;
#endif
		case LFRICTION_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b00010,
					&xHigherPriorityTaskWoken);
			break;
		case RFRICTION_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b00001,
					&xHigherPriorityTaskWoken);
			break;
		case FEEDER_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b00100,
					&xHigherPriorityTaskWoken);
			break;
#ifdef ACTIVE_GUIDANCE
		case BFRICTION_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b01000,
					&xHigherPriorityTaskWoken);
			break;
		case GFRICTION_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(launcher_event_group, 0b10000,
					&xHigherPriorityTaskWoken);
			break;
#endif
		case PITCH_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
					&xHigherPriorityTaskWoken);
			break;
		case YAW_MOTOR_ID:
			xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b10,
					&xHigherPriorityTaskWoken);
			break;
		default:
			xResult = pdFAIL;
			//error handler
			break;
		}

		//switches tasks if a higher priority task is ready.
		//required because the function is in an ISR
		if (xResult != pdFAIL) {
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //forces current task to yield if higher priority task is called
		}
	} else {
		//this is a useless statement so that it is possible to set a breakpoint here lol
		//error handler
	}
}

void process_chassis_can_msg(uint16_t msg_id, uint8_t rx_buffer[]) {
	//for future use
}

/**
 * Centers the raw motor angle to between -Pi to +Pi
 */
void angle_offset(raw_data_t *motor_data, angle_data_t *angle_data) {
	int32_t temp_ang = 0;

	//if there's a gearbox, use the ticks after the gearbox.
	//make sure center angle is properly set with respect to the zero-ing angle
	//YOUR ROBOT MUST HAVE A WAY TO ZERO THIS ANGLE AND IMPLEMENT A ZEROING FUNCTION AT STARTUP
	//IF NOT IT WON'T WORK 							-wx
	temp_ang = angle_data->ticks - angle_data->center_ang;
	if (temp_ang > angle_data->max_ticks) {
		temp_ang -= angle_data->tick_range;
	} else if (temp_ang < angle_data->min_ticks) {
		temp_ang += angle_data->tick_range;
	}
//	angle_data->ticks = temp_ang;
	angle_data->adj_ang = (float) temp_ang * angle_data->ang_range
			/ angle_data->tick_range;
}

void motor_calc_odometry(raw_data_t *motor_data, angle_data_t *angle_data,
		uint32_t feedback_times[]) {
	int16_t abs_angle_diff;
	if (angle_data->init == 0) {
		angle_data->ticks = motor_data->angle[0];
		if (angle_data->ticks > angle_data->max_ticks) {
			angle_data->ticks -= angle_data->tick_range;
		}
		if (angle_data->ticks < angle_data->min_ticks) {
			angle_data->ticks += angle_data->tick_range;
		}
		motor_data->angle[1] = motor_data->angle[0];
		angle_data->init = 1;
		return;
	}
	abs_angle_diff = motor_data->angle[0] - motor_data->angle[1];
	//generally the motor won't exceed half a turn between each feedback
	if (abs_angle_diff > angle_data->max_raw_ticks) {
		abs_angle_diff -= angle_data->raw_ticks_range;
	} else if (abs_angle_diff < angle_data->min_raw_ticks) {
		abs_angle_diff += angle_data->raw_ticks_range;
	}

	uint16_t gear_ticks = angle_data->raw_ticks_range
			* angle_data->gearbox_ratio;
	angle_data->ticks += abs_angle_diff;
	while (angle_data->ticks > angle_data->max_ticks) {
		angle_data->ticks -= angle_data->tick_range;
	}
	while (angle_data->ticks < angle_data->min_ticks) {
		angle_data->ticks += angle_data->tick_range;
	}

	angle_data->dist = angle_data->ticks * angle_data->wheel_circ / gear_ticks;
	motor_data->angle[1] = motor_data->angle[0];
}


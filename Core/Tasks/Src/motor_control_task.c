/*
 * motor_control_task.c
 *
 *  Created on: Sep 26, 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "can_msg_processor.h"
#include "motor_control.h"
#include "motor_control_task.h"
#include "motor_config.h"
#include "bsp_lk_motor.h"

extern motor_data_t g_can_motors[24];
extern motor_map_t dji_motor_map[25];
extern QueueHandle_t g_buzzing_task_msg;
extern remote_cmd_t g_remote_cmd;

extern uint8_t g_safety_toggle;
volatile uint32_t g_motor_control_time;
extern motor_data_t g_pitch_motor;

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

void motor_control_task(void *argument) {

	uint8_t CAN_send_data[8];

	uint32_t dji_enabled_motors = 0;
	// Build enabled motors bitmask
	for (uint8_t i = 0; i < 24; i++) {
		if (dji_motor_map[i + 1].motor_data != NULL) {
			dji_enabled_motors |= 1 << i;
		}
	}

	TickType_t start_time;
	TickType_t delay = 0;

	while (1) {
		delay = 0;
		start_time = xTaskGetTickCount();

		//if safety is on
		if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {

// check if there is LK motor
#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
    PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG
			lk_motor_kill(&g_pitch_motor);
#endif
// check if there is DM motors
#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
			dm4310_clear_para(&dm_pitch_motor);
			dm4310_ctrl_send(PITCH_MOTOR_CAN_PTR, &dm_pitch_motor);
#endif
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
			dm4310_clear_para(&dm_yaw_motor);
	        dm4310_ctrl_send(YAW_MOTOR_CAN_PTR, &dm_yaw_motor);
#endif

			// clears DJI motors

			// clears output sent to motors, killing them
			CAN_send_data[0] = 0;
			CAN_send_data[1] = 0;
			CAN_send_data[2] = 0;
			CAN_send_data[3] = 0;
			CAN_send_data[4] = 0;
			CAN_send_data[5] = 0;
			CAN_send_data[6] = 0;
			CAN_send_data[7] = 0;

			// FDCAN1
			if (dji_enabled_motors & 0x00000F) {
				can_send_msg(&hfdcan1, 0x200, CAN_send_data);
			}
			if (dji_enabled_motors & 0x0000F0) {
				can_send_msg(&hfdcan1, 0x1FF, CAN_send_data);
			}
			if (dji_enabled_motors & 0x000F00) {
				can_send_msg(&hfdcan1, 0x2FF, CAN_send_data);
			}

			// FDCAN2
			if (dji_enabled_motors & 0x00F000) {
				can_send_msg(&hfdcan2, 0x200, CAN_send_data);
			}
			if (dji_enabled_motors & 0x0F0000) {
				can_send_msg(&hfdcan2, 0x1FF, CAN_send_data);
			}
			if (dji_enabled_motors & 0xF00000) {
				can_send_msg(&hfdcan2, 0x2FF, CAN_send_data);
			}

			vTaskDelayUntil(&start_time, 10);
		} else {
			// do not kill motors; send output to them

			if (dji_enabled_motors & 0x00000F) {
				CAN_send_data[0] = (dji_motor_map[1].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[1].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[2].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[2].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[3].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[3].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[4].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[4].motor_data->output);

			    can_send_msg(&hfdcan1, 0x200, CAN_send_data);

			}

			if (dji_enabled_motors & 0x00F000) {
				CAN_send_data[0] = (dji_motor_map[13].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[13].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[14].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[14].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[15].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[15].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[16].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[16].motor_data->output);

			    can_send_msg(&hfdcan2, 0x200, CAN_send_data);

			}

			if (dji_enabled_motors & 0x0000F0) {
				CAN_send_data[0] = (dji_motor_map[5].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[5].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[6].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[6].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[7].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[7].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[8].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[8].motor_data->output);

			    can_send_msg(&hfdcan1, 0x2FF, CAN_send_data);

			}

			if (dji_enabled_motors & 0x0F0000) {
				CAN_send_data[0] = (dji_motor_map[5 + 12].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[5 + 12].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[6 + 12].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[6 + 12].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[7 + 12].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[7 + 12].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[8 + 12].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[8 + 12].motor_data->output);

			    can_send_msg(&hfdcan2, 0x1FF, CAN_send_data);

			}

			if (dji_enabled_motors & 0x000F00) {
				CAN_send_data[0] = (dji_motor_map[9].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[9].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[10].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[10].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[11].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[11].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[12].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[12].motor_data->output);

			    can_send_msg(&hfdcan1, 0x2FF, CAN_send_data);

			}

			if (dji_enabled_motors & 0xF00000) {
				CAN_send_data[0] = (dji_motor_map[9 + 12].motor_data->output) >> 8;
				CAN_send_data[1] = (dji_motor_map[9 + 12].motor_data->output);
				CAN_send_data[2] = (dji_motor_map[10 + 12].motor_data->output) >> 8;
				CAN_send_data[3] = (dji_motor_map[10 + 12].motor_data->output);
				CAN_send_data[4] = (dji_motor_map[11 + 12].motor_data->output) >> 8;
				CAN_send_data[5] = (dji_motor_map[11 + 12].motor_data->output);
				CAN_send_data[6] = (dji_motor_map[12 + 12].motor_data->output) >> 8;
				CAN_send_data[7] = (dji_motor_map[12 + 12].motor_data->output);

			    can_send_msg(&hfdcan2, 0x2FF, CAN_send_data);

			}

#if PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_SPD || \
	PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_ANG || \
	PITCH_MOTOR_TYPE == TYPE_LK_MG5010E_MULTI_ANG
		lk_read_motor_sang(&g_pitch_motor);
#endif

#if PITCH_MOTOR_TYPE == TYPE_DM4310_MIT
			dm4310_ctrl_send(PITCH_MOTOR_CAN_PTR, &dm_pitch_motor);
#endif
#if YAW_MOTOR_TYPE == TYPE_DM4310_MIT
			dm4310_ctrl_send(YAW_MOTOR_CAN_PTR, &dm_yaw_motor);
#endif
			delay = (delay > 2) ? 1 : delay;
			vTaskDelayUntil(&start_time, 2 - (delay));
		}
	}
}

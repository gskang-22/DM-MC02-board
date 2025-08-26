/*
 * control_input_task.c
 *
 *  Created on: 4 Jul 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "motor_control.h"
#include "control_keyboard.h"
#include "control_remote.h"
#include "control_sbc.h"
#include "INS_task.h"

//extern TaskHandle_t buzzing_task_handle;
//extern TaskHandle_t gimbal_control_task_handle;
//extern TaskHandle_t movement_control_task_handle;
//extern TaskHandle_t control_input_task_handle;

extern motor_data_t g_can_motors[24];
extern orientation_data_t imu_heading;
extern INS_t INS;
extern QueueHandle_t g_buzzing_task_msg;
extern remote_cmd_t g_remote_cmd;
extern ref_robot_dmg_t ref_dmg_data;
extern uint32_t ref_dmg_data_txno;
extern ref_game_robot_data2_t ref_robot_data;

chassis_control_t chassis_ctrl_data;
gun_control_t launcher_ctrl_data;
gimbal_control_t gimbal_ctrl_data;
pid_data_t yaw_pid_data;
speed_shift_t gear_speed;
int g_spinspin_mode = 0;
int supercap_dash = 0;
int aimbot_mode = 0;
extern int supercap_enabled;

uint8_t control_mode = CONTROL_DEFAULT;
uint8_t g_safety_toggle = ARM_SWITCH;
uint8_t launcher_safety_toggle = (ARM_SWITCH | LAUNCHER_SAFETY);

uint32_t reset_debounce_time = 0;
uint32_t reset_start_time = 0;

void control_input_task(void *argument) {
	TickType_t start_time;
	control_reset();
	chassis_yaw_pid_init();
	gimbal_ctrl_data.imu_mode = GIMBAL_MODE;
//	aimbot_pid_init();
	dbus_remote_start();
	gear_speed.curr_gear = GEAR_DEFAULT;
	set_gear();
	g_safety_toggle = 1;
	vTaskDelay(100);
	uint8_t rc_check;

	//check if remote is giving non zero values, reset uart in case packet isn't aligned properly
	while (fabs(g_remote_cmd.left_x) > 50 || fabs(g_remote_cmd.right_x) > 50 || fabs(g_remote_cmd.left_x) > 50 || fabs(g_remote_cmd.right_x) > 50){
		uint8_t temp_msg;
		temp_msg = not_ok;
		xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
		rc_check = ulTaskNotifyTake(pdTRUE, 200);
		HAL_UART_DMAStop(DBUS_UART);
		dbus_remote_start();
		if (rc_check){
			vTaskDelay(200);
		}
	}
	g_safety_toggle = ARM_SWITCH;

	uint32_t last_song = 0;
	while (1) {
		rc_check = ulTaskNotifyTake(pdTRUE, 200);
		if (rc_check) {
			status_led(1, on_led);
			start_time = xTaskGetTickCount();
			if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
				if (g_remote_cmd.keyboard_keys & KEY_OFFSET_SHIFT){
					if (g_remote_cmd.keyboard_keys & KEY_OFFSET_CTRL){
						if (HAL_GetTick() - reset_debounce_time > 100){
							reset_start_time = HAL_GetTick();
						}
						reset_debounce_time = HAL_GetTick();
						if (HAL_GetTick() - reset_start_time > 5000){
							NVIC_SystemReset();
						}
					}
				}


				if ((g_remote_cmd.left_switch == ge_LSW_UNSAFE) && (HAL_GetTick() - last_song > 5000)){
					uint8_t temp_msg;
					last_song = HAL_GetTick();
					temp_msg = song;
//					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
				}
				control_mode_change(g_remote_cmd.side_dial);
				g_safety_toggle = 0;
				launcher_safety_toggle = 0;
				control_reset();
			} else {

				switch (control_mode) {
				case KEYBOARD_CTRL_MODE:
//					keyboard_gear_shifter(&gear_speed);
					set_gear();
					keyboard_control_input();
					break;
				case REMOTE_CTRL_MODE:
//					remote_gear_shifter(&gear_speed);
					set_gear();
					remote_control_input();
					break;
#ifdef HAS_SBC
				case SBC_CTRL_MODE:
					//sbc_control_input();
					nx_control_input();
					break;
					;
#endif

				default:
					break;

				}
				status_led(1, off_led);
			}
		} else {
			//restart remote uart
			if (HAL_GetTick() - g_remote_cmd.last_time > 100) {
				HAL_UART_DMAStop(DBUS_UART);
				dbus_remote_start();
				g_remote_cmd.last_time = HAL_GetTick();
			}
//			kill_can();
			control_reset();
			launcher_safety_toggle = LAUNCHER_SAFETY;
			g_safety_toggle = 1;

		}
		vTaskDelayUntil(&start_time, CONTROL_DELAY);
	}
	osThreadTerminate(NULL);
}

float chassis_center_yaw() {
	chassis_centering_config(); // set chassis centering pid based on level

	speed_pid(0, g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang,
			&yaw_pid_data);
	if (fabs(yaw_pid_data.output) < CHASSIS_YAW_MIN){
		return 0;
	}
	return yaw_pid_data.output;
}

void chassis_centering_config() {
#ifdef LVL_TUNING
	//	static uint8_t prev_robot_level = -1;
	//
	//	// Hopefully with this, we can adjust pid values without it being overwritten all the time
	//	if (prev_robot_level == ref_robot_data.robot_level) return;
	//	prev_robot_level = ref_robot_data.robot_level;

	uint8_t curr_level = ref_robot_data.robot_level;

	if (supercap_dash && supercap_enabled) {
		curr_level += 4;
	}
	switch (curr_level) {
		case 1:
			yaw_pid_data.kp = LV1_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV1_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV1_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV1_CHASSIS_YAW_MAX_RPM;
			break;

		case 2:
			yaw_pid_data.kp = LV2_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV2_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV2_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV2_CHASSIS_YAW_MAX_RPM;
			break;

		case 3:
			yaw_pid_data.kp = LV3_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV3_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV3_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV3_CHASSIS_YAW_MAX_RPM;
			break;

		case 4:
			yaw_pid_data.kp = LV4_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV4_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV4_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV4_CHASSIS_YAW_MAX_RPM;
			break;

		case 5:
			yaw_pid_data.kp = LV5_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV5_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV5_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV5_CHASSIS_YAW_MAX_RPM;
			break;

		case 6:
			yaw_pid_data.kp = LV6_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV6_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV6_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV6_CHASSIS_YAW_MAX_RPM;
			break;

		case 7:
			yaw_pid_data.kp = LV7_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV7_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV7_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV7_CHASSIS_YAW_MAX_RPM;
			break;

		case 8:
			yaw_pid_data.kp = LV8_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV8_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV8_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV8_CHASSIS_YAW_MAX_RPM;
			break;

		case 9:
			yaw_pid_data.kp = LV9_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV9_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV9_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV9_CHASSIS_YAW_MAX_RPM;
			break;

		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
			yaw_pid_data.kp = LV10_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV10_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV10_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		default:
			yaw_pid_data.kp = LV1_CHASSIS_YAW_KP;
			yaw_pid_data.ki = LV1_CHASSIS_YAW_KI;
			yaw_pid_data.kd = LV1_CHASSIS_YAW_KD;
			yaw_pid_data.max_out = LV1_CHASSIS_YAW_MAX_RPM;
	}
#else
	yaw_pid_data.kp = CHASSIS_YAW_KP;
	yaw_pid_data.ki = CHASSIS_YAW_KI;
	yaw_pid_data.kd = CHASSIS_YAW_KD;
#endif
}


void chassis_set_ctrl(float forward, float horizontal, float yaw){
	chassis_ctrl_data.enabled = 1;
	chassis_ctrl_data.horizontal = horizontal;
	chassis_ctrl_data.forward = forward;
	chassis_ctrl_data.yaw = yaw;
}

void chassis_kill_ctrl(){
	chassis_ctrl_data.enabled = 0;
	chassis_ctrl_data.forward = 0;
	chassis_ctrl_data.horizontal = 0;
	chassis_ctrl_data.yaw = 0;
}
uint8_t gimbal_aim_at_damaged_plate(float* yaw_rad) {
	static uint32_t last_dmg_data;
	if (last_dmg_data != ref_dmg_data_txno) {
		last_dmg_data = ref_dmg_data_txno;
		if (ref_dmg_data.dmg_type == 0) {
			switch (ref_dmg_data.armor_type) {
			case 1:
				*yaw_rad = imu_heading.yaw
						- g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						+ (PI / 2);

				return 1;
			case 2:
				*yaw_rad = imu_heading.yaw
						- g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						+ (PI);

				return 1;
			case 3:
				*yaw_rad = imu_heading.yaw
						- g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang
						- (PI / 2);

				return 1;
			case 0:
				*yaw_rad = imu_heading.yaw
						- g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;

				return 1;
			default:
				break;

			}
		}
	}
	return 0;
}

void control_reset() {
	chassis_ctrl_data.forward = 0;
	chassis_ctrl_data.horizontal = 0;
	chassis_ctrl_data.yaw = 0;
	chassis_ctrl_data.enabled = 0;
	gimbal_ctrl_data.pitch = 0;
	gimbal_ctrl_data.yaw = imu_heading.yaw;
	gimbal_ctrl_data.enabled = 0;
	launcher_ctrl_data.firing = 0;
	launcher_ctrl_data.projectile_speed = 0;
	launcher_ctrl_data.enabled = 0;
	g_spinspin_mode = 0;
	aimbot_mode = 0;
}

void control_mode_change(int16_t left_dial_input) {
//assume already in shutdown mode here
	static uint32_t last_trig_time;
	uint8_t temp_msg;
	if (g_remote_cmd.left_switch == ge_LSW_CONFIG) {
		if (left_dial_input > 330 || left_dial_input < -330) {
			if (HAL_GetTick() - last_trig_time > 1000) {
				switch (control_mode) {
				case KEYBOARD_CTRL_MODE:
				case REMOTE_CTRL_MODE:
					control_mode = SBC_CTRL_MODE;
					temp_msg = control_sbc;
					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
					break;
				default:
					last_trig_time = HAL_GetTick();
					break;
				}
			}
		} else {
			last_trig_time = HAL_GetTick();
		}

	} else {
		switch (control_mode) {
		case KEYBOARD_CTRL_MODE:
			if (left_dial_input < -330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = REMOTE_CTRL_MODE;
					temp_msg = control_control;
					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
					launcher_safety_toggle = LAUNCHER_SAFETY;
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		case REMOTE_CTRL_MODE:
			if (left_dial_input > 330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = KEYBOARD_CTRL_MODE;
					temp_msg = control_keyboard;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		case SBC_CTRL_MODE:
			if (left_dial_input < -330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = REMOTE_CTRL_MODE;
					temp_msg = control_control;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
				}
			} else if (left_dial_input > 330) {
				if (HAL_GetTick() - last_trig_time > 1000) {
					control_mode = KEYBOARD_CTRL_MODE;
					temp_msg = control_keyboard;
					launcher_safety_toggle = LAUNCHER_SAFETY;
					xQueueSendToBack(g_buzzing_task_msg, &temp_msg, 0);
				}
			} else {
				last_trig_time = HAL_GetTick();
			}
			break;
		default:
			break;
		}
	}
}

//ADDs angle to gimbal ctrl
void gimbal_turn_ang(float pit_radians, float yaw_radians) {
//	yaw_radians = gimbal_ctrl_data.yaw + yaw_radians;
	while (yaw_radians > PI) {
		yaw_radians -= 2 * PI;
	}
	while (yaw_radians < -PI) {
		yaw_radians += 2 * PI;
	}
	xSemaphoreTake(gimbal_ctrl_data.yaw_semaphore,portMAX_DELAY);
	gimbal_ctrl_data.delta_yaw += yaw_radians;
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);
	xSemaphoreTake(gimbal_ctrl_data.pitch_semaphore,portMAX_DELAY);
	gimbal_ctrl_data.pitch += pit_radians;
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);
//	gimbal_ctrl_data.yaw = yaw_radians;
}
//SETs angle to gimbal ctrl
void gimbal_set_ang(float pit_radians, float yaw_radians) {
	while (yaw_radians > PI) {
		yaw_radians -= 2 * PI;
	}
	while (yaw_radians < -PI) {
		yaw_radians += 2 * PI;
	}
	gimbal_ctrl_data.pitch = pit_radians;
	gimbal_ctrl_data.yaw = yaw_radians;
}

void set_gear() {
	switch (gear_speed.curr_gear) {
	case 1:
		gear_speed.spin_mult = GEAR1_YAW_MULT;
		gear_speed.trans_mult = GEAR1_SPEED_MULT;
		gear_speed.accel_mult = GEAR1_ACCEL_MULT;
		break;
	case 2:
		gear_speed.spin_mult = GEAR2_YAW_MULT;
		gear_speed.trans_mult = GEAR2_SPEED_MULT;
		gear_speed.accel_mult = GEAR2_ACCEL_MULT;
		break;
	case 3:
		gear_speed.spin_mult = GEAR3_YAW_MULT;
		gear_speed.trans_mult = GEAR3_SPEED_MULT;
		gear_speed.accel_mult = GEAR3_ACCEL_MULT;
		break;
	case 4:
		gear_speed.spin_mult = GEAR4_YAW_MULT;
		gear_speed.trans_mult = GEAR4_SPEED_MULT;
		gear_speed.accel_mult = GEAR4_ACCEL_MULT;
		break;
	case 5:
		gear_speed.spin_mult = GEAR5_YAW_MULT;
		gear_speed.trans_mult = GEAR5_SPEED_MULT;
		gear_speed.accel_mult = GEAR5_ACCEL_MULT;
		break;
	case 6:
		gear_speed.spin_mult = GEAR6_YAW_MULT;
		gear_speed.trans_mult = GEAR6_SPEED_MULT;
		gear_speed.accel_mult = GEAR6_ACCEL_MULT;
		break;
	default:
		gear_speed.spin_mult = GEAR3_YAW_MULT;
		gear_speed.trans_mult = GEAR3_SPEED_MULT;
		gear_speed.accel_mult = GEAR3_ACCEL_MULT;
		break;
	}

}

void chassis_yaw_pid_init() {
#ifdef LVL_TUNING
	yaw_pid_data.kp = LV1_CHASSIS_YAW_KP;
	yaw_pid_data.ki = LV1_CHASSIS_YAW_KI;
	yaw_pid_data.kd = LV1_CHASSIS_YAW_KD;
	yaw_pid_data.max_out = LV1_CHASSIS_YAW_MAX_RPM;
#else
	yaw_pid_data.kp = CHASSIS_YAW_KP;
	yaw_pid_data.ki = CHASSIS_YAW_KI;
	yaw_pid_data.kd = CHASSIS_YAW_KD;
	yaw_pid_data.max_out = CHASSIS_YAW_MAX_RPM;
#endif
}



void dbus_reset() {
	g_remote_cmd.right_switch = ge_RSW_SHUTDOWN;
	g_remote_cmd.right_x = 0;
	g_remote_cmd.right_y = 0;
	g_remote_cmd.left_x = 0;
	g_remote_cmd.left_y = 0;
	g_remote_cmd.left_switch = 0;
	g_remote_cmd.mouse_x = 0;
	g_remote_cmd.mouse_y = 0;
	g_remote_cmd.mouse_z = 0;
	g_remote_cmd.mouse_left = 0;
	g_remote_cmd.mouse_right = 0;
	if (control_mode == 0) {
		gimbal_ctrl_data.pitch = 0;
		gimbal_ctrl_data.yaw = 0;
	}
	if (control_mode == 1) {
		gimbal_ctrl_data.pitch = INS.Pitch;
		gimbal_ctrl_data.yaw = imu_heading.yaw;
		gimbal_ctrl_data.delta_yaw = 0;
	}
}


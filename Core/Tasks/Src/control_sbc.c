/*
 * control_sbc.c
 *
 *  Created on: 6 Jul 2023
 *      Author: wx
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "control_input_task.h"
#include "control_sbc.h"
#include "motor_control.h"

extern remote_cmd_t g_remote_cmd;
extern QueueHandle_t g_buzzing_task_msg;
extern sbc_data_t sbc_data;
extern uint8_t sbc_new_data;
pid_data_t aimbot_y_pid;
pid_data_t aimbot_x_pid;


extern ref_game_state_t ref_game_state;
extern orientation_data_t imu_heading;
extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t g_safety_toggle;
extern uint8_t launcher_safety_toggle;
uint32_t sbc_last_time;

//static uint32_t last_damage_time;
static uint32_t last_damage_time;
#define LOW_SPIN 0.1

typedef struct {
	float x_offset;
	float y_offset;
} aimbot_offset_t;
aimbot_offset_t aimbot_offset;
extern motor_data_t g_can_motors[24];

void nx_control_input() {
	// Use remote input to control the chassis
	nx_remote_chassis_input();

	// Use remote input to enable / disable the gimbal
	// Firing command by the aimbot
	nx_gimbal_input();

	// Use remote input to enable / disable the launcher
	// Firing command by the aimbot
	nx_launcher_input();
}

void nx_remote_chassis_input() {
	if (g_safety_toggle || g_remote_cmd.right_switch != ge_RSW_ALL_ON) {
		chassis_kill_ctrl();
	} else {
		chassis_ctrl_data.enabled = 1;
		float horizontal_input = 0.0;
		float forward_input = 0.0;
		float yaw_input = 0.0;

		forward_input = (float) g_remote_cmd.left_y / RC_LIMITS;
		horizontal_input = (float) g_remote_cmd.left_x / RC_LIMITS;
		if (g_remote_cmd.left_switch == ge_LSW_STANDBY){
			if (abs(g_remote_cmd.side_dial) > 50 ){
				yaw_input = (float)g_remote_cmd.side_dial * CHASSIS_SPINSPIN_MAX/660;
			} else {
				yaw_input = chassis_center_yaw();
			}
		}else {
			yaw_input = chassis_center_yaw();
		}

		chassis_set_ctrl(forward_input, horizontal_input, yaw_input);
	}
}

void nx_gimbal_input() {
	if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;
	}
}

void nx_launcher_input() {
	if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN
			|| g_remote_cmd.left_switch != ge_LSW_UNSAFE) {
		if (g_remote_cmd.left_switch != ge_LSW_UNSAFE) {
			launcher_safety_toggle = 0;
		}
		if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN){
			launcher_ctrl_data.enabled = 0;
		}
		launcher_ctrl_data.firing = 0;
		launcher_ctrl_data.projectile_speed = 0;
	} else {
		launcher_ctrl_data.enabled = 1;
		launcher_ctrl_data.projectile_speed = 1;
	}
}

void sbc_control_input() {
	static uint8_t sbc_timeout;
	//check for new damage
	sbc_timeout = 0;
	//add in timeout?
	if (HAL_GetTick() - sbc_last_time > 1000) {
		sbc_timeout = 1;
	}
	if (g_remote_cmd.left_switch != ge_LSW_STANDBY) {
		//add in armour plate override if no spinspin
		sbc_gimbal_control_input(sbc_timeout);
		if (ref_game_state.game_progress == 4){
			sbc_chassis_control_input(sbc_timeout);
			sbc_launcher_control_input(sbc_timeout);
		} else {
			chassis_ctrl_data.forward = 0;
			chassis_ctrl_data.yaw = 0;
			chassis_ctrl_data.horizontal = 0;
			launcher_ctrl_data.firing = 0;
			launcher_ctrl_data.projectile_speed = 0;
		}
		sbc_new_data = 0;
	} else {
		sbc_new_data = 0;
		control_reset();
	}
}

void aimbot_pid_init() {
	aimbot_offset.y_offset = 0;
	aimbot_offset.x_offset = 0;
	aimbot_x_pid.kp = AIMBOT_X_KP;
	aimbot_x_pid.ki = AIMBOT_X_KI;
	aimbot_x_pid.kd = AIMBOT_X_KD;
	aimbot_x_pid.max_out = 1.5;
	aimbot_x_pid.int_max = AIMBOT_KI_MAX;

	aimbot_y_pid.kp = AIMBOT_Y_KP;
	aimbot_y_pid.ki = AIMBOT_Y_KI;
	aimbot_y_pid.kd = AIMBOT_Y_KD;
	aimbot_y_pid.max_out = 1.5;
	aimbot_y_pid.int_max = AIMBOT_KI_MAX;
}

void sbc_gimbal_control_input(uint8_t timeout) {
	if (g_safety_toggle || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
		gimbal_ctrl_data.enabled = 0;
	} else {
		gimbal_ctrl_data.enabled = 1;
		if (sbc_new_data == 1) {
			switch (sbc_data.cmd_id) {
			case SBC_YOLO_BB_ID:
				sbc_gimbal_process_yolo();
				break;
			case SBC_AIMBOT_NORM_ID:
				sbc_gimbal_process_norm();
				break;
			case SBC_GIMBAL_SET_ANG_ID:
				sbc_gimbal_process_set_ang();
				break;
			case SBC_GIMBAL_TURN_ANG_ID:
				sbc_gimbal_process_turn_ang();
				break;
			default:
				return;
			}
		} else {
			if (timeout == 1) {
				float extra = 0;
				if (gimbal_aim_at_damaged_plate(&extra) == 1) {
					last_damage_time = HAL_GetTick();
					gimbal_set_ang(0, extra);
				}
//				gimbal_ctrl_data.pitch = 0;
				//check if new damage
				aimbot_x_pid.error[0] = 0;
				aimbot_y_pid.error[0] = 0;
				aimbot_offset.y_offset = AIMBOT_Y_OFFSET;
				aimbot_offset.x_offset = AIMBOT_X_OFFSET;
			}
		}
	}
}
void sbc_gimbal_process_yolo() {
	sbc_yolo_data_t *sbc_yolo_data = &sbc_data.data.yolo_data;
	int16_t y_range = (sbc_yolo_data->y_max - sbc_yolo_data->y_min) / 2;
	int16_t x_range = (sbc_yolo_data->x_max - sbc_yolo_data->x_min) / 2;
	;
	float sbc_x = (sbc_yolo_data->x_coord - x_range) / x_range;
	float sbc_y = (sbc_yolo_data->y_coord - y_range) / y_range;
	float pit_rad;
	float yaw_rad;
	if (sbc_y != aimbot_offset.y_offset) {
		speed_pid(aimbot_offset.y_offset, sbc_y, &aimbot_y_pid);
	}
	if (sbc_x != aimbot_offset.x_offset) {
		speed_pid(aimbot_offset.x_offset, sbc_x, &aimbot_x_pid);
	}
	pit_rad = imu_heading.pit
			+ (aimbot_y_pid.output * FOV_MULT * AIMBOT_PIT_MULT / y_range);
	yaw_rad = imu_heading.yaw
			+ (aimbot_x_pid.output * FOV_MULT * AIMBOT_YAW_MULT / x_range);
	gimbal_set_ang(pit_rad, yaw_rad);

}
void sbc_gimbal_process_norm() {
	//FLIP BACK ON SBC SOME DAY PLS
	sbc_gimbal_data_t *sbc_gim_data = &sbc_data.data.gimbal_data;
	float sbc_x = sbc_gim_data->pitch - 0.5;
	float sbc_y = sbc_gim_data->yaw - 0.5;
	float pit_rad;
	float yaw_rad;
	if (sbc_gim_data->spinspin == 0) {
		pit_rad = PITCH_SURVEILLANCE;
		if (gimbal_aim_at_damaged_plate(&yaw_rad) == 1) {
			last_damage_time = HAL_GetTick();
		} else if (HAL_GetTick() - last_damage_time > DAMAGE_TIMEOUT) {
			yaw_rad = imu_heading.yaw + sbc_x;
		} else {
//			yaw_rad = imu_heading.yaw;
			yaw_rad = gimbal_ctrl_data.yaw;
		}

		//since nanopi continuously sends the same value, make sure integral = 0
		aimbot_y_pid.integral = 0;
		aimbot_x_pid.integral = 0;

	} else {
		if (sbc_y != aimbot_offset.y_offset) {
			speed_pid(aimbot_offset.y_offset, sbc_y, &aimbot_y_pid);
			pit_rad = imu_heading.pit + (aimbot_y_pid.output * AIMBOT_PIT_MULT);
		} else {
			pit_rad = 0;
		}
		if (sbc_x != aimbot_offset.x_offset) {
			speed_pid(aimbot_offset.x_offset, sbc_x, &aimbot_x_pid);
		}
		yaw_rad = imu_heading.yaw + (aimbot_x_pid.output * AIMBOT_YAW_MULT);
	}
	gimbal_set_ang(pit_rad, yaw_rad);

}

void sbc_gimbal_process_set_ang() {
	float pit_rad;
	float yaw_rad = g_can_motors[YAW_MOTOR_ID-1].angle_data.adj_ang;
	pit_rad = sbc_data.data.gimbal_data.pitch;
	if (sbc_data.data.gimbal_data.yaw < 3.14
			&& sbc_data.data.gimbal_data.yaw > -3.14) {
		yaw_rad = sbc_data.data.gimbal_data.yaw;
	}
	gimbal_set_ang(pit_rad, yaw_rad);

}

void sbc_gimbal_process_turn_ang() {
	float pit_rad;
	float yaw_rad;

	if (sbc_data.data.gimbal_data.pitch < 3.14
			&& sbc_data.data.gimbal_data.pitch > -3.14) {
		pit_rad = gimbal_ctrl_data.pitch + sbc_data.data.gimbal_data.pitch;
	} else {
		pit_rad = 0;
	}
	yaw_rad = gimbal_ctrl_data.yaw + sbc_data.data.gimbal_data.yaw;
	gimbal_set_ang(pit_rad, yaw_rad);

}

void sbc_chassis_control_input(uint8_t sbc_timeout) {

	if (g_safety_toggle || g_remote_cmd.right_switch != ge_RSW_ALL_ON) {
		chassis_kill_ctrl();
	} else {
		float forward = 0;
		float horizontal = 0;
		float yaw = 0;
		uint8_t spinspin = 0;

#ifdef CHASSIS_CAN_SPINSPIN
		if (sbc_new_data == 1) {
			switch (sbc_data.cmd_id) {
			case SBC_AIMBOT_NORM_ID:
			case SBC_GIMBAL_TURN_ANG_ID:
			case SBC_GIMBAL_SET_ANG_ID:
				if (HAL_GetTick() - last_damage_time < DAMAGE_TIMEOUT) {
					spinspin = 1;
				} else {
					spinspin = sbc_data.data.gimbal_data.spinspin;
					if (spinspin == 1) {
						last_damage_time = HAL_GetTick();
					} else {
						spinspin = 2;
					}
				}
				break;
			default:
				spinspin = 2;
				return;
			}
		} else if (HAL_GetTick() - last_damage_time < DAMAGE_TIMEOUT) {
			spinspin = 1;
		} else {
			spinspin = 2;
		}

#endif
		//possible random spin rpm mode?
//		static uint32_t spin_spin_rand;

		if (spinspin == 1) {
			yaw = CHASSIS_SPINSPIN_MAX;
		} else if (spinspin == 2){
			yaw = CHASSIS_SPINSPIN_MAX * LOW_SPIN;
//			if (HAL_GetTick() - spin_spin_rand > SPINSPIN_RANDOM_DELAY){
////				yaw = (((float)(get_microseconds()%129) * 0.9) /129.0) + 0.1;
//				yaw = yaw * CHASSIS_SPINSPIN_MAX;
////				prev_yaw = yaw;
////				spin_spin_rand = HAL_GetTick();
//			} else{
//				yaw=prev_yaw;
//			}
		}
		else{
			yaw = chassis_center_yaw();
		}
		chassis_set_ctrl(forward, horizontal, yaw);
	}
}
#define BURST_TIME 400
#define REST_TIME 800
void sbc_launcher_control_input(uint8_t timeout) {
	static uint32_t burst_fire_time;
	static uint8_t bursting;
	if (g_safety_toggle || launcher_safety_toggle
			|| g_remote_cmd.right_switch == ge_RSW_SHUTDOWN
			|| g_remote_cmd.left_switch != ge_LSW_UNSAFE) {
		bursting = 1;
		if (g_remote_cmd.left_switch != ge_LSW_UNSAFE) {
			launcher_safety_toggle = 0;
		}
		if (g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
			launcher_ctrl_data.enabled = 0;
		}
//		launcher_ctrl_data.enabled = 0;
		launcher_ctrl_data.firing = 0;
		launcher_ctrl_data.projectile_speed = 0;
	} else {
		launcher_ctrl_data.enabled = 1;
		launcher_ctrl_data.projectile_speed = 1;
		if (timeout == 1) {
			bursting = 1;
			launcher_ctrl_data.firing = 0;
		} else if (sbc_new_data) {
			switch (sbc_data.cmd_id) {
			case SBC_AIMBOT_NORM_ID:
			case SBC_GIMBAL_TURN_ANG_ID:
			case SBC_GIMBAL_SET_ANG_ID:
				if (sbc_data.data.gimbal_data.fire == 1) {
					switch (bursting) {
					case 1:
						launcher_ctrl_data.firing = 1;
						bursting = 2;
						burst_fire_time = HAL_GetTick();
						break;
					case 2:
						if (HAL_GetTick() - burst_fire_time > BURST_TIME) {
							bursting = 0;
							burst_fire_time = HAL_GetTick();
							launcher_ctrl_data.firing = 0;
						} else {
							launcher_ctrl_data.firing = 1;
						}
						break;

					case 0:
						if (HAL_GetTick() - burst_fire_time > REST_TIME) {
							bursting = 1;
							launcher_ctrl_data.firing = 1;
							burst_fire_time = HAL_GetTick();
						} else {
							launcher_ctrl_data.firing = 0;
						}
						break;
					default:
						launcher_ctrl_data.firing = 0;
						break;
					}
				} else {
					if (bursting == 0) {
						if (HAL_GetTick() - burst_fire_time > REST_TIME) {
							bursting = 1;
							burst_fire_time = HAL_GetTick();
						}
					} else if (bursting == 2) {
						if (HAL_GetTick() - burst_fire_time > BURST_TIME) {
							bursting = 0;
							burst_fire_time = HAL_GetTick();
						}
					}

					launcher_ctrl_data.firing = 0;

				}
				break;
			default:
				return;
			}
		}
	}
}

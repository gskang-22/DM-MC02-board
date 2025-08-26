/*
 * cvAimbotCommandThread.c
 *
 *  Created on: Mar 20, 2025
 *      Author: cw
 */

#include <cvAimbotCommandThread.h>
#include <Telemetry.h>
#include "robot_config.h"
#include "INS_task.h"
#include "bsp_damiao.h"
extern motor_data_t g_can_motors[24];
extern motor_data_t g_pitch_motor;

extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;

extern INS_t INS;
extern orientation_data_t imu_heading;

extern gimbal_control_t gimbal_ctrl_data;
extern uint8_t control_mode;

extern gun_control_t launcher_ctrl_data;
extern gyro_data_t gyro_proc_data;

cvAimbotCommandThread* cvAimbotCommandInstance = nullptr;

double gimbal_data_yaw;
double gimbal_data_pitch;

extern remote_cmd_t g_remote_cmd;
extern uint8_t g_rc_check;

uint8_t g_is_aiming = false;

cvAimbotCommandThread::~cvAimbotCommandThread(){
}

void cvAimbotCommandThread::init() {
	cvAimbotCommandInstance = this;
	yaw = 0.0f;
	pitch = 0.0f;
	aim_state = false;
	fire_state = false;
}

void cvAimbotCommandThread::loop() {
	g_is_aiming = aim_state;

	if (control_mode == SBC_CTRL_MODE) {
		gimbal_data_yaw = yaw * 2;
		gimbal_data_pitch = pitch;

		if (!g_rc_check || g_remote_cmd.right_switch == ge_RSW_SHUTDOWN) {
			gimbal_ctrl_data.enabled = 0;
		} else {
			gimbal_ctrl_data.enabled = 1;
		}

		if (aim_state) {
			gimbal_ctrl_data.delta_yaw = yaw * 2; // + g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
			gimbal_ctrl_data.pitch = pitch + INS.Pitch;
			launcher_ctrl_data.firing = fire_state;
		}
	}

	osDelay(2);
	portYIELD();
}

void cvAimbotCommandThread::handle_cv_gimbal(uint8_t sender_id, cvGimbalCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_gimbal(packet);
}

void cvAimbotCommandThread::send_command_gimbal(cvGimbalCommandPacket* packet) {
	yaw = YAW_INVERT * packet->yaw;
	pitch = PITCH_INVERT * packet->pitch;
}

void cvAimbotCommandThread::handle_cv_firing(uint8_t sender_id, firingCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_firing(packet);
}

void cvAimbotCommandThread::send_command_firing(firingCommandPacket* packet) {
	fire_state = packet->fire_state;
}

void cvAimbotCommandThread::handle_cv_aim(uint8_t sender_id, aimCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		return;
	}
	if (cvAimbotCommandInstance == nullptr) {
		scanf("cv aimbot command thread instance does not exist yet\r\n");
		return;
	}

	cvAimbotCommandInstance->send_command_aim(packet);
}
//
void cvAimbotCommandThread::send_command_aim(aimCommandPacket* packet) {
	aim_state = packet->aim_state;
}

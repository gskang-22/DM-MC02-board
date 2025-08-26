/*
 * spd_cmd_thread.cpp
 *
 *  Created on: May 16, 2025
 *      Author: Yassine, JL
 */

#include <spd_cmd_thread.h>
#include <Telemetry.h>
#include "typedefs.h"
#include "robot_config.h"
#include <cmath>
#include <algorithm>

extern remote_cmd_t g_remote_cmd;
extern gimbal_control_t	gimbal_ctrl_data;
extern chassis_control_t chassis_ctrl_data;
extern gun_control_t launcher_ctrl_data;
extern uint8_t control_mode;

extern int g_spinspin_mode;
//extern uint8_t set_launcher;
static float test_vals[3];

double global_dt = 0.0;
extern uint8_t g_rc_check;

uint8_t g_is_navigating = false;

typedef struct {
    uint32_t curr_receive_time;
    uint32_t last_receive_time;
    bool is_navigating;
    bool beyblade_mode;
    float V_horz;  // Horizontal speed. In the direction of the X axis.
    float V_lat;   // Lateral speed. In the direction of the Y axis.
    float V_yaw;   // Chassis yaw axis. Not used in beyblade mode.
} nav_state_t;

nav_state_t g_nav_state;

ChassisSpdCmdThread* chassisSpeedInstance = nullptr;

ChassisSpdCmdThread::~ChassisSpdCmdThread(){
}

void ChassisSpdCmdThread::init(){
	chassisSpeedInstance = this;

	curr_receive_time = 0;
	last_receive_time = 0;

	V_horz = 0;
	V_lat = 0;
	V_yaw = 0;

	beyblade_mode = false;

	gimbal_yaw = 0;
	gimbal_ctrl_data.enabled = 1;
	chassis_ctrl_data.enabled = 1;
}

void ChassisSpdCmdThread::loop() {
    g_nav_state.curr_receive_time = curr_receive_time;
    g_nav_state.last_receive_time = last_receive_time;
    g_nav_state.is_navigating = is_navigating;
    g_nav_state.beyblade_mode = beyblade_mode;
    g_nav_state.V_horz = V_horz;
    g_nav_state.V_lat = V_lat;
    g_nav_state.V_yaw = V_yaw;

    g_is_navigating = is_navigating;

//	if (g_remote_cmd.left_switch == ge_RSW_SHUTDOWN && manual_mode){
	if (control_mode == SBC_CTRL_MODE) {
		// kill control if right switch is not at the bottom
		if (g_remote_cmd.right_switch != ge_RSW_ALL_ON || !g_rc_check) { // Safety kill
			V_horz = 0;
			V_lat = 0;
			V_yaw = 0;
			gimbal_yaw = 0;
			beyblade_mode = false;
			control_reset();
		}

		else {
			if (is_navigating) {
//				double dt = (curr_receive_time - last_receive_time) / 1000.0;
//				gimbal_ctrl_data.delta_yaw = gimbal_yaw * dt;
				gimbal_ctrl_data.yaw = gimbal_yaw * 2;


				if (!beyblade_mode)
					V_yaw = chassis_center_yaw();
				else
					V_yaw = beyblade_mode;

				chassis_set_ctrl(V_horz, V_lat, V_yaw);
			}
		}
	}

	osDelay(25);
	portYIELD();

}


void ChassisSpdCmdThread::handle_chassis_spd_commands(uint8_t sender_id, chassisSpeedCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		scanf("Unreliable chassis command packet");
		return;
	}
	if (chassisSpeedInstance == nullptr) {
		scanf("Speed chassis command thread instance does not exist yet\r\n");
		return;
	}
	else{
//		scanf("Received chassis speed command");
		chassisSpeedInstance->send_chassis_spd_commands(packet);
	}
}

void ChassisSpdCmdThread::handle_chassis_spin_commmands(uint8_t sender_id, chassisSpinCommandPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		scanf("Unreliable spin command packet");
		return;
	}
	if (chassisSpeedInstance == nullptr) {
		scanf("Speed chassis command thread instance does not exist yet\r\n");
		return;
	}
	else{
		chassisSpeedInstance->send_navigation_commands(packet->spinning_state);
	}
}

void ChassisSpdCmdThread::handle_navigation_commands(uint8_t sender_id, isNavigatingPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
		scanf("Unreliable spin command packet");
		return;
	}
	if (chassisSpeedInstance == nullptr) {
		scanf("Speed chassis command thread instance does not exist yet\r\n");
		return;
	}
	else{
		chassisSpeedInstance->set_spinspin(packet->navigating_state);
	}
}

void ChassisSpdCmdThread::send_chassis_spd_commands(chassisSpeedCommandPacket* packet){
	this->V_horz = packet->V_horz;
	this->V_lat = packet->V_lat;
	this->gimbal_yaw = packet->V_yaw;

	last_receive_time = curr_receive_time;
	curr_receive_time = HAL_GetTick();

	portYIELD();
}

void ChassisSpdCmdThread::set_spinspin(bool beyblade_mode) {
	this->beyblade_mode = beyblade_mode;
}

void ChassisSpdCmdThread::send_navigation_commands(bool is_navigating) {
	this->is_navigating = is_navigating;
}


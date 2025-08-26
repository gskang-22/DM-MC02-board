/*
 * status_thread.cpp
 *
 *  Created on: Mar 19, 2025
 *      Author: cw
 */

#include <status_thread.h>
#include "referee_msgs.h"

#include "robot_config.h"

#include "cvAimbotCommandThread.h"

extern ref_game_state_t ref_game_state;
extern uint32_t ref_game_state_txno;
extern ref_game_robot_data2_t ref_robot_data;
extern uint32_t ref_robot_data_txno;
extern ref_game_robot_HP_t ref_robot_hp;
extern uint32_t ref_robot_hp_txno;
extern ref_game_result_t ref_game_result_data;
extern uint32_t ref_game_result_txno;
extern ref_rfid_status_t ref_rfid_status_data;
extern uint32_t ref_rfid_status_txno;


extern cvAimbotCommandThread* cvAimbotCommandInstance;

statusThread* statusInstance = nullptr;

// Declare your data with the proper data structure defined in DataStructures.h
static competitionStatusData competition_data;
static competitionStatusPacket competition_packet;

static occupationStatusData occupation_data;
static occupationStatusPacket occupation_packet;

static winStatusData win_data;
static winStatusPacket win_packet;

static LeftTriggerPositionData left_trigger_data;
static leftTriggerPositionPacket left_trigger_packet;

extern remote_cmd_t g_remote_cmd;
extern uint8_t control_mode;

statusThread::~statusThread(){}

void statusThread::init(){}

void statusThread::loop()
{
	if (last_ref_game_state_txno != ref_game_state_txno) {
		competition_data.game_progress = ref_game_state.game_progress;
		competition_data.time_left = ref_game_state.stage_remain_time;
		last_ref_game_state_txno = ref_game_state_txno;
	}

	if (last_ref_robot_data_txno != ref_robot_data_txno) {
		competition_data.robot_id = ref_robot_data.robot_id;
		competition_data.current_hp = ref_robot_data.current_HP;
		last_ref_robot_data_txno = ref_robot_data_txno;

		// Determining team colour (red if id < 100, else blue )
		team_colour = (competition_data.robot_id < 100) ? RED_TEAM : BLUE_TEAM;
	}

	if (last_ref_robot_hp_txno != ref_robot_hp_txno) {
		competition_data.red_hero_hp = ref_robot_hp.red_1_HP;
		competition_data.red_standard_hp = ref_robot_hp.red_3_HP;
		competition_data.red_sentry_hp = ref_robot_hp.red_7_HP;

		competition_data.blue_hero_hp = ref_robot_hp.blu_1_HP;
		competition_data.blue_standard_hp = ref_robot_hp.blu_3_HP;
		competition_data.blue_sentry_hp = ref_robot_hp.blu_7_HP;

	    last_ref_robot_hp_txno = ref_robot_hp_txno;
	}

	if (last_ref_rfid_status_txno != ref_rfid_status_txno) {
		occupation_data.resupply_occupation = ref_rfid_status_data.rfid_buff & REF_RFID_RMUL_CENTRAL;
		occupation_data.central_occupation = ref_rfid_status_data.rfid_buff & REF_RFID_RMUL_RESUPPLY;

		last_ref_rfid_status_txno = ref_rfid_status_txno;
	}

	if (last_game_result_txno != ref_game_result_txno) {
		win_packet.win_state = false;
		// during competition result calculation period
		if (competition_data.game_progress == 5) {
			if ((team_colour == RED_TEAM && ref_game_result_data.winner == RED_WIN) ||
				(team_colour == BLUE_TEAM && ref_game_result_data.winner == BLUE_WIN)) {
				win_packet.win_state = true;
			}
		}

		last_game_result_txno++;
	}

	left_trigger_data.trigger_position = g_remote_cmd.left_switch;

	competition_data.toArray((uint8_t*) &competition_packet);
	occupation_data.toArray((uint8_t*) &occupation_packet);
	win_data.toArray((uint8_t*) &win_packet);
	left_trigger_data.toArray((uint8_t*) &left_trigger_packet);

	MAKE_RELIABLE(competition_packet);
	UART_network->send(&competition_packet);

	MAKE_RELIABLE(occupation_packet);
	UART_network->send(&occupation_packet);

	MAKE_RELIABLE(win_packet);
	UART_network->send(&win_packet);

	MAKE_RELIABLE(left_trigger_packet);
	UART_network->send(&left_trigger_packet);

	osDelay(500);

	portYIELD();
}

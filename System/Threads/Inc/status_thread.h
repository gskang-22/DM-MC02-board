/*
 * status_thread.h
 *
 *  Created on: Mar 19, 2025
 *      Author: cw
 */

#ifndef THREADS_INC_STATUS_THREAD_H_
#define THREADS_INC_STATUS_THREAD_H_

#include <stm32h7xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

typedef enum
{
	RED_TEAM		= 1,
	BLUE_TEAM		= 2,
} team_colour;

class statusThread: public Thread {
public:

	statusThread(): Thread("Status") {};
	~statusThread();

	void init();
	void loop();

private:

	uint32_t last_ref_game_state_txno = 0;
	uint32_t last_ref_robot_data_txno = 0;
	uint32_t last_ref_robot_hp_txno = 0;
	uint32_t last_ref_event_txno = 0;
	uint32_t last_game_result_txno = 0;
	uint32_t last_ref_rfid_status_txno = 0;

	uint8_t team_colour = RED_TEAM; // 1 for red, 2 for blue
};

extern statusThread* statusInstance;

#endif /* THREADS_INC_STATUS_THREAD_H_ */

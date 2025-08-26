/*
 * cvAimbotCommandThread.h
 *
 *  Created on: Mar 20, 2025
 *      Author: cw
 */

#ifndef THREADS_INC_CVAIMBOTCOMMANDTHREAD_H_
#define THREADS_INC_CVAIMBOTCOMMANDTHREAD_H_

#include <stm32h7xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

class cvAimbotCommandThread : public Thread {
public:

	cvAimbotCommandThread(): Thread("cvAimbotCommand") {};
	~cvAimbotCommandThread();

	void init();
	void loop();

	static void handle_cv_gimbal(uint8_t sender_id, cvGimbalCommandPacket* packet);
	static void handle_cv_firing(uint8_t sender_id, firingCommandPacket* packet);
	static void handle_cv_aim(uint8_t sender_id, aimCommandPacket* packet);

	void send_command_gimbal(cvGimbalCommandPacket* packet);
	void send_command_firing(firingCommandPacket* packet);
	void send_command_aim(aimCommandPacket* packet);

	bool aim_state;

private:

	float pitch;
	float yaw;
	bool fire_state;

};

extern cvAimbotCommandThread* cvAimbotCommandInstance;

#endif /* THREADS_INC_CVAIMBOTCOMMANDTHREAD_H_ */

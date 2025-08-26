/*
 * dummy_thread.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

#ifndef THREADS_INC_SUPERCAP_COMM_THREAD_H_
#define THREADS_INC_SUPERCAP_COMM_THREAD_H_

#include <stm32h7xx_hal.h>
#include <main.h>
#include <Thread.h>
#include "DataStructures.h"

#include "Telemetry.h"

struct ref_msg_packet {
	uint8_t enable_module;
	uint8_t reset;
	uint8_t pow_limit;
	uint16_t energy_buffer;
} __attribute__((packed));


struct supercap_msg_packet {
	float chassis_power;
	uint8_t error;
	uint8_t cap_energy;
} __attribute__((packed));

class SuperCapCommThread : public Thread {
public:

	SuperCapCommThread(): Thread("SuperCapComm"), V_cap(0), P_chassis(0), charge_state(0) {};
	~SuperCapCommThread();

	void init();
	void loop();

	static void handle_supercap(uint8_t sender_id, SuperCapDataPacket* packet);

private:

	float V_cap;
	float P_chassis;
	uint8_t charge_state;

	void txHeaderConfig();

	CAN_TxHeaderTypeDef TxHeader;
	FDCAN_RxHeaderTypeDef RxHeader;

	ref_msg_packet txMsg;
	supercap_msg_packet rxMsg;

};

void supercapISR(uint8_t* rxdata);

extern SuperCapCommThread* SuperCapCommInstance;

#endif /* THREADS_INC_DUMMY_THREAD_H_ */

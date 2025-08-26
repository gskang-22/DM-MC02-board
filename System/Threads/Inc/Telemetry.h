/*
 * Telemetry.h
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#ifndef CORE_INC_TELEMETRY_H_
#define CORE_INC_TELEMETRY_H_

#include "RoCo.h"
#include "usart.h"
//#include "motor_control.h"
#include "crc.h"

#define CAP1_NODE_ID       (0x003)
#define CAP2_NODE_ID       (0x008)
#define CURRENT_NODE_ID     CAP2_NODE_ID
#define OTHER_NODE_ID		CAP1_NODE_ID

#define MAKE_IDENTIFIABLE(PACKET) (PACKET).id = CURRENT_NODE_ID
#define MAKE_RELIABLE_MCU(PACKET) (PACKET).crc = (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t*) &(PACKET), sizeof((PACKET)) - 2)
#define IS_RELIABLE_MCU(PACKET) (PACKET).crc == (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t*) &(PACKET), sizeof((PACKET)) - 2)

class Telemetry {
public:

	static void setup();

	static void set_id(uint32_t id);


private:
};

//extern NetworkBus* UART6_network;
extern NetworkBus* UART_network;

extern CANBus* CAN1_network;
extern CANBus* CAN2_network;

#endif /* CORE_INC_TELEMETRY_H_ */

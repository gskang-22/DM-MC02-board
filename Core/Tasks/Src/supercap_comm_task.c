/*
 * supercap_comm_task.c
 *
 *  Created on: Jul 16, 2025
 *      Author: gskan
 */

#include "board_lib.h"
#include "supercap_comm_task.h"
#include "robot_config.h"
#include "can_msg_processor.h"

#include "motor_config.h"

float chassis_power;
uint8_t charging_state;
uint32_t supercap_last_receive_time = 0;
int supercap_enabled = 1;

#ifdef SUPERCAP_PRESENT

void supercap_comm_task(void *argument) {
	 uint8_t enable_supercap_module = 1;
	 uint8_t reset_supercap_module = 0;
	 extern ref_game_robot_data2_t ref_robot_data;
	 extern ref_game_state_t ref_game_state;

	 FDCAN_TxHeaderTypeDef TxHeader;
	 ref_msg_packet txMsg;

	 txHeaderConfig(&TxHeader);

	 while (1) {
		 // Disable use of supercap if charge falls below threshold
		 // DOES NOT DISABLE THE SUPERCAP
		 if (charging_state < SUPERCAP_DISABLE_THRESHOLD) {
			 supercap_enabled = 0;
		 } else if (!supercap_enabled && charging_state > SUPERCAP_DISABLE_THRESHOLD) {
			 supercap_enabled = 1;
		 }

		if ((HAL_GetTick() - supercap_last_receive_time > SUPERCAP_TIMEOUT)) {
			reset_supercap_module = 1;
		}

		 txMsg.enable_module = enable_supercap_module;
		 txMsg.reset = 0;
		 if (reset_supercap_module)
			 reset_supercap_module = 0;
		 txMsg.pow_limit = ref_robot_data.chassis_power_limit + SUPER_CAP_OFFSET; // -n cuz supercap not very accurate
		 txMsg.energy_buffer = 100; // hmm where this 100 came from

		 while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
		     // wait or handle timeout
		 }

		// Transmit FDCAN message
		 HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, (uint8_t *)&txMsg);
		// todo allow supercap can ID to be set in config file

		 if(ref_robot_data.current_HP <= 0 || ref_game_state.game_progress == 5)
			 enable_supercap_module = 0;
		 else
			 enable_supercap_module = 1;

		 osDelay(100);

		 portYIELD();
	 }
}

void txHeaderConfig(FDCAN_TxHeaderTypeDef* TxHeader) {
    TxHeader->Identifier = SUPERCAP_NODE_ID;          // Standard ID
    TxHeader->IdType = FDCAN_STANDARD_ID;            // Standard vs Extended
    TxHeader->TxFrameType = FDCAN_DATA_FRAME;        // Data frame
    TxHeader->DataLength = FDCAN_DLC_BYTES_5;        // 5 bytes of data
    TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE; // Error state indicator
    TxHeader->BitRateSwitch = FDCAN_BRS_OFF;         // No bit rate switch (classic CAN)
    TxHeader->FDFormat = FDCAN_CLASSIC_CAN;          // Classic CAN frame
    TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS; // Don't record TX events
}

 void supercapISR(uint8_t* rxdata){
	 supercap_msg_packet *supercap_packet = ( supercap_msg_packet*)rxdata;
	 chassis_power = supercap_packet->chassis_power;
	 charging_state = supercap_packet->cap_energy * 100 / 255;
	 supercap_last_receive_time = HAL_GetTick();
 }

#endif


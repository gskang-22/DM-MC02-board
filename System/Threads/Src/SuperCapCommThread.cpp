/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

 #include <SuperCapCommThread.h>
 //#include <supercap_def.h>
 #include <Telemetry.h>
 #include "referee_msgs.h"
 
 uint8_t enable_supercap_module = true;
 uint8_t reset_supercap_module = false;
 
 SuperCapCommThread* SuperCapCommInstance = nullptr;
 
 uint8_t enable_module;
 uint8_t reset;
 
 float chassis_power;
 float cap_voltage;
 uint8_t charging_state;
 extern ref_game_robot_data2_t ref_robot_data;
 extern ref_game_state_t ref_game_state;
 
 SuperCapCommThread::~SuperCapCommThread(){
 }
 
 void SuperCapCommThread::init(){
	 txHeaderConfig();
 }
 
 void SuperCapCommThread::loop()
 {
	 txMsg.enable_module = enable_supercap_module;
	 txMsg.reset = reset_supercap_module;
	 if (reset_supercap_module)
		 reset_supercap_module = false;
	 txMsg.pow_limit = ref_robot_data.chassis_power_limit;
	 txMsg.energy_buffer = 100;
	 uint32_t TxMailbox;  // Declare TxMailbox here
 
	 // Transmit data
	 while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
		 // Optionally add a timeout here to prevent infinite loop
	 }
 
	 if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, (uint8_t *)&txMsg, &TxMailbox) != HAL_OK)
		 int i = 1;
 
	 if(ref_robot_data.current_HP <= 0 || ref_game_state.game_progress == 5)
		 enable_supercap_module = false;
	 else
		 enable_supercap_module = true;
	 osDelay(100);
 
	 portYIELD();
 }
 
 void SuperCapCommThread::txHeaderConfig(){
	 TxHeader.StdId = SUPERCAP_NODE_ID;
	 TxHeader.ExtId = 0;
	 TxHeader.RTR = CAN_RTR_DATA;
	 TxHeader.IDE = CAN_ID_STD;
	 TxHeader.DLC = 5;
	 TxHeader.TransmitGlobalTime = DISABLE;
 }
 
 void supercapISR(uint8_t* rxdata){
	 supercap_msg_packet *supercap_packet = (struct supercap_msg_packet*)rxdata;
	 uint8_t i = 0;
	 chassis_power = supercap_packet->chassis_power;
	 charging_state = supercap_packet->cap_energy*100/255;
 }

/*
 * RoCanDriver.cpp
 *
 *  Created on: Jun 4, 2023
 *      Author: YassineBakkali
 */

#include "RoCanDriver.h"

#ifdef BUILD_WITH_CAN
#include <cstring>
#include <inttypes.h>
#include "stdio.h"
#include <algorithm>


#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "can_msg_processor.h"
#include "bsp_lk_motor.h"
#include "SuperCapCommThread.h"
#include "bsp_damiao.h"

extern EventGroupHandle_t gimbal_event_group;
extern EventGroupHandle_t chassis_event_group;
extern EventGroupHandle_t launcher_event_group;
#define ANGLE_LPF 0
#define SPEED_LPF 0
extern motor_data_t g_can_motors[24];
//extern motor_t motor[num];
extern dm_motor_t dm_pitch_motor;
extern dm_motor_t dm_yaw_motor;
//extern motor_map_t lk_motor_map[65];
extern motor_map_t dji_motor_map[25];

//#include <can_message_processor.h>


static ROCANDriver* instance;

ROCANDriver::ROCANDriver(FDCAN_HandleTypeDef* can, uint32_t can_id): can(can), can_id(can_id) {
	instance = this;
//	LOG_INFO("Driver created for FDCAN%d", getSenderID(fdcan));
	CANDriver_list.push_back(this);
	this->RxData = (uint8_t*) pvPortMalloc(RX_ELEMENT_NUMBER*RX_ELEMENT_SIZE);

    if((RxData == nullptr)){
//        LOG_ERROR("Unable to allocate Rx buffer for FDCAN%d", getSenderID(fdcan));
    }
    init();
}

ROCANDriver::~ROCANDriver() {
    vPortFree(RxData);
    CANDriver_list.erase(std::remove(CANDriver_list.begin(), CANDriver_list.end(), this), CANDriver_list.end());
}

void ROCANDriver::init() {

	/* Configure Rx filter */
	if(can == &hcan1)
		MX_CAN1_Init();
	else if (can == &hcan2)
		MX_CAN2_Init();
	/* Start the FDCan line */
	//MX_CAN2_Init();
	filterConfig(0, 0);
	TxHeaderConfig();
	TxHeaderConfigID(0);
//	start();
}

void ROCANDriver::filterConfig(uint32_t id, uint32_t mask){

	// Node ID (can_id)

    CAN_FilterTypeDef can_filter_st = {0};
    can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = (id >> 16);
	can_filter_st.FilterIdLow = (id & 0xFFFF);
	can_filter_st.FilterMaskIdHigh = (mask >> 16);
	can_filter_st.FilterMaskIdLow = (mask & 0xFFFF);

	if (can->Instance == CAN1) {
	    can_filter_st.FilterBank = 0;
	    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	    if(HAL_CAN_ConfigFilter(can, &can_filter_st) != HAL_OK) {
	           // LOG_ERROR("Unable to configure CAN RX filters for CAN%d", getSenderID(can));
	       }
	    start();
	    HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_ERROR);
	} else if (can->Instance == CAN2) {
	    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1; // Uncomment line if using RX1 queue as well.
		can_filter_st.SlaveStartFilterBank = 14;
		can_filter_st.FilterBank = 14;
		if(HAL_CAN_ConfigFilter(can, &can_filter_st) != HAL_OK) {
			           // LOG_ERROR("Unable to configure CAN RX filters for CAN%d", getSenderID(can));
	   }
		start();
	    HAL_CAN_ActivateNotification(can, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR); // Uncomment line if using RX1 queue as well.
	}
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1; // Uncomment line if using RX1 queue as well.
	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK) {
		           // LOG_ERROR("Unable to configure CAN RX filters for CAN%d", getSenderID(can));
   }
	HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR); // Uncomment line if using RX1 queue as well.
}


uint32_t ROCANDriver::get_can_id() {
    return can_id;
}

void ROCANDriver::TxHeaderConfig() {
    TxHeader.StdId = 0;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;
}

void ROCANDriver::TxHeaderConfigID(uint32_t id) {
    if(id < 0x800) {
        TxHeader.StdId = id;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.RTR = CAN_RTR_DATA;
    } else {
        TxHeader.ExtId = id;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;
    }
}

void ROCANDriver::TxHeaderConfigLength(uint32_t length) {
    if(length <= 8) {
        TxHeader.DLC = length;
    } else {
        TxHeader.DLC = 8; // CAN supports only up to 8 bytes
    }
}

void ROCANDriver::start() {
    if(HAL_CAN_Start(can) != HAL_OK) {
        // LOG_ERROR("Couldn't start CAN%d module", getSenderID(can));
    	int a = 8;
    }
}

//void ROCANDriver::loop() {
//	while(true){
//		if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE) {
//			uint8_t sender = getSenderID(can);
//			uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0);
//			while (fill_level > 0) {
//				HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &RxHeader, RxData);
//				uint32_t length = RxHeader.DLC;
//				receiveCAN(sender, RxData, length);
//				fill_level = HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0);
//			}
//		}
//	}
//}

void ROCANDriver::ISR(FDCAN_HandleTypeDef *hcan){
//	uint8_t sender = getSenderID(can);
	if (hcan->Instance == CAN1) {
		uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
		while (fill_level > 0) {
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

			if (RxHeader.StdId >= 0x200 && RxHeader.StdId <= 0x20E){
				if (dji_motor_map[RxHeader.StdId - 0x200].motor_data != NULL){
					convert_raw_can_data(dji_motor_map[RxHeader.StdId - 0x200].motor_data, RxHeader.StdId, (uint8_t*)RxData);
				}
			} else {
//				 if (RxHeader.StdId > 0x140 && RxHeader.StdId <= 0x160){
//					if (lk_motor_map[RxHeader.StdId - 0x140].motor_data != NULL){
//						process_lk_motor(RxData, lk_motor_map[RxHeader.StdId-0x140].motor_data);
//						BaseType_t xHigherPriorityTaskWoken, xResult;
//						xHigherPriorityTaskWoken = pdFALSE;
//						xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
//								&xHigherPriorityTaskWoken);
//					}
//				}
				if (RxHeader.StdId == DEVC_NODE_ID){
					supercapISR(RxData);
				} else if ((RxHeader.StdId >= 0x90 && RxHeader.StdId <= 0x94)|| (RxHeader.StdId >= 0x70 && RxHeader.StdId <= 0x75)){
					int fb_id = (RxData[0])&0x0F;
					switch(fb_id){
					case 1:
					case 5:
						dm4310_fbdata(&dm_pitch_motor,&RxData[0]);
						break;
					}
				//handle LK motor or other data
				} else {
					// should not run here
					uint8_t sender = getSenderID(hcan);
					uint32_t length = RxHeader.DLC;
					receiveCAN(sender, RxData, length);
				}
		//		HAL_CAN_ActivateNotification(hcan,
		//				CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL
		//						| CAN_IT_RX_FIFO0_OVERRUN);
			}
			fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
		}
	}
	if (hcan->Instance == CAN2) {
		uint32_t fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
		while (fill_level > 0){
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	//		HAL_CAN_DeactivateNotification(hcan,
	//				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL
	//						| CAN_IT_RX_FIFO1_OVERRUN);
			if (RxHeader.StdId >= 0x200 && RxHeader.StdId <= 0x210){
			//StdId +12 to seperate the motors on CAN1 and CAN2
				if (dji_motor_map[RxHeader.StdId - 0x200+12].motor_data != NULL){
					convert_raw_can_data(dji_motor_map[RxHeader.StdId - 0x200+12].motor_data, RxHeader.StdId+12, RxData);
				}
			} else {
//				if (RxHeader.StdId > 0x140 && RxHeader.StdId <= 0x160){
//			//handle LK motor or other data
//					if (lk_motor_map[RxHeader.StdId - 0x140].motor_data != NULL){
//						process_lk_motor(RxData, lk_motor_map[RxHeader.StdId-0x140].motor_data);
//						BaseType_t xHigherPriorityTaskWoken, xResult;
//						xHigherPriorityTaskWoken = pdFALSE;
//						xResult = xEventGroupSetBitsFromISR(gimbal_event_group, 0b01,
//								&xHigherPriorityTaskWoken);
//					}
//				}
				if (RxHeader.StdId == DEVC_NODE_ID){
					supercapISR(RxData);
				} else if (RxHeader.StdId >= 0x70 && RxHeader.StdId <= 0x74){
					int fb_id = (RxData[0])&0x0F;
					switch(fb_id){
					case 1:
						dm4310_fbdata(&dm_yaw_motor,&RxData[0]);
						break;
						}
				} else {
					uint8_t sender = getSenderID(hcan);
					uint32_t length = RxHeader.DLC;
					receiveCAN(sender, RxData, length);
				}
			}
		//		HAL_CAN_ActivateNotification(hcan,
		//				CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL
		//						| CAN_IT_RX_FIFO1_OVERRUN);
			fill_level = HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1);
		}
	}
}
void ROCANDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
    this->receiver_func = receiver;
}

void ROCANDriver::transmit(uint8_t* buffer, uint32_t length) {
    uint32_t TxMailbox;  // Declare TxMailbox here

    for (uint32_t offset = 0; offset < length; offset += 8) {
            // Calculate the size of the current chunk (either 8 or less for the last chunk)
            uint32_t chunkSize = (length - offset > 8) ? 8 : (length - offset);

            // Configure the header for the current chunk size
            TxHeaderConfigLength(chunkSize);

            // Wait for a free mailbox
            while (HAL_CAN_GetTxMailboxesFreeLevel(can) == 0) {
                // Optionally add a timeout here to prevent infinite loop
            }

            // Transmit the current chunk
            if (HAL_CAN_AddTxMessage(can, &TxHeader, &buffer[offset], &TxMailbox) != HAL_OK) {
                // Handle transmission error
                break;
            }
        }
}

uint8_t* ROCANDriver::getRxBuffer() {
    return this->RxData;
}

ROCANDriver* ROCANDriver::getInstance(FDCAN_HandleTypeDef* can) {
    for (auto & driver : CANDriver_list) {
        if (driver->getCAN() == can)
            return driver;
    }
    return nullptr;
}

void ROCANDriver::receiveCAN(uint8_t sender_id, uint8_t* buffer, uint32_t length) {
    this->receiver_func(sender_id, buffer, length);
}

FDCAN_HandleTypeDef* ROCANDriver::getCAN() {
    return this->can;
}

// Fifo0 Rx Callback
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan) {
    ROCANDriver* driver = ROCANDriver::getInstance(hcan);
    driver->ISR(hcan);
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;  // Required for ISR context
//    xSemaphoreGiveFromISR(driver->getSemaphore(), nullptr);
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hcan) {
    ROCANDriver* driver = ROCANDriver::getInstance(&hcan1);
    driver->ISR(&hcan2);
}

void HAL_CAN_RxFifo0FullCallback(FDCAN_HandleTypeDef *hcan)
{
	int a = 98;
}

void HAL_CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan) {
    if(hcan == &hcan1) {
        MX_CAN1_Init();
        ROCANDriver* driver = ROCANDriver::getInstance(hcan);
//        while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
        driver->filterConfig(driver->get_can_id(), 0x0);
        driver->start();
    } else if (hcan == &hcan2) {
        MX_CAN2_Init();
        ROCANDriver* driver = ROCANDriver::getInstance(hcan);
//        while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
        driver->filterConfig(driver->get_can_id(), 0x0);
        driver->start();
    }
}

/**
 * @brief Get the sender id from the CAN port ID
 *
 * @param fdcan the FDCAN port to get
 * @return uint8_t the sender_id
 */
uint8_t ROCANDriver::getSenderID(FDCAN_HandleTypeDef* can) {
    for(int i = 0; i < NB_CAN_PORTS; ++i){
        if(this->mapper[i] == can->Instance){
            return i+1;
        }
    }
    return 0;
}


std::vector<ROCANDriver*> ROCANDriver::CANDriver_list;

#endif

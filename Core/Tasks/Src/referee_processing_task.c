/*
 * referee_processing_task.c
 *
 *  Created on: Jun 18, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_queue.h"
#include "bsp_referee.h"
#include "bsp_usart.h"
#include "referee_processing_task.h"
#include "referee_msgs.h"
#include "robot_config.h"
#include "rtos_g_vars.h"

extern int g_spinspin_mode;
extern uint8_t remote_raw_data[18];
extern TaskHandle_t referee_processing_task_handle;
extern DMA_HandleTypeDef hdma_usart6_rx;
static ref_msg_t g_ref_msg_buffer;

ref_game_state_t ref_game_state;
uint32_t ref_game_state_txno = 0;

ref_game_robot_HP_t ref_robot_hp;
uint32_t ref_robot_hp_txno = 0;

ref_game_robot_data2_t ref_robot_data;
uint32_t ref_robot_data_txno = 0;

ref_robot_power_data_t ref_power_data;
uint32_t ref_power_data_txno = 0;

ref_game_robot_pos_t ref_robot_pos;
uint32_t ref_robot_pos_txno = 0;

ref_buff_data_t ref_buff_data;
uint32_t ref_buff_data_txno = 0;

ref_robot_dmg_t ref_dmg_data;
uint32_t ref_dmg_data_txno = 0;

ref_shoot_data_t ref_shoot_data;
uint32_t ref_shoot_data_txno = 0;

ref_magazine_data_t ref_mag_data;
uint32_t ref_mag_data_txno = 0;
uint8_t g_ref_tx_seq = 0;

uint8_t ref_buffer[2];
queue_t referee_uart_q;

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart){
	if (huart== DBUS_UART){
		HAL_UART_DMAStop(DBUS_UART);
		dbus_remote_start();
	} else if (huart == REFEREE_UART){
	    __HAL_DMA_DISABLE(&hdma_usart6_rx);
		ref_usart_start(REFEREE_UART, ref_buffer, 2, &referee_uart_q);
	}
}

void referee_processing_task(void *argument) {
	ref_processing_status_t proc_status;
//	g_referee_limiters.feeding_speed = LV1_FEEDER;
//	g_referee_limiters.projectile_speed = LV1_PROJECTILE;
//	g_referee_limiters.wheel_power_limit = LV1_POWER;
//	g_referee_limiters.robot_level = 1;
	status_led(7, on_led);
	status_led(8, off_led);
	ref_robot_data.robot_id = 0;
	ref_usart_start(REFEREE_UART, ref_buffer, 2, &referee_uart_q);
	while (1) {

		uint8_t has_data = ulTaskNotifyTake(pdTRUE, 1000);
		status_led(5, on_led);
		if (queue_get_size(&referee_uart_q) > 7) {
			while (queue_get_size(&referee_uart_q) > 7) {
				proc_status = ref_process_data(&referee_uart_q, &g_ref_msg_buffer);
				if (proc_status == PROCESS_SUCCESS) {
					switch (g_ref_msg_buffer.cmd_id) {
					case REF_ROBOT_SHOOT_DATA_CMD_ID:
						memcpy(&ref_shoot_data, &g_ref_msg_buffer.data,
								sizeof(ref_shoot_data_t));
						ref_shoot_data_txno++;
						break;
					case REF_GAME_STATE_CMD_ID:
						memcpy(&ref_game_state, &g_ref_msg_buffer.data,
								sizeof(ref_game_state_t));
						ref_game_state_txno++;
						break;
					case REF_ROBOT_DATA_CMD_ID:
						memcpy(&ref_robot_data, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_data2_t));
						ref_robot_data_txno++;
						break;
					case REF_ROBOT_POS_DATA_CMD_ID:
						memcpy(&ref_robot_pos, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_pos_t));
						ref_robot_pos_txno++;
						break;
					case REF_ROBOT_POWER_DATA_CMD_ID:
						memcpy(&ref_power_data, &g_ref_msg_buffer.data,
								sizeof(ref_robot_power_data_t));
						ref_power_data_txno++;
						break;

					case REF_ROBOT_DMG_DATA_CMD_ID:
						memcpy(&ref_dmg_data, &g_ref_msg_buffer.data,
								sizeof(ref_robot_dmg_t));
						ref_dmg_data_txno++;
#ifdef SPIN_WHEN_DAMAGED
						if (ref_dmg_data.dmg_type == 0){
							g_spinspin_mode = 1;
						}
#endif
						break;

					case REF_ROBOT_HP_CMD_ID:
						memcpy(&ref_robot_hp, &g_ref_msg_buffer.data,
								sizeof(ref_game_robot_HP_t));
						ref_robot_hp_txno++;
						break;
					case REF_ROBOT_MAGAZINE_DATA_CMD_ID:
						memcpy(&ref_mag_data, &g_ref_msg_buffer.data,
								sizeof(ref_magazine_data_t));
						ref_mag_data_txno++;
						//add in the memcpys here
						break;
					default:
						break;
					}
//						if (msg_buffer.cmd_id == REF_ROBOT_SHOOT_DATA_CMD_ID){
//							xQueueSend(uart_data_queue, &msg_buffer, 0);
//						}
				} else if (proc_status == INSUFFICIENT_DATA) {
					break;
				}
			}
		}
		if (!has_data){
		    __HAL_DMA_DISABLE(&hdma_usart6_rx);
			ref_usart_start(REFEREE_UART, ref_buffer, 2, &referee_uart_q);

		}

		status_led(5, off_led);


		status_led(5, on_led);
// for having varying firing speed, which we don't need now
//#ifdef LVL_TUNING
//		if (ref_robot_data.robot_level == 1) {
//			g_referee_limiters.feeding_speed = LV1_FEEDER;
//			g_referee_limiters.projectile_speed = LV1_PROJECTILE;
//			g_referee_limiters.robot_level = 1;
//			status_led(7, on_led);
//			status_led(8, off_led);
//		} else if (ref_robot_data.robot_level == 2) {
//			g_referee_limiters.feeding_speed = LV2_FEEDER;
//			g_referee_limiters.projectile_speed = LV2_PROJECTILE;
//			g_referee_limiters.robot_level = 2;
//			status_led(7, off_led);
//			status_led(8, on_led);
//		} else if (ref_robot_data.robot_level == 3) {
//			g_referee_limiters.feeding_speed = LV3_FEEDER;
//			g_referee_limiters.projectile_speed = LV3_PROJECTILE;
//			g_referee_limiters.robot_level = 3;
//			status_led(7, on_led);
//			status_led(8, on_led);
//		} else {
//			g_referee_limiters.feeding_speed = LV1_FEEDER;
//			g_referee_limiters.projectile_speed = LV1_PROJECTILE;
//		}
//#endif
	}
}




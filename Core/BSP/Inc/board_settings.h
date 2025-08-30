/*
 * board_settings.h
 *
 *  Created on: Jul 5, 2021
 *      Author: wx
 */

#ifndef BSP_INC_BOARD_SETTINGS_H_
#define BSP_INC_BOARD_SETTINGS_H_

#define DBUS_UART 			&huart10
#define SBC_UART			&huart7
#define REFEREE_UART		&huart1
#define REMOTE_DATA_SIZE 	18
#define IMU_HSPI 			&hspi2
//#define IST_I2C			hi2c3

#define SERVO_TIMER			&htim2
#define HERO_ZOOM_CHANNEL	TIM_CHANNEL_1 // controls servo for new hero zoom-in lens (PA0)
#define HERO_VTM_CHANNEL	TIM_CHANNEL_3 // controls servo for new hero self-adjusting VTM (PA2)

#define BUZZER_TIMER		&htim12
#define BUZZER_CHANNEL		TIM_CHANNEL_3

#endif /* BSP_INC_BOARD_SETTINGS_H_ */

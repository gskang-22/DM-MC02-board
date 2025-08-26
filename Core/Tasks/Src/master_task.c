/*
 * master_task.c
 *
 *  Created on: 14 Sep 2023
 *      Author: cwx
 */

#include "board_lib.h"
//#include "startup_task.h"
#include "gimbal_control_task.h"
#include "movement_control_task.h"
#include "referee_processing_task.h"
#include "control_input_task.h"
#include "launcher_control_task.h"
#include "imu_processing_task.h"
#include "robot_config.h"
#include "buzzing_task.h"
#include "motor_config.h"
#include "usb_task.h"
#include "telemetry_task.h"
#include "motor_control_task.h"
#include "INS_task.h"
#include "hud_new.h"
#include "supercap_comm_task.h"

#define ISR_SEMAPHORE_COUNT 1
#define QUEUE_SIZE 1

extern TaskHandle_t master_task_handle;
TaskHandle_t gimbal_control_task_handle;
TaskHandle_t movement_control_task_handle;
TaskHandle_t referee_processing_task_handle;
TaskHandle_t control_input_task_handle;
TaskHandle_t launcher_control_task_handle;
TaskHandle_t buzzing_task_handle;
TaskHandle_t motor_calib_task_handle;
TaskHandle_t usb_task_handle;
TaskHandle_t imu_processing_task_handle;
TaskHandle_t telemetry_task_handle;
TaskHandle_t motor_control_task_handle;
TaskHandle_t hud_task_handle;
TaskHandle_t supercap_task_handle;
TaskHandle_t dm_motor_control_task_handle;
TaskHandle_t INS_task_handle;

EventGroupHandle_t gimbal_event_group;
EventGroupHandle_t chassis_event_group;
EventGroupHandle_t launcher_event_group;

SemaphoreHandle_t usb_continue_semaphore;

QueueHandle_t gyro_data_queue;
QueueHandle_t accel_data_queue;
QueueHandle_t mag_data_queue;

QueueHandle_t telem_data_queue;
QueueHandle_t g_buzzing_task_msg;
QueueHandle_t xvr_data_queue;
QueueHandle_t uart_data_queue;

extern gimbal_control_t gimbal_ctrl_data;


void master_task(void* argument){
	imu_init();

	gimbal_event_group = xEventGroupCreate();
	chassis_event_group = xEventGroupCreate();
	launcher_event_group = xEventGroupCreate();

	usb_continue_semaphore = xSemaphoreCreateBinary();

	gyro_data_queue = xQueueCreate(5, sizeof(gyro_data_t));
	accel_data_queue = xQueueCreate(5, sizeof(accel_data_t));
	mag_data_queue = xQueueCreate(5, sizeof(mag_data_t));
	//telem_data_queue = xQueueCreate(10, sizeof(telem_data_struct_t));
	g_buzzing_task_msg = xQueueCreate(48, sizeof(uint8_t));
//	uart_data_queue = xQueueCreate(5, sizeof(ref_msg_t));

	gimbal_ctrl_data.yaw_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(gimbal_ctrl_data.yaw_semaphore);
	gimbal_ctrl_data.pitch_semaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(gimbal_ctrl_data.pitch_semaphore);

	/* add threads, ... */
	//todo: adjust priorities
	//Threads creation\

#ifdef SENTRY
	xTaskCreate(INS_task, "INS_task",
	        configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
	        &INS_task_handle);
#endif

	xTaskCreate(imu_processing_task, "IMU_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 13,
			&imu_processing_task_handle);

	xTaskCreate(motor_calib_task, "motor_calib_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 9,
			&motor_calib_task_handle);

	xTaskCreate(control_input_task, "RC_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 4,
			&control_input_task_handle);
	xTaskCreate(referee_processing_task, "referee_task", 512, (void*) 1,
			(UBaseType_t) 2, &referee_processing_task_handle);
	xTaskCreate(buzzing_task, "buzzer_task",
	configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 1, &buzzing_task_handle);
	if (usb_continue_semaphore == NULL) {
		//error handler
	} else {
		xTaskCreate(usb_task, "usb_task",
		configMINIMAL_STACK_SIZE, (void*) 1, (UBaseType_t) 1, &usb_task_handle);
	}

//	xTaskCreate(telemetry_task, "telemetry_task", 700, (void*) 1,
//			(UBaseType_t) 5, &telemetry_task_handle);

#ifdef SUPERCAP_PRESENT
	xTaskCreate(supercap_comm_task, "supercap_comm_task",
			configMINIMAL_STACK_SIZE, NULL, (UBaseType_t) 1, &supercap_task_handle);
#endif

	xTaskCreate(new_hud_task, "new_hud_task", 512, (void*) 3,
			(UBaseType_t) 5, &hud_task_handle);

//	vTaskDelete(master_task_handle);
	while(1){
		vTaskDelay(1000);
	}

}

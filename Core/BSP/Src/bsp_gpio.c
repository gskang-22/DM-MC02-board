/*
 * bsp_gpio.c
 *
 *  Created on: Sep 7, 2021
 *      Author: wx
 */
#include "board_lib.h"
#include "bsp_gpio.h"
#include "bsp_imu.h"

extern uint8_t imu_init_status;

void laser_on()
{ //set to reset for open day
//	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}

void laser_off()
{
//	HAL_GPIO_WritePin(LASER_GPIO_GPIO_Port, LASER_GPIO_Pin, GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// handle interrupts for IMU and Magnetometer
	if (imu_init_status == 1) {
		if (IMU_HSPI.State == HAL_SPI_STATE_READY) {
			if (GPIO_Pin == GYRO_INT_Pin) {
				gyro_get_data();
				gyro_process_data();
				//ist8310_get_data();
			}
			if (GPIO_Pin == ACC_INT_Pin) {
				accel_get_data();
				accel_process_data();
			}
		}
//		if (GPIO_Pin == IST_INT_Pin) {
//			if (hi2c3.State == HAL_I2C_STATE_READY) {
//				ist8310_get_data();
//			}
//		}
	}
}

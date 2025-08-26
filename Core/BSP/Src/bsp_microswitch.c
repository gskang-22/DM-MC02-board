/*
 * bsp_microswitch.c
 *
 *  Created on: Dec 8, 2024
 *      Author: cw
 */
#include "bsp_microswitch.h"
uint8_t projectile_loaded;
uint8_t gimbal_upper_bound;
uint8_t gimbal_lower_bound;

void microswitch_int() {
//	uint8_t pin_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
//	// Falling edge, projectile loaded
//	if (pin_state == 0) {
//		projectile_loaded = 1;
//	} else if (pin_state == 1) {
//		projectile_loaded = 0;
//	}
}

void microswitch_int1() {
//	uint8_t pin_state = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
//	// Falling edge, gimbal_lower_bound
//	if (pin_state == 0) {
//		gimbal_lower_bound = 1;
//	} else if (pin_state == 1) {
//		gimbal_lower_bound = 0;
//	}
}

void microswitch_int2() {
//	uint8_t pin_state = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_6);
//	// Falling edge, gimbal_upper_bound
//	if (pin_state == 0) {
//		gimbal_upper_bound = 1;
//	} else if (pin_state == 1) {
//		gimbal_upper_bound = 0;
//	}
}

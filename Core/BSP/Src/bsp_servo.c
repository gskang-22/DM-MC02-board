/*
 * bsp_servo.c
 *
 *  Created on: Mar 13, 2025
 *      Author: gskan
 */




#include <bsp_servo.h>
#include "board_lib.h"


void servo_init()
{
	  HAL_TIM_PWM_Start(SERVO_TIMER,HERO_ZOOM_CHANNEL);
	  HAL_TIM_PWM_Start(SERVO_TIMER,HERO_VTM_CHANNEL);
}

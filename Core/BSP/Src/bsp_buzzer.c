/*
 * bsp_buzzer.c
 *
 *  Created on: May 23, 2021
 *      Author: wx
 */

#include "board_lib.h"
#include "bsp_buzzer.h"

void buzzer(uint16_t freq)
{
	if (freq == 0)
	{
        // Turn off PWM by setting duty to 0
        __HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CHANNEL, 0);
	}
	else
	{
        __HAL_TIM_SET_PRESCALER(BUZZER_TIMER, (84 * 500 / freq));
        __HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CHANNEL, 500);
	}
}


void buzzer_init()
{
	  HAL_TIM_PWM_Start(BUZZER_TIMER, BUZZER_CHANNEL);
	  __HAL_TIM_SET_COMPARE(BUZZER_TIMER, BUZZER_CHANNEL, 0);
}



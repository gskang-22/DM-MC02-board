/*
 * j60_10motor_task.c
 *
 *  Created on: Jul 4, 2025
 *      Author: YI MING
 */
#include "j60_10motor_task.h"
#include "cmsis_os.h"
#include "j60_10.h"
#include <math.h>

motor_t j60_motor[6];
static float time_counter = 0.0f;  // Time counter for sine wave
float target_position = 0.0f;

void j60_10_TASK(void) {
	Init_J60_Motor(&j60_motor[0], 1, 1);
	Enable_J60_Motor(&j60_motor[0]);
	

    for (;;) {
        // Generate sine wave: amplitude = 1 rad, period = 2π seconds (frequency ≈ 0.16 Hz)
        target_position = 3.142f * sinf(time_counter);
        
        j60_motor[0].cmd.pos_set = target_position;  // Set sine wave position
        j60_motor[0].cmd.vel_set = 0.0f;    // No velocity
        j60_motor[0].cmd.kp_set = 0.0f;   // Position gain
        j60_motor[0].cmd.kd_set = 0.0f;     // Damping gain
        j60_motor[0].cmd.tor_set = 0.0f;    // No torque
        Send_J60_Motor_Command(&j60_motor[0]);
        
        // Increment time (10ms per loop = 0.01 seconds)
        time_counter += 0.01f;
        
        osDelay(10);  // Wait 10ms for reasonable CAN bus timing
    }
}

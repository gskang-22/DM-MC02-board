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
#include "buzzer.h"

motor_t j60_motor[6];
static float time_counter = 0.0f;  // Time counter for sine wave
float target_position = 0.0f;

void j60_10_TASK(void) {
	Init_J60_Motor(&j60_motor[0], 1, 1, 3.142);
	Enable_J60_Motor(&j60_motor[0]);
    
    // Play startup melody for 2 seconds before starting motor control
    Buzzer_PlayStartupMelody();

    for (;;) {
    	if (j60_motor[0].para.enable_failed == 1 || j60_motor[0].para.online == 0){
    		J60_Cmd_Clear(&j60_motor[0]);
    		osDelay(10);
    		continue;
    	}
        
        // Safety check: Stop motor if torque exceeds 0.5 N·m
        if (J60_Safety_Check(&j60_motor[0], 1.0f) == 1) {
            // Motor is in safety stop - do not send new commands
            // Optional: Add alert sound for safety stop
            Buzzer_Alert();
            osDelay(10);
            continue;
        }
        
        // Generate sine wave: amplitude = 1 rad, period = 2π seconds (frequency ≈ 0.16 Hz)
        target_position = (3.142f/2.0f) * sinf(time_counter);
        
        j60_motor[0].cmd.pos_set = target_position;  // Set sine wave position
        j60_motor[0].cmd.vel_set = 0.0f;    // No velocity
        j60_motor[0].cmd.kp_set = 50.0f;   // Position gain
        j60_motor[0].cmd.kd_set = 5.0f;     // Damping gain
        j60_motor[0].cmd.tor_set = 0.0f;    // No torque
        Send_J60_Motor_Command(&j60_motor[0]);
        
        // Increment time (10ms per loop = 0.01 seconds)
        time_counter += 0.01f;
        J60_10_MOTOR_ONLINE_CHECK();
        osDelay(10);  // Wait 10ms for reasonable CAN bus timing
    }
}

void J60_10_MOTOR_ONLINE_CHECK(void){
	if (j60_motor[0].para.ping > 5){
		j60_motor[0].para.online = 0;
	}else{
		j60_motor[0].para.online = 1;
	}
}

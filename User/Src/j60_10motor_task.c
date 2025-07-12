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
#include "pc_mcu_uart.h"

motor_t j60_motor[6];
static float time_counter = 0.0f;  // Time counter for sine wave
float target_position = 0.0f;

void j60_10_TASK(void) {
	// Initialize motors: (motor, id, can_channel, rad_offset, direction, pos_limit_min, pos_limit_max)
	// direction: 1 = forward, -1 = reverse (flips all commands and feedback)
	Init_J60_Motor(&j60_motor[0], 1, 1, 0.2164, -1, -0.32f, 0.2164f);  // Motor 0: ID=1, CAN1, limits: ±0.3 rad
	Init_J60_Motor(&j60_motor[1], 2, 1, 0.83, -1, -1.1f, 0.83f);    // Motor 1: ID=2, CAN1, limits: ±0.5 rad
	Init_J60_Motor(&j60_motor[2], 3, 1, -0.785, -1, -0.785f, 0.785f);  // Motor 2: ID=3, CAN1, limits: ±0.7 rad
	Enable_J60_Motor(&j60_motor[0]);
	osDelay(10);
	Enable_J60_Motor(&j60_motor[1]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[2]);
    
    // Play startup melody for 2 seconds before starting motor control
    Buzzer_PlayStartupMelody();

        for (;;) {
    	// Check motor 0 status
    	if (j60_motor[0].para.enable_failed == 1 || j60_motor[0].para.online == 0){
    		J60_Cmd_Clear(&j60_motor[0]);
    	}
    	
    	// Check motor 1 status
    	if (j60_motor[1].para.enable_failed == 1 || j60_motor[1].para.online == 0){
    		J60_Cmd_Clear(&j60_motor[1]);
    	}
    	
    	// Check motor 2 status
    	if (j60_motor[2].para.enable_failed == 1 || j60_motor[2].para.online == 0){
    		J60_Cmd_Clear(&j60_motor[2]);
    	}
    	
    	// If all motors are offline, skip this cycle
    	if ((j60_motor[0].para.enable_failed == 1 || j60_motor[0].para.online == 0) &&
    	    (j60_motor[1].para.enable_failed == 1 || j60_motor[1].para.online == 0) &&
    	    (j60_motor[2].para.enable_failed == 1 || j60_motor[2].para.online == 0)) {
    		osDelay(10);
    		continue;
    	}
        
                // Safety check for motor 0: Stop motor if torque exceeds 8.0 N·m
        uint8_t motor0_safety = J60_Safety_Check(&j60_motor[0], 8.0f);
        
        // Safety check for motor 1: Stop motor if torque exceeds 8.0 N·m
        uint8_t motor1_safety = J60_Safety_Check(&j60_motor[1], 8.0f);
        
        // Safety check for motor 2: Stop motor if torque exceeds 8.0 N·m
        uint8_t motor2_safety = J60_Safety_Check(&j60_motor[2], 8.0f);
        
        if (motor0_safety == 1 || motor1_safety == 1 || motor2_safety == 1) {
            // One or more motors in safety stop - alert
            Buzzer_Alert();
            osDelay(10);
            continue;
        }
        
        // Generate sine wave: amplitude = 1 rad, period = 2π seconds (frequency ≈ 0.16 Hz)
        target_position = (3.142f/2.0f) * sinf(time_counter);
        
        // Motor 0 commands - from PC data (will be limited to ±0.3 rad)
        j60_motor[0].cmd.pos_set = pc_mcu_rx_data[0];  // Position from PC
        j60_motor[0].cmd.vel_set = 0.0f;    // No velocity
        j60_motor[0].cmd.kp_set = 40.0f;   // Position gain
        j60_motor[0].cmd.kd_set = 4.0f;     // Damping gain
        j60_motor[0].cmd.tor_set = 0.0f;    // No torque
        
        // Motor 1 commands (will be limited to ±0.5 rad)
        j60_motor[1].cmd.pos_set = pc_mcu_rx_data[1];  // Position from PC
        j60_motor[1].cmd.vel_set = 0.0f;    // No velocity
        j60_motor[1].cmd.kp_set = 40.0f;   // Position gain
        j60_motor[1].cmd.kd_set = 4.0f;     // Damping gain
        j60_motor[1].cmd.tor_set = 0.0f;    // No torque
        
        // Motor 2 commands (will be limited to ±0.7 rad)
        j60_motor[2].cmd.pos_set = pc_mcu_rx_data[2];  // Position from PC
        j60_motor[2].cmd.vel_set = 0.0f;    // No velocity
        j60_motor[2].cmd.kp_set = 40.0f;   // Position gain
        j60_motor[2].cmd.kd_set = 4.0f;     // Damping gain
        j60_motor[2].cmd.tor_set = 0.0f;    // No torque
        
        // Send commands to all motors
        Send_J60_Motor_Command(&j60_motor[0]);
        Send_J60_Motor_Command(&j60_motor[1]);
        osDelay(1);
        Send_J60_Motor_Command(&j60_motor[2]);
        
        // Increment time (10ms per loop = 0.01 seconds)
        time_counter += 0.01f;
        J60_10_MOTOR_ONLINE_CHECK();
        osDelay(10);  // Wait 10ms for reasonable CAN bus timing
    }
}

void J60_10_MOTOR_ONLINE_CHECK(void){
	// Check motor 0 online status
	if (j60_motor[0].para.ping > 5){
		j60_motor[0].para.online = 0;
	}else{
		j60_motor[0].para.online = 1;
	}
	
	// Check motor 1 online status
	if (j60_motor[1].para.ping > 5){
		j60_motor[1].para.online = 0;
	}else{
		j60_motor[1].para.online = 1;
	}
	
	// Check motor 2 online status
	if (j60_motor[2].para.ping > 5){
		j60_motor[2].para.online = 0;
	}else{
		j60_motor[2].para.online = 1;
	}
}

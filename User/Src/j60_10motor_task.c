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
#include "dji_ndj_remote.h"  // Add remote control header

motor_t j60_motor[6];
static float time_counter = 0.0f;  // Time counter for sine wave
float target_position = 0.0f;

// Initialization state machine
typedef enum {
    INIT_PHASE = 0,
    POSITIONING_PHASE = 1,
    NORMAL_OPERATION = 2,
    ERROR_PHASE = 99
} robot_state_t;

// Motor position ramping parameters
static float current_positions[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static const float POSITION_INCREMENT = 0.006f;  // Position increment per cycle (rad) - 0.003 rad ≈ 0.17° per 10ms cycle

// Motor configuration - change this to select which motors to use
typedef enum {
    USE_MOTORS_123 = 0,  // Use motors 0,1,2 (IDs 1,2,3)
    USE_MOTORS_456 = 1,  // Use motors 3,4,5 (IDs 4,5,6)
    USE_ALL_MOTORS = 2   // Use all motors 0,1,2,3,4,5 (IDs 1,2,3,4,5,6)
} motor_config_t;

static const motor_config_t MOTOR_CONFIG = USE_MOTORS_123;  // Currently using motors 4,5,6
static robot_state_t robot_state = INIT_PHASE;
static uint32_t state_timer = 0;
static const float POSITION_TOLERANCE = 0.05f;  // ±0.05 rad tolerance

// Initial position values for motors
const float MOTOR0_INIT_POS = 0.24f;
const float MOTOR1_INIT_POS = 0.86f;
const float MOTOR2_INIT_POS = -0.785f;
const float MOTOR3_INIT_POS = -0.24f;
const float MOTOR4_INIT_POS = -0.86f;
const float MOTOR5_INIT_POS = 0.785f;

// Desired positions for positioning phase
const float MOTOR0_DESIRED_POS = 0.0f;
const float MOTOR1_DESIRED_POS = 0.0f;
const float MOTOR2_DESIRED_POS = -0.785f;
const float MOTOR3_DESIRED_POS = 0.0f;
const float MOTOR4_DESIRED_POS = 0.0f;
const float MOTOR5_DESIRED_POS = 0.785f;

// External declaration for remote control
extern dji_ndj6 dji_remote;

// Function prototypes
uint8_t Check_All_Motors_Enabled(void);
uint8_t Check_Motors_At_Position(void);
void Set_Motors_To_Desired_Position(void);

void j60_10_TASK(void) {
	// Initialize motors: (motor, id, can_channel, rad_offset, direction, pos_limit_min, pos_limit_max)
	// direction: 1 = forward, -1 = reverse (flips all commands and feedback)
	Init_J60_Motor(&j60_motor[0], 1, 1, MOTOR0_INIT_POS, -1, -0.32f, MOTOR0_INIT_POS);  // Motor 0: ID=1, CAN1
	Init_J60_Motor(&j60_motor[1], 2, 1, MOTOR1_INIT_POS, -1, -1.1f, MOTOR1_INIT_POS);    // Motor 1: ID=2, CAN1
	Init_J60_Motor(&j60_motor[2], 3, 1, MOTOR2_INIT_POS, -1, MOTOR2_INIT_POS, 0.785f);  // Motor 2: ID=3, CAN1
	Init_J60_Motor(&j60_motor[3], 4, 1, MOTOR3_INIT_POS, -1, MOTOR3_INIT_POS, 0.32f);  // Motor 3: ID=4, CAN1
	Init_J60_Motor(&j60_motor[4], 5, 1, MOTOR4_INIT_POS, -1, MOTOR4_INIT_POS, 1.1f);  // Motor 4: ID=5, CAN1
	Init_J60_Motor(&j60_motor[5], 6, 1, MOTOR5_INIT_POS, -1, -0.785f, MOTOR5_INIT_POS);  // Motor 5: ID=6, CAN1
	
	// Initialize current positions to motor initial positions
	current_positions[0] = MOTOR0_INIT_POS;
	current_positions[1] = MOTOR1_INIT_POS;
	current_positions[2] = MOTOR2_INIT_POS;
	current_positions[3] = MOTOR3_INIT_POS;
	current_positions[4] = MOTOR4_INIT_POS;
	current_positions[5] = MOTOR5_INIT_POS;
	
	Enable_J60_Motor(&j60_motor[0]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[1]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[2]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[3]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[4]);
	osDelay(20);
	Enable_J60_Motor(&j60_motor[5]);
	osDelay(20);
//	pc_mcu_rx_data[0] = 0.2164f;
//	pc_mcu_rx_data[1] = 0.83f;
//	pc_mcu_rx_data[2] = -0.785f;
//	pc_mcu_rx_data[3] = {0.2164, 0.83, -0.785, 0, 0, 0, 0};
//	pc_mcu_rx_data[4] = {0.2164, 0.83, -0.785, 0, 0, 0, 0};
//	pc_mcu_rx_data[5] = {0.2164, 0.83, -0.785, 0, 0, 0, 0};
    
    // Play startup melody for 2 seconds before starting motor control
    Buzzer_PlayStartupMelody();

    for (;;) {
        // Check if remote is online and right switch is valid (not 1)
        if (dji_remote.online == 0 || dji_remote.right_switch == 1) {
            // Remote is offline or right switch is in position 1 (invalid)
            // Reset to initialization phase
            robot_state = INIT_PHASE;
            state_timer = 0;
            
            // Reset current positions to actual motor positions
            int start_motor, end_motor;
            if (MOTOR_CONFIG == USE_MOTORS_123) {
                start_motor = 0;
                end_motor = 3;
            } else if (MOTOR_CONFIG == USE_MOTORS_456) {
                start_motor = 3;
                end_motor = 6;
            } else { // USE_ALL_MOTORS
                start_motor = 0;
                end_motor = 6;
            }
            
            for (int i = start_motor; i < end_motor; i++) {
                current_positions[i] = j60_motor[i].para.pos;
            }
            
            // Clear commands for active motors only
            start_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 0 : 3;
            end_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 3 : 6;
            for (int i = start_motor; i < end_motor; i++) {
                J60_Cmd_Clear(&j60_motor[i]);
            }
            
            // Reset pc_mcu_rx_data to initial values
            // pc_mcu_rx_data[0] = MOTOR0_INIT_POS;
            // pc_mcu_rx_data[1] = MOTOR1_INIT_POS;
            // pc_mcu_rx_data[2] = MOTOR2_INIT_POS;
            // pc_mcu_rx_data[3] = MOTOR3_INIT_POS;
            // pc_mcu_rx_data[4] = MOTOR4_INIT_POS;
            // pc_mcu_rx_data[5] = MOTOR5_INIT_POS;
            
            // Send commands for active motors only
            if (MOTOR_CONFIG == USE_MOTORS_123) {
                Send_J60_Motor_Command(&j60_motor[0]);
                Send_J60_Motor_Command(&j60_motor[1]);
                osDelay(1);
                Send_J60_Motor_Command(&j60_motor[2]);
            } else if (MOTOR_CONFIG == USE_MOTORS_456) {
                Send_J60_Motor_Command(&j60_motor[3]);
                Send_J60_Motor_Command(&j60_motor[4]);
                osDelay(1);
                Send_J60_Motor_Command(&j60_motor[5]);
            } else { // USE_ALL_MOTORS
                Send_J60_Motor_Command(&j60_motor[0]);
                Send_J60_Motor_Command(&j60_motor[1]);
                osDelay(1);
                Send_J60_Motor_Command(&j60_motor[2]);
                Send_J60_Motor_Command(&j60_motor[3]);
                osDelay(1);
                Send_J60_Motor_Command(&j60_motor[4]);
                Send_J60_Motor_Command(&j60_motor[5]);
            }
            osDelay(10);
            continue;
        }
        
        // State machine for robot initialization and operation
        switch (robot_state) {
            case INIT_PHASE:  // Case 0: Check if all motors are enabled
                if (Check_All_Motors_Enabled() && (dji_remote.right_switch == 2 || dji_remote.right_switch == 3)) {
                    // Reset current positions to actual motor positions before entering positioning phase
                    int start_motor, end_motor;
                    if (MOTOR_CONFIG == USE_MOTORS_123) {
                        start_motor = 0;
                        end_motor = 3;
                    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
                        start_motor = 3;
                        end_motor = 6;
                    } else { // USE_ALL_MOTORS
                        start_motor = 0;
                        end_motor = 6;
                    }
                    
                    // Reset current positions to actual motor positions
                    for (int i = start_motor; i < end_motor; i++) {
                        current_positions[i] = j60_motor[i].para.pos;
                    }
                    
                    robot_state = POSITIONING_PHASE;
                    state_timer = 0;
                } else {
                    // Wait for all motors to be enabled and right switch to be 2 or 3
                    state_timer++;
                    if (state_timer > 500) {  // 5 seconds timeout
                        robot_state = ERROR_PHASE;
                    }
                }
                break;
                
            case POSITIONING_PHASE:  // Case 1: Move motors to desired position and check feedback
                Set_Motors_To_Desired_Position();
                state_timer++;
                
                // Allow 3 seconds for positioning
                if (state_timer > 300) {
                    if (Check_Motors_At_Position()) {
                    	if(dji_remote.right_switch == 2){
                    		robot_state = NORMAL_OPERATION;
                    		state_timer = 0;
                    	}

                    } else {
                        robot_state = ERROR_PHASE;  // Go to error if not at position or right switch not 2
                    }
                }
                break;
                
            case NORMAL_OPERATION:  // Case 2: Normal operation
                // Check if any active motor is offline or enable failed
                {
                    uint8_t motor_error = 0;
                    int start_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 0 : 3;
                    int end_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 3 : 6;
                    
                    for (int i = start_motor; i < end_motor; i++) {
                        if (j60_motor[i].para.enable_failed == 1 || j60_motor[i].para.online == 0) {
                            motor_error = 1;
                            break;
                        }
                    }
                    
                    if (motor_error) {
                        // Clear commands for active motors
                        for (int i = start_motor; i < end_motor; i++) {
                            J60_Cmd_Clear(&j60_motor[i]);
                        }
                    
                                             // Reset pc_mcu_rx_data to initial values
                         // pc_mcu_rx_data[0] = MOTOR0_INIT_POS;
                         // pc_mcu_rx_data[1] = MOTOR1_INIT_POS;
                         // pc_mcu_rx_data[2] = MOTOR2_INIT_POS;
                         // pc_mcu_rx_data[3] = MOTOR3_INIT_POS;
                         // pc_mcu_rx_data[4] = MOTOR4_INIT_POS;
                         // pc_mcu_rx_data[5] = MOTOR5_INIT_POS;
                         
                         osDelay(10);
                         continue;
                     }
                 }
                 
                 // Safety check for active motors: Stop motor if torque exceeds 8.0 N·m
                 {
                     uint8_t safety_error = 0;
                     int start_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 0 : 3;
                     int end_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 3 : 6;
                     
                     for (int i = start_motor; i < end_motor; i++) {
                         if (J60_Safety_Check(&j60_motor[i], 8.0f) == 1) {
                             safety_error = 1;
                             break;
                         }
                     }
                     
                     if (safety_error) {
                         // One or more motors in safety stop - alert
                         Buzzer_Alert();
                         osDelay(10);
                         continue;
                     }
                 }
                
                                 // Generate sine wave: amplitude = 1 rad, period = 2π seconds (frequency ≈ 0.16 Hz)
                 // target_position = (3.142f/2.0f) * sinf(time_counter);
                 
                 // Set motor commands for active motors only
                 {
                     int start_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 0 : 3;
                     int end_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 3 : 6;
                     
                     for (int i = start_motor; i < end_motor; i++) {
                         j60_motor[i].cmd.pos_set = pc_mcu_rx_data[i];  // Position from PC
                         j60_motor[i].cmd.vel_set = 0.0f;    // No velocity
                         j60_motor[i].cmd.kp_set = 45.0f;   // Position gain
                         j60_motor[i].cmd.kd_set = 4.5f;     // Damping gain
                         j60_motor[i].cmd.tor_set = 0.0f;    // No torque
                     }
                 }
                 break;
                
            case ERROR_PHASE:  // Case 99: Error handling
                // Clear commands for active motors
                {
                    int start_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 0 : 3;
                    int end_motor = (MOTOR_CONFIG == USE_MOTORS_123) ? 3 : 6;
                    for (int i = start_motor; i < end_motor; i++) {
                        J60_Cmd_Clear(&j60_motor[i]);
                    }
                }
                
                // Start beeping
                Buzzer_Alert();
                
                // Stay in error state until remote is cycled
                break;
        }
        
        // Send commands to active motors only
        if (MOTOR_CONFIG == USE_MOTORS_123) {
            Send_J60_Motor_Command(&j60_motor[0]);
            Send_J60_Motor_Command(&j60_motor[1]);
            osDelay(1);
            Send_J60_Motor_Command(&j60_motor[2]);
        } else if (MOTOR_CONFIG == USE_MOTORS_456) {
            Send_J60_Motor_Command(&j60_motor[3]);
            Send_J60_Motor_Command(&j60_motor[4]);
            osDelay(1);
            Send_J60_Motor_Command(&j60_motor[5]);
        } else { // USE_ALL_MOTORS
            Send_J60_Motor_Command(&j60_motor[0]);
            Send_J60_Motor_Command(&j60_motor[1]);
            osDelay(1);
            Send_J60_Motor_Command(&j60_motor[2]);
            Send_J60_Motor_Command(&j60_motor[3]);
            osDelay(1);
            Send_J60_Motor_Command(&j60_motor[4]);
            Send_J60_Motor_Command(&j60_motor[5]);
        }
        
        // Increment time (10ms per loop = 0.01 seconds)
        time_counter += 0.01f;
        J60_10_MOTOR_ONLINE_CHECK();
        // DJI_NDJ_REMOTE_CHECK_ONLINE(); // Check remote online status
        osDelay(10);  // Wait 10ms for reasonable CAN bus timing
    }
}

void J60_10_MOTOR_ONLINE_CHECK(void){
    int start_motor, end_motor;
    
    if (MOTOR_CONFIG == USE_MOTORS_123) {
        start_motor = 0;
        end_motor = 3;
    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
        start_motor = 3;
        end_motor = 6;
    } else { // USE_ALL_MOTORS
        start_motor = 0;
        end_motor = 6;
    }
    
    // Check online status for active motors only
    for (int i = start_motor; i < end_motor; i++) {
        if (j60_motor[i].para.ping > 5){
            j60_motor[i].para.online = 0;
        }else{
            j60_motor[i].para.online = 1;
        }
    }
    
    // Set inactive motors as offline to avoid interference (only if not using all motors)
    if (MOTOR_CONFIG == USE_MOTORS_123) {
        // Motors 3,4,5 are inactive
        j60_motor[3].para.online = 0;
        j60_motor[4].para.online = 0;
        j60_motor[5].para.online = 0;
    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
        // Motors 0,1,2 are inactive
        j60_motor[0].para.online = 0;
        j60_motor[1].para.online = 0;
        j60_motor[2].para.online = 0;
    }
    // If USE_ALL_MOTORS, no motors need to be set offline
}

uint8_t Check_All_Motors_Enabled(void) {
    int start_motor, end_motor;
    
    if (MOTOR_CONFIG == USE_MOTORS_123) {
        start_motor = 0;
        end_motor = 3;
    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
        start_motor = 3;
        end_motor = 6;
    } else { // USE_ALL_MOTORS
        start_motor = 0;
        end_motor = 6;
    }
    
    for (int i = start_motor; i < end_motor; i++) {
        if (j60_motor[i].para.enable_failed == 1 || j60_motor[i].para.online == 0) {
            return 0; // Not all motors enabled or online
        }
    }
    return 1; // All motors enabled and online
}

uint8_t Check_Motors_At_Position(void) {
    float desired_positions[6] = {MOTOR0_DESIRED_POS, MOTOR1_DESIRED_POS, MOTOR2_DESIRED_POS, 
                                  MOTOR3_DESIRED_POS, MOTOR4_DESIRED_POS, MOTOR5_DESIRED_POS};
    
    int start_motor, end_motor;
    
    if (MOTOR_CONFIG == USE_MOTORS_123) {
        start_motor = 0;
        end_motor = 3;
    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
        start_motor = 3;
        end_motor = 6;
    } else { // USE_ALL_MOTORS
        start_motor = 0;
        end_motor = 6;
    }
    
    // First check if all current_positions have reached desired positions
    for (int i = start_motor; i < end_motor; i++) {
        if (fabs(current_positions[i] - desired_positions[i]) > 0.001f) {
            return 0; // Not all target positions have been reached
        }
    }
    
    // Then check if the actual motor positions match the target positions
    for (int i = start_motor; i < end_motor; i++) {
        // Check if the motor's current position is within the tolerance of its desired position
        if (fabs(j60_motor[i].para.pos - desired_positions[i]) > POSITION_TOLERANCE) {
            return 0; // Not all motors at desired position
        }
    }
    return 1; // All motors at desired position
}

void Set_Motors_To_Desired_Position(void) {
    float desired_positions[6] = {MOTOR0_DESIRED_POS, MOTOR1_DESIRED_POS, MOTOR2_DESIRED_POS, 
                                  MOTOR3_DESIRED_POS, MOTOR4_DESIRED_POS, MOTOR5_DESIRED_POS};
    
    int start_motor, end_motor;
    
    if (MOTOR_CONFIG == USE_MOTORS_123) {
        start_motor = 0;
        end_motor = 3;
    } else if (MOTOR_CONFIG == USE_MOTORS_456) {
        start_motor = 3;
        end_motor = 6;
    } else { // USE_ALL_MOTORS
        start_motor = 0;
        end_motor = 6;
    }
    
    // Gradually move each motor's position toward the desired position
    for (int i = start_motor; i < end_motor; i++) {
        // Calculate position difference
        float pos_diff = desired_positions[i] - current_positions[i];
        
        // Increment position by a small amount toward the desired position
        if (fabs(pos_diff) < POSITION_INCREMENT) {
            // If close enough, just set to the desired position
            current_positions[i] = desired_positions[i];
        } else if (pos_diff > 0) {
            // Move toward positive direction
            current_positions[i] += POSITION_INCREMENT;
        } else {
            // Move toward negative direction
            current_positions[i] -= POSITION_INCREMENT;
        }
        
        // Set the incremented position
        j60_motor[i].cmd.pos_set = current_positions[i];
        j60_motor[i].cmd.vel_set = 0.0f;
        j60_motor[i].cmd.kp_set = 45.0f;  // Normal gain for accurate positioning
        j60_motor[i].cmd.kd_set = 4.5f;   // Normal damping for stability
        j60_motor[i].cmd.tor_set = 0.0f;
    }
}

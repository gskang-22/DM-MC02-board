/*
 * j60_10.c
 *
 *  Created on: Jul 4, 2025
 *      Author: YI MING
 */
#include "bsp_fdcan.h"
#include "j60_10.h"
#include <math.h>
#include "stm32h7xx_hal.h"  // For HAL_GetTick



void Enable_J60_Motor(motor_t *motor)
{
    uint32_t can_id = (motor->id & 0x1F) | (2 << 5); // Enable command index = 2
    uint8_t dummy_data[1] = {0}; // In case DLC=0 is not allowed

    // Send with DLC = 0 or 1 depending on FDCAN config
    switch(motor->can_number) {
        case 1:
            fdcanx_send_data(&hfdcan1, can_id, dummy_data, 0);
            break;
        case 2:
            fdcanx_send_data(&hfdcan2, can_id, dummy_data, 0);
            break;
        case 3:
            fdcanx_send_data(&hfdcan3, can_id, dummy_data, 0);
            break;
        default:
            // Invalid CAN number, do nothing or handle error
            break;
    }
}

// Initialize a single J60 motor with specified ID and CAN channel
void Init_J60_Motor(motor_t *motor, int16_t motor_id, uint8_t can_channel, float rad_offset, int8_t direction, float pos_limit_min, float pos_limit_max)
{
    // Set motor identification
    motor->id = motor_id;
    motor->can_number = can_channel;
    
    // Initialize all control commands to zero
    motor->cmd.pos_set = 0.0f;
    motor->cmd.vel_set = 0.0f;
    motor->cmd.kp_set = 0.0f;
    motor->cmd.kd_set = 0.0f;
    motor->cmd.tor_set = 0.0f;
    
    // Initialize feedback parameters to zero/safe values
    motor->para.id = 0;
    motor->para.state = 0;
    motor->para.p_int = 0;
    motor->para.v_int = 0;
    motor->para.t_int = 0;
    motor->para.kp_int = 0;
    motor->para.kd_int = 0;
    motor->para.pos = 0.0f;
    motor->para.vel = 0.0f;
    motor->para.tor = 0.0f;
    motor->para.Kp = 0.0f;
    motor->para.Kd = 0.0f;
    motor->para.Tmos = 0.0f;
    motor->para.Tcoil = 0.0f;
    motor->para.temperature = 0.0f;
    motor->para.torque = 0.0f;
    motor->para.speed = 0.0f;
    motor->para.encoder_angle = 0.0f;
    motor->para.previous_angle = 0.0f;
    motor->para.rotations = 0;
    motor->para.heartbeat = 0;
    motor->para.ping = 0;
    motor->para.online = 1;
    motor->para.safety_stop = 0; // Initialize safety stop flag

    // Set direction and offset
    motor->rad_offset = rad_offset;
    motor->direction = direction; // Set motor direction (1 = forward, -1 = reverse)
    motor->rad_offset = rad_offset * direction;
    
    // Set position limits
    motor->pos_limit_min = pos_limit_min;
    motor->pos_limit_max = pos_limit_max;
    motor->limit_enabled = 1; // Enable position limits by default
}

// Send control command to a single J60 motor
void Send_J60_Motor_Command(motor_t *motor)
{
    // Apply position limits first
    float limited_position = Apply_Position_Limits(motor, motor->cmd.pos_set);
    
    // Apply direction to motor commands
    float pos_set_with_direction = limited_position * motor->direction;
    float vel_set_with_direction = motor->cmd.vel_set * motor->direction;
    float tor_set_with_direction = motor->cmd.tor_set * motor->direction;

	float pos_set_after_offset = pos_set_with_direction - motor->rad_offset;
    uint16_t pos_u16 = MAP_F32_TO_U16(pos_set_after_offset, -40.0f, 40.0f);
    uint16_t vel_u14 = MAP_F32_TO_U14(vel_set_with_direction, -40.0f, 40.0f);
    uint16_t kp_u10  = MAP_F32_TO_U10(motor->cmd.kp_set, 0.0f, 1023.0f);
    uint8_t  kd_u8   = MAP_F32_TO_U8(motor->cmd.kd_set, 0.0f, 51.0f);
    uint16_t torque_u16 = MAP_F32_TO_U16(tor_set_with_direction, -40.0f, 40.0f);

    uint8_t cmd_data[8] = {0};

    // Pack bits according to official J60 protocol:
    // Bit0~Bit15: Target angle (16 bits)
    // Bit16~Bit29: Target angular velocity (14 bits)
    // Bit30~Bit39: Kp stiffness (10 bits)
    // Bit40~Bit47: Kd damping (8 bits) 
    // Bit48~Bit63: Target torque (16 bits)
    
    // Bit0~Bit15: Target angle (bytes 0-1)
    cmd_data[0] = pos_u16 & 0xFF;           // Lower 8 bits
    cmd_data[1] = (pos_u16 >> 8) & 0xFF;    // Upper 8 bits
    
    // Bit16~Bit29: Target velocity (14 bits spanning bytes 2-3)
    cmd_data[2] = vel_u14 & 0xFF;           // Lower 8 bits of velocity (bits 16-23)
    cmd_data[3] = (vel_u14 >> 8) & 0x3F;    // Upper 6 bits of velocity (bits 24-29)
    
    // Bit30~Bit39: Kp (10 bits, 2 bits in byte 3, 8 bits in byte 4)
    cmd_data[3] |= ((kp_u10 & 0x03) << 6);  // Lower 2 bits of Kp (bits 30-31)
    cmd_data[4] = (kp_u10 >> 2) & 0xFF;     // Upper 8 bits of Kp (bits 32-39)
    
    // Bit40~Bit47: Kd (8 bits in byte 5)
    cmd_data[5] = kd_u8 & 0xFF;
    
    // Bit48~Bit63: Target torque (16 bits in bytes 6-7)
    cmd_data[6] = torque_u16 & 0xFF;        // Lower 8 bits
    cmd_data[7] = (torque_u16 >> 8) & 0xFF; // Upper 8 bits

    //ping+1 mean send msg and receive function should set it back to 0
    motor->para.ping += 1;

    // Create CAN ID: motor_id + command index 4 (control command)
    uint32_t motor_ctrl_id = (motor->id & 0x1F) | (4 << 5);

    // Send via appropriate CAN channel
    switch(motor->can_number) {
        case 1:
            fdcanx_send_data(&hfdcan1, motor_ctrl_id, cmd_data, 8);
            break;
        case 2:
            fdcanx_send_data(&hfdcan2, motor_ctrl_id, cmd_data, 8);
            break;
        case 3:
            fdcanx_send_data(&hfdcan3, motor_ctrl_id, cmd_data, 8);
            break;
        default:
            // Invalid CAN number, do nothing or handle error
            break;
    }
}

// Process J60 motor feedback data according to protocol documentation
void J60_Process_Feedback(motor_t *motor, uint8_t *rx_data)
{
    // Get current time in ms (using HAL_GetTick)
    uint32_t current_time = HAL_GetTick();
    
    // Update frequency tracking
    if (motor->para.update_count == 0) {
        // First update - initialize
        motor->para.last_update_time = current_time;
        motor->para.update_interval_sum = 0;
        motor->para.update_frequency = 0.0f;
    } else {
        // Calculate interval since last update
        uint32_t interval = current_time - motor->para.last_update_time;
        
        // Update tracking variables
        motor->para.update_interval_sum += interval;
        
        // Calculate frequency every 100 updates
        if (motor->para.update_count % 100 == 0 && motor->para.update_interval_sum > 0) {
            // Calculate frequency in Hz (updates per second)
            motor->para.update_frequency = 100000.0f / (float)motor->para.update_interval_sum;
            
            // Reset sum for next calculation
            motor->para.update_interval_sum = 0;
        }
        
        // Update last update time
        motor->para.last_update_time = current_time;
    }
    
    // Increment update counter
    motor->para.update_count++;

    // Extract raw values from 8-byte CAN message according to official J60 feedback format:
    // Bit0~Bit19: 当前角度 (20 bits)
    // Bit20~Bit39: 当前角速度 (20 bits) 
    // Bit40~Bit55: 当前扭矩 (16 bits)
    // Bit56: 温度标志位 (1 bit)
    // Bit57~Bit63: 当前温度 (7 bits)
    
    // Position: 20-bit value (bits 0-19) - little-endian extraction
    uint32_t pos_raw = (uint32_t)rx_data[0] |           // bits 0-7
                       ((uint32_t)rx_data[1] << 8) |    // bits 8-15  
                       (((uint32_t)rx_data[2] & 0x0F) << 16); // bits 16-19 (lower 4 bits of byte 2)
    
    // Velocity: 20-bit value (bits 20-39) - spans bytes 2,3,4
    uint32_t vel_raw = (((uint32_t)rx_data[2] & 0xF0) >> 4) |  // bits 20-23 (upper 4 bits of byte 2)
                       ((uint32_t)rx_data[3] << 4) |           // bits 24-31
                       ((uint32_t)rx_data[4] << 12);           // bits 32-39
    
    // Torque: 16-bit value (bits 40-55) - little-endian extraction
    uint16_t tor_raw = (uint16_t)rx_data[5] |           // bits 40-47
                       ((uint16_t)rx_data[6] << 8);     // bits 48-55
    
    // Temperature flag: bit 56 (bit 0 of byte 7)
    uint8_t temp_flag = rx_data[7] & 0x01;
    
    // Temperature: bits 57-63 (bits 1-7 of byte 7)
    uint8_t temp_raw = (rx_data[7] >> 1) & 0x7F;
    
    // Store raw integer values
    motor->para.p_int = pos_raw;
    motor->para.v_int = vel_raw;
    motor->para.t_int = tor_raw;
    
    // Convert to float values using J60 ranges
    float pos_before_offset = MAP_U20_TO_F32(pos_raw, J60_POS_MIN, J60_POS_MAX);
    float vel_raw_float = MAP_U20_TO_F32(vel_raw, J60_VEL_MIN, J60_VEL_MAX);
    float tor_raw_float = MAP_U16_TO_F32(tor_raw, J60_TOR_MIN, J60_TOR_MAX);
    
    // Apply direction to feedback data
    motor->para.pos = (pos_before_offset + motor->rad_offset) * motor->direction;  // Apply direction to position
    motor->para.vel = vel_raw_float * motor->direction;  // Apply direction to velocity  
    motor->para.tor = tor_raw_float * motor->direction;  // Apply direction to torque
    
    // Process temperature
    motor->para.temperature = MAP_U7_TO_F32(temp_raw, J60_TEMP_MIN, J60_TEMP_MAX);  // [-20, +200] °C
    
    // Set temperature type based on flag
    if (temp_flag == 0) {
        motor->para.Tmos = motor->para.temperature;   // MOSFET temperature
        motor->para.Tcoil = 0.0f;                     // Motor temperature not available
    } else {
        motor->para.Tmos = 0.0f;                      // MOSFET temperature not available
        motor->para.Tcoil = motor->para.temperature;  // Motor temperature
    }
    
    motor->para.ping = 0;
}

void J60_Enable_Feedback(motor_t *motor, uint8_t *rx_data)
{
    motor->para.enable_failed = rx_data[0];
}

void J60_Cmd_Clear(motor_t *motor){
	motor->cmd.pos_set = 0.0f;
	motor->cmd.vel_set = 0.0f;
	motor->cmd.kp_set = 0.0f;
	motor->cmd.kd_set = 0.0f;
	motor->cmd.tor_set = 0.0f;
}

/**
 * @brief Apply position limits to the motor command
 * @param motor Pointer to motor structure
 * @param position Desired position in radians
 * @return Limited position value in radians
 */
float Apply_Position_Limits(motor_t *motor, float position)
{
    // If limits are not enabled, return the original position
    if (!motor->limit_enabled) {
        return position;
    }
    
    // Apply limits
    if (position > motor->pos_limit_max) {
        return motor->pos_limit_max;
    } else if (position < motor->pos_limit_min) {
        return motor->pos_limit_min;
    }
    
    // Position is within limits, return unchanged
    return position;
}

/**
 * @brief Safety check function for motor torque
 * @param motor Pointer to motor structure
 * @param torque_limit Maximum allowed torque (absolute value)
 * @return 1 if motor is in safety stop, 0 if motor is safe to operate
 */
uint8_t J60_Safety_Check(motor_t *motor, float torque_limit)
{
    // Check if absolute torque exceeds limit
    if (fabsf(motor->para.tor) > torque_limit) {
        // Set safety stop flag
        motor->para.safety_stop = 1;
        
        // Clear all motor commands for safety
        J60_Cmd_Clear(motor);
        
        return 1; // Motor is in safety stop
    }
    
    // Motor is safe to operate
    return 0;
}

/**
 * @brief Reset safety stop flag
 * @param motor Pointer to motor structure
 */
void J60_Safety_Reset(motor_t *motor)
{
    motor->para.safety_stop = 0;
}

/**
 * @brief Set position limits for a motor
 * @param motor Pointer to motor structure
 * @param min_pos Minimum position limit in radians
 * @param max_pos Maximum position limit in radians
 */
void J60_Set_Position_Limits(motor_t *motor, float min_pos, float max_pos)
{
    motor->pos_limit_min = min_pos;
    motor->pos_limit_max = max_pos;
}

/**
 * @brief Enable or disable position limits for a motor
 * @param motor Pointer to motor structure
 * @param enable 1 to enable limits, 0 to disable limits
 */
void J60_Enable_Position_Limits(motor_t *motor, uint8_t enable)
{
    motor->limit_enabled = enable;
}

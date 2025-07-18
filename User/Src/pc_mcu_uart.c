/*
 * pc_mcu_uart.c
 *
 *  Created on: Jun 7, 2025
 *      Author: YI MING
 */
#include "pc_mcu_uart.h"
#include "usart.h"
#include "cmsis_os.h"
#include "string.h"
#include "imu_task.h"
#include "j60_10motor_task.h"

// *** From MCU's perspective ***
// pc_mcu_tx_data = Data that MCU TRANSMITS to PC (MCU → PC)
float pc_mcu_tx_data[MCU_TO_PC_FLOATS] = {0};  // MCU sends 25 floats to PC

// pc_mcu_rx_data = Data that MCU RECEIVES from PC (PC → MCU)
float pc_mcu_rx_data[PC_TO_MCU_FLOATS] = {0.2164, 0.83, -0.785, 0, 0, 0, 0};  // MCU receives 7 floats from PC

// UART buffers (from MCU's perspective)
uint8_t tx_buffer[MCU_TO_PC_MSG_SIZE];     // MCU transmit buffer: 102 bytes (25 floats + CRC)
uint8_t rx_buffer[PC_TO_MCU_MSG_SIZE];     // MCU receive buffer: 30 bytes (7 floats + CRC)

// CRC Error Handler Variables
static uart_error_handler_t error_handler = {0};

// Error handling configuration
#define MAX_CONSECUTIVE_ERRORS  5      // Max consecutive errors before recovery
#define RECEIVE_TIMEOUT_MS      2000   // 2 second timeout
#define RECOVERY_DELAY_MS       10    // Delay during recovery

// Round-trip time measurement constants
#define RTT_EMA_ALPHA           0.2f   // Exponential moving average alpha (0.0-1.0)
#define PING_SEQUENCE_OFFSET    6      // Offset in pc_mcu_rx/tx_data for ping sequence

void MCU_TO_PC_DATA_ASSIGN(){
	// Add some specific test values to MCU transmission
	pc_mcu_tx_data[0] = imuVelocity[0];
	pc_mcu_tx_data[1] = imuVelocity[1];
	pc_mcu_tx_data[2] = imuVelocity[2];
	pc_mcu_tx_data[3] = gyro[0];
	pc_mcu_tx_data[4] = gyro[1];
	pc_mcu_tx_data[5] = gyro[2];
	pc_mcu_tx_data[6] = imuGravityProjected[0];
    pc_mcu_tx_data[7] = imuGravityProjected[1];
    pc_mcu_tx_data[8] = imuGravityProjected[2];
    pc_mcu_tx_data[9] = j60_motor[0].para.pos;
    pc_mcu_tx_data[10] = j60_motor[1].para.pos;
    pc_mcu_tx_data[11] = j60_motor[2].para.pos;
    pc_mcu_tx_data[12] = j60_motor[3].para.pos;
    pc_mcu_tx_data[13] = j60_motor[4].para.pos;
    pc_mcu_tx_data[14] = j60_motor[5].para.pos;
    pc_mcu_tx_data[15] = j60_motor[0].para.vel;
    pc_mcu_tx_data[16] = j60_motor[1].para.vel;
    pc_mcu_tx_data[17] = j60_motor[2].para.vel;
    pc_mcu_tx_data[18] = j60_motor[3].para.vel;
    pc_mcu_tx_data[19] = j60_motor[4].para.vel;
    pc_mcu_tx_data[20] = j60_motor[5].para.vel;
    
    // Send ping sequence number back to PC for round-trip time measurement
    pc_mcu_tx_data[21] = pc_mcu_rx_data[PING_SEQUENCE_OFFSET];
    
    // Add UART connection status and error statistics for debugging
    pc_mcu_tx_data[22] = (float)PC_MCU_UART_Is_Connected();                    // 1.0 = connected, 0.0 = disconnected
    pc_mcu_tx_data[23] = (float)PC_MCU_UART_Get_Time_Since_Last_Receive();     // Time since last receive (ms)
    pc_mcu_tx_data[24] = (float)error_handler.round_trip_time;                 // Current round-trip time (ms)
}

uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        crc &= 0xFFFF;
    }
    return crc;
}

void UART_Error_Recovery(void) {
    // Perform UART error recovery
    error_handler.recovery_count++;
    error_handler.consecutive_errors = 0;
    error_handler.error_state = 1;
    
    // 1. Abort any ongoing UART operations
//    HAL_UART_Abort(&huart10);
    
    // 2. Clear UART error flags
//    __HAL_UART_CLEAR_FLAG(&huart10, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
    
    // 3. Clear receive buffer
    memset(rx_buffer, 0, PC_TO_MCU_MSG_SIZE);
    
    // 4. Small delay to let line stabilize
    osDelay(RECOVERY_DELAY_MS);
    
    // 5. Restart UART receive
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);
    
    if (status == HAL_OK) {
        error_handler.error_state = 0;  // Recovery successful
    }
}

void Check_Receive_Timeout(void) {
    uint32_t current_time = HAL_GetTick();
    
    // Check if too much time has passed since last successful receive
    if (error_handler.last_receive_time > 0 && 
        (current_time - error_handler.last_receive_time) > RECEIVE_TIMEOUT_MS) {
        
        // Timeout occurred - perform recovery
        UART_Error_Recovery();
//        error_handler.last_receive_time = current_time;  // Reset timeout
    }
}

void PC_MCU_UART_Process_Received_Data(void) {
    // Process data that MCU RECEIVED from PC (7 floats)
    uint16_t received_crc = (uint16_t)rx_buffer[PC_TO_MCU_FLOATS * 4] | 
                           ((uint16_t)rx_buffer[PC_TO_MCU_FLOATS * 4 + 1] << 8);
    uint16_t calculated_crc = crc16_ccitt(rx_buffer, PC_TO_MCU_FLOATS * 4);
    
    error_handler.total_received++;
    
    if (received_crc == calculated_crc) {
        // *** CRC SUCCESS ***
        // Record timestamp when reception completed
        error_handler.rx_timestamp = HAL_GetTick();
        
        // Copy received floats to pc_mcu_rx_data[] (MCU received data)
        memcpy(pc_mcu_rx_data, rx_buffer, PC_TO_MCU_FLOATS * 4);
        
        // Calculate round-trip time if this is a response to our ping
        if (pc_mcu_rx_data[PING_SEQUENCE_OFFSET] != 0) {
            // Calculate round-trip time
            error_handler.round_trip_time = error_handler.rx_timestamp - error_handler.tx_timestamp;
            
            // Update min/max round-trip times
            if (error_handler.min_round_trip_time == 0 || 
                error_handler.round_trip_time < error_handler.min_round_trip_time) {
                error_handler.min_round_trip_time = error_handler.round_trip_time;
            }
            
            if (error_handler.round_trip_time > error_handler.max_round_trip_time) {
                error_handler.max_round_trip_time = error_handler.round_trip_time;
            }
            
            // Update exponential moving average
            if (error_handler.avg_round_trip_time == 0) {
                error_handler.avg_round_trip_time = error_handler.round_trip_time;
            } else {
                error_handler.avg_round_trip_time = (uint32_t)(
                    (1.0f - RTT_EMA_ALPHA) * error_handler.avg_round_trip_time + 
                    RTT_EMA_ALPHA * error_handler.round_trip_time
                );
            }
        }
        
        // Reset error tracking on successful receive
        error_handler.consecutive_errors = 0;
        error_handler.last_receive_time = HAL_GetTick();
        error_handler.error_state = 0;
        
        // Re-arm UART receive for next message from PC
        HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);
    } 
    else {
        // *** CRC ERROR - Comprehensive Error Handling ***
        error_handler.crc_errors++;
        error_handler.consecutive_errors++;
        
        // Check if too many consecutive errors
        if (error_handler.consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            // Perform comprehensive recovery
            UART_Error_Recovery();
        } 
        else {
            // Simple recovery - clear buffer and restart receive
            memset(rx_buffer, 0, PC_TO_MCU_MSG_SIZE);
            HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);
        }
    }
}

void PC_MCU_UART_TASK(void) {
    // Initialize error handler
    error_handler.last_receive_time = HAL_GetTick();
    error_handler.min_round_trip_time = 0;
    error_handler.max_round_trip_time = 0;
    error_handler.avg_round_trip_time = 0;
    
    // Start first receive (expecting 7 floats from PC)
    HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);

    // Ping sequence number for round-trip time measurement
    static uint32_t ping_sequence = 0;

    for (;;) {
        // Check for receive timeout
        Check_Receive_Timeout();
        
        // Prepare data that MCU will TRANSMIT to PC (25 floats)
        MCU_TO_PC_DATA_ASSIGN();
        
        // Increment ping sequence number for next transmission
        ping_sequence++;
        if (ping_sequence > 65000) ping_sequence = 1; // Avoid overflow and zero
        
        // Record timestamp before transmission
        error_handler.tx_timestamp = HAL_GetTick();
        
        // Store ping sequence in the outgoing data
        pc_mcu_tx_data[PING_SEQUENCE_OFFSET] = (float)ping_sequence;
        
        memcpy(tx_buffer, pc_mcu_tx_data, MCU_TO_PC_FLOATS * 4);
        uint16_t crc = crc16_ccitt(tx_buffer, MCU_TO_PC_FLOATS * 4);
        tx_buffer[MCU_TO_PC_FLOATS * 4] = crc & 0xFF;
        tx_buffer[MCU_TO_PC_FLOATS * 4 + 1] = (crc >> 8) & 0xFF;

        // MCU TRANSMITS message to PC (25 floats)
        error_handler.total_transmissions++;
        HAL_UART_Transmit_IT(&huart10, tx_buffer, MCU_TO_PC_MSG_SIZE);

        // Re-arm UART receive periodically to ensure it's always active
        // Only if not in error state
        if (!error_handler.error_state) {
            HAL_UART_Receive_IT(&huart10, rx_buffer, PC_TO_MCU_MSG_SIZE);
        }

        osDelay(10); // Send every 10ms
    }
}

// *** UART Communication Status Functions (for buzzer alerts) ***

/**
 * @brief Check if UART communication is connected/active
 * @return 1 if connected, 0 if disconnected
 */
uint8_t PC_MCU_UART_Is_Connected(void) {
    uint32_t current_time = HAL_GetTick();
    
    // If we never received anything, consider disconnected
    if (error_handler.last_receive_time == 0) {
        return 0;
    }
    
    // Check if time since last receive exceeds timeout threshold
    if ((current_time - error_handler.last_receive_time) > UART_CONNECTION_TIMEOUT_MS) {
        return 0; // Disconnected
    }
    
    return 1; // Connected
}

/**
 * @brief Get time elapsed since last successful UART receive
 * @return Time in milliseconds since last receive (0 if never received)
 */
uint32_t PC_MCU_UART_Get_Time_Since_Last_Receive(void) {
    uint32_t current_time = HAL_GetTick();
    
    if (error_handler.last_receive_time == 0) {
        return 0xFFFFFFFF; // Never received - return max value
    }
    
    return (current_time - error_handler.last_receive_time);
}

/**
 * @brief Get pointer to UART error statistics
 * @return Pointer to error handler structure
 */
uart_error_handler_t* PC_MCU_UART_Get_Error_Stats(void) {
    return &error_handler;
}


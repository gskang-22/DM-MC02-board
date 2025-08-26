#ifndef BSP_DAMIAO_H
#define BSP_DAMIAO_H

#include "main.h"
#include "board_lib.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include <stdint.h>

// DM4310 Motor Control Modes
#define MIT_MODE     0x00
#define POS_MODE     0x10
#define SPEED_MODE   0x20
#define POSI_MODE    0x30

// Parameter Limits for DM4310
#define P_MIN   -(PI * 4)
#define P_MAX    (PI * 4)
#define V_MIN   -45.0f
#define V_MAX    45.0f
#define T_MIN   -18.0f
#define T_MAX    18.0f
#define KP_MIN    0.0f
#define KP_MAX  500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f
#define P_ROUNDS  4.0f  // Number of rotation rounds

// Type definitions for PID control
typedef struct _PID {
    float kp, ki, kd;
    float error, lastError;
    float integral, maxIntegral;
    float output, maxOutput;
    float deadzone;
    float errLpfRatio;
} PID;

//typedef struct _CascadePID {
//    PID inner;
//    PID outer;
//    float output;
//} CascadePID;

// Motor Command Structure
// to store fixed kp, kd, pos, vel, tor values
typedef struct {
    float kp_set;
    float kd_set;
    float pos_set;
    float vel_set;
    float tor_set;
} dm_motor_cmd_t;

// Motor Control Structure
// values to be sent to the DM4310 motor
typedef struct {
    uint8_t mode;   // 0: MIT Mode, 1: Position-Speed Mode, 2: Speed Mode, 3: Position Force Mode
    float kp_set;
    float kd_set;
    float pos_set;
    float vel_set;
    float tor_set;
} dm_motor_ctrl_t;

// Motor Parameter Structure
typedef struct {
    uint8_t id;         // Motor ID
    uint8_t state;      // Motor state
    uint8_t heartbeat;  // Heartbeat counter
    uint8_t online;     // 1: Online, 0: Offline
    uint16_t p_int;     // Position as integer
    uint16_t v_int;     // Velocity as integer
    uint16_t t_int;     // Torque as integer
    float pos;          // Position in radians
    float vel;          // Velocity in rad/s
    float tor;          // Torque
    float Tmos;         // MOSFET temperature
    float Tcoil;        // Coil temperature
    uint16_t disconnect_time; // used to check if dm motor is initialised
} dm_motor_para_t;

typedef struct {
	float phy_min_ang;
	float phy_max_ang;
	float center_ang;
	float adj_ang;
} dm_angle_data_t;

// Motor Structure
typedef struct {
    uint16_t id;           		// Motor ID for commands
    uint8_t motor_type;			// Store motor type
    dm_motor_cmd_t cmd;       	// Command data
    dm_motor_ctrl_t ctrl;     	// Control data
    dm_motor_para_t para;     	// Parameter data (feedback)
    pid_data_t angle_pid;
    dm_angle_data_t angle_data;
    uint32_t disconnect_time; // Time since last feedback in ms
} dm_motor_t;

// Function prototypes
void dm4310_motor_init(void);
void dm4310_enable(FDCAN_HandleTypeDef* hcan, dm_motor_t* motor);
void dm4310_disable(FDCAN_HandleTypeDef* hcan, dm_motor_t* motor);
void dm4310_ctrl_send(FDCAN_HandleTypeDef* hcan, dm_motor_t* motor);
void dm4310_set(dm_motor_t* motor);
void dm4310_clear_para(dm_motor_t* motor);
void dm4310_clear_err(FDCAN_HandleTypeDef* hcan, dm_motor_t* motor);
void dm4310_fbdata(dm_motor_t* motor, uint8_t* rx_data);

// Low-level control functions
void enable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void save_pos_zero(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

// Control mode functions
void mit_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void pos_speed_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel);
void speed_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);
void pos_force_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, uint16_t vel, uint16_t i);
void MFspeed_ctrl(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float vel);
void MFtorque_command(FDCAN_HandleTypeDef* hcan, uint16_t motor_id, float desired_torque);

// Utility functions
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x_float, float x_min, float x_max, int bits);

// Helper functions
float dm_yaw_encoder_mod(float raw_angle);

// Motor data mapping functions
void dmmapyawfbdata(dm_motor_t *yaw_motor);
void dmmappitchfbdata(dm_motor_t *pitch_motor);

#endif /* BSP_DAMIAO_H */ 

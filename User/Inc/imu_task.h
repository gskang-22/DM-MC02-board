#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "cmsis_os.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

// Velocity array offsets
#define INS_VX_ADDRESS_OFFSET     0
#define INS_VY_ADDRESS_OFFSET     1
#define INS_VZ_ADDRESS_OFFSET     2

// Projected gravity array offsets
#define INS_GX_ADDRESS_OFFSET     0
#define INS_GY_ADDRESS_OFFSET     1
#define INS_GZ_ADDRESS_OFFSET     2

// Gravity constant (m/s²)
#define GRAVITY_CONSTANT         9.81f

// Sample time for integration (1ms = 0.001s)
#define SAMPLE_TIME              0.001f

// External variables
extern float imuAngle[3];
extern float imuVelocity[3];
extern float imuGravityProjected[3];

// Function declarations
void TransformAccelToWorldFrame(float quat[4], float accel_body[3], float accel_world[3]);
void CalculateVelocity(float accel_world[3], float velocity[3]);
void CalculateProjectedGravity(float quat[4], float gravity_projected[3]);

#endif

#include "imu_task.h"
#include "bmi088driver.h"
#include "MahonyAHRS.h"
//#include "tim.h"
//#include "vofa.h"
#include <math.h>

#define DES_TEMP    40.0f
#define KP          100.f
#define KI          50.f
#define KD          10.f
#define MAX_OUT     500

float gyro[3] = {0.0f};
float acc[3] = {0.0f};
static float temp = 0.0f;

float imuQuat[4] = {0.0f};
float imuAngle[3] = {0.0f};
float imuVelocity[3] = {0.0f};  // Velocity in world frame (x, y, z)
float imuGravityProjected[3] = {0.0f};  // Projected gravity in world frame (-1 to 1)

float out = 0;
float err = 0;
float err_l = 0;
float err_ll = 0;

// Variables for velocity calculation
static float accel_world[3] = {0.0f};  // Acceleration in world frame
static float accel_world_filtered[3] = {0.0f};  // Low-pass filtered acceleration
static const float filter_alpha = 0.1f;  // Low-pass filter coefficient

void AHRS_init(float quat[4])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

void AHRS_update(float quat[4], float gyro[3], float accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

void GetAngle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}

/**
 * @brief Transform acceleration from body frame to world frame using quaternion
 * @param quat: Quaternion [w, x, y, z]
 * @param accel_body: Acceleration in body frame [x, y, z]
 * @param accel_world: Output acceleration in world frame [x, y, z]
 */
void TransformAccelToWorldFrame(float quat[4], float accel_body[3], float accel_world[3])
{
    float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    
    // Rotation matrix elements (quaternion to rotation matrix)
    float r11 = 1 - 2*(q2*q2 + q3*q3);
    float r12 = 2*(q1*q2 - q0*q3);
    float r13 = 2*(q1*q3 + q0*q2);
    
    float r21 = 2*(q1*q2 + q0*q3);
    float r22 = 1 - 2*(q1*q1 + q3*q3);
    float r23 = 2*(q2*q3 - q0*q1);
    
    float r31 = 2*(q1*q3 - q0*q2);
    float r32 = 2*(q2*q3 + q0*q1);
    float r33 = 1 - 2*(q1*q1 + q2*q2);
    
    // Transform acceleration from body frame to world frame
    accel_world[0] = r11*accel_body[0] + r12*accel_body[1] + r13*accel_body[2];
    accel_world[1] = r21*accel_body[0] + r22*accel_body[1] + r23*accel_body[2];
    accel_world[2] = r31*accel_body[0] + r32*accel_body[1] + r33*accel_body[2];
    
    // Remove gravity (assuming z-axis points up in world frame)
    accel_world[2] -= GRAVITY_CONSTANT;
}

/**
 * @brief Calculate velocity by integrating acceleration
 * @param accel_world: Acceleration in world frame [x, y, z]
 * @param velocity: Output velocity [x, y, z]
 */
void CalculateVelocity(float accel_world[3], float velocity[3])
{
    // Apply low-pass filter to reduce noise
    accel_world_filtered[0] = filter_alpha * accel_world[0] + (1.0f - filter_alpha) * accel_world_filtered[0];
    accel_world_filtered[1] = filter_alpha * accel_world[1] + (1.0f - filter_alpha) * accel_world_filtered[1];
    accel_world_filtered[2] = filter_alpha * accel_world[2] + (1.0f - filter_alpha) * accel_world_filtered[2];
    
    // Integrate acceleration to get velocity (v = v0 + a*dt)
    velocity[0] += accel_world_filtered[0] * SAMPLE_TIME;
    velocity[1] += accel_world_filtered[1] * SAMPLE_TIME;
    velocity[2] += accel_world_filtered[2] * SAMPLE_TIME;
    
    // Apply velocity decay to reduce drift (optional, adjust factor as needed)
    const float velocity_decay = 0.999f;
    velocity[0] *= velocity_decay;
    velocity[1] *= velocity_decay;
    velocity[2] *= velocity_decay;
}

/**
 * @brief Calculate projected gravity components in robot frame, independent of yaw rotation
 * @param quat: Quaternion [w, x, y, z] representing current orientation
 * @param gravity_projected: Output projected gravity [x, y, z] in robot frame
 * @note This gives gravity direction in robot frame, unaffected by yaw rotation
 */
void CalculateProjectedGravity(float quat[4], float gravity_projected[3])
{
    // Extract pitch and roll angles from quaternion, ignoring yaw
    float pitch = asinf(-2.0f*(quat[1]*quat[3] - quat[0]*quat[2]));
    float roll = atan2f(2.0f*(quat[0]*quat[1] + quat[2]*quat[3]), 2.0f*(quat[0]*quat[0] + quat[3]*quat[3]) - 1.0f);
    
    // Calculate gravity vector in robot frame using only pitch and roll
    // When robot is level: gravity = [0, 0, -1]
    // When robot tilts: gravity components change based on tilt angles
    gravity_projected[0] = sinf(pitch);           // X component (forward/backward tilt)
    gravity_projected[1] = -sinf(roll) * cosf(pitch);  // Y component (left/right tilt)  
    gravity_projected[2] = -cosf(pitch) * cosf(roll);  // Z component (up/down)
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
void ImuTask_Entry(void const * argument)
{
    /* USER CODE BEGIN ImuTask_Entry */
    osDelay(10);
//    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init())
    {
        osDelay(100);
    }
    
    AHRS_init(imuQuat);
    
    // Initialize velocity and gravity projection to zero
    imuVelocity[0] = 0.0f;
    imuVelocity[1] = 0.0f;
    imuVelocity[2] = 0.0f;
    
    imuGravityProjected[0] = 0.0f;
    imuGravityProjected[1] = 0.0f;
    imuGravityProjected[2] = -1.0f;  // Start assuming upright position
    
    /* Infinite loop */
    for(;;)
    {
        BMI088_read(gyro, acc, &temp);
        
        AHRS_update(imuQuat, gyro, acc);
        GetAngle(imuQuat, imuAngle + INS_YAW_ADDRESS_OFFSET, imuAngle + INS_PITCH_ADDRESS_OFFSET, imuAngle + INS_ROLL_ADDRESS_OFFSET);
        
        // Calculate velocity from acceleration
        TransformAccelToWorldFrame(imuQuat, acc, accel_world);
        CalculateVelocity(accel_world, imuVelocity);
        
        // Calculate projected gravity components
        CalculateProjectedGravity(imuQuat, imuGravityProjected);
        
        err_ll = err_l;
        err_l = err;
        err = DES_TEMP - temp;
        out = KP*err + KI*(err + err_l + err_ll) + KD*(err - err_l);
        if (out > MAX_OUT) out = MAX_OUT;
        if (out < 0) out = 0.f;
//        htim3.Instance->CCR4 = (uint16_t)out;
        
//        vofa_send_data(0, imuAngle[0]);
//        vofa_send_data(1, imuAngle[1]);
//        vofa_send_data(2, imuAngle[2]);
//        vofa_send_data(3, imuVelocity[0]);
//        vofa_send_data(4, imuVelocity[1]);
//        vofa_send_data(5, imuVelocity[2]);
//        vofa_send_data(6, imuGravityProjected[0]);
//        vofa_send_data(7, imuGravityProjected[1]);
//        vofa_send_data(8, imuGravityProjected[2]);
//        vofa_sendframetail();
        
        osDelay(1);
    }
    /* USER CODE END ImuTask_Entry */
}




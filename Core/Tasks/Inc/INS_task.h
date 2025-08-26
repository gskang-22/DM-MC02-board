/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
//#include "QuaternionEKF.h"

#define imuX 1
#define imuY 0
#define imuZ 2

#define INS_TASK_PERIOD 1
//Referred to https://nitinjsanket.github.io/tutorials/attitudeest/mahony.html


typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    //Gyroscope model: measured angular velocity from gyro = latent ideal angular velocity we wish to recover + b + n
    float Gyro[3];
    float Accel[3]; // ���ٶ�
    float MotionAccel_b[3]; // gyro bias which changes with time and other factors like temparature
    float MotionAccel_n[3]; // white gaussian gyro noise

    float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��

    // ���ٶ��ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // λ��
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		float YawAngleLast;
		float YawRoundCount;
		
		float v_n;//����ϵ����ˮƽ�˶�������ٶ�
		float x_n;//����ϵ����ˮƽ�˶������λ��
		
		uint8_t ins_flag;
} INS_t;


/**
 * @brief ����������װ���Ĳ���,demo�п�����
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

void INS_Init(void);
void INS_task(void *argument);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif



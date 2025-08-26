/**
  *********************************************************************
  * @file      ins_task.c/h
  * @brief     ����������mahony������ȡ������̬��ͬʱ��ȡ�����ھ�������ϵ�µ��˶����ٶ�
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "INS_task.h"
//#include "QuaternionEKF.h"
#include "mahony_filter.h"
#include <math.h>
#include "board_lib.h"
#include "imu_processing_task.h"

INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;
extern float imu_test[6];

void INS_Init(void)
{
	 mahony_init(&mahony,1.0f,0.0f,0.005f);
   INS.AccelLPF = 0.089f;
   mahony.q0 = 0;
   mahony.q1 =0;
   mahony.q2 =0;
   mahony.q3 =0;
   mahony.rMat[0][0] = 0;
   mahony.rMat[0][1] = 0;
   mahony.rMat[0][2] = 0;
   mahony.rMat[1][0] = 0;
   mahony.rMat[1][1] = 0;
   mahony.rMat[1][2] = 0;
   mahony.rMat[2][0] = 0;
   mahony.rMat[2][1] = 0;
   mahony.rMat[2][2] = 0;
}

void INS_task(void *argument)
{
	osDelay(1000);
	 INS_Init();

	 while(1)
	 {
		ins_dt = 0.005f;

		mahony.dt = ins_dt;

    INS.Accel[0] = imu_test[0];
    INS.Accel[1] = imu_test[1];
    INS.Accel[2] = imu_test[2];
	  Accel.x=imu_test[0];
	  Accel.y=imu_test[1];
		Accel.z=imu_test[2];
    INS.Gyro[0] = imu_test[3];
    INS.Gyro[1] = imu_test[4];
    INS.Gyro[2] = -imu_test[5];
  	Gyro.x=imu_test[3];
		Gyro.y=imu_test[4];
		Gyro.z=imu_test[5];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		float norm = sqrtf(mahony.q0 * mahony.q0 + mahony.q1 * mahony.q1 + mahony.q2 * mahony.q2 + mahony.q3 * mahony.q3);
		mahony.q0 /= norm;
		mahony.q1 /= norm;
		mahony.q2 /= norm;
		mahony.q3 /= norm;
	  RotationMatrix_update(&mahony);

		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;

      // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
		float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++) // ͬ����һ����ͨ�˲�
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt)
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt);
//			INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
//														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // ת���ص���ϵn

		//��������
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
		  INS.MotionAccel_n[0]=0.0f;	//x��
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
		  INS.MotionAccel_n[1]=0.0f;	//y��
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
		  INS.MotionAccel_n[2]=0.0f;//z��
		}

		if(ins_time>100.0f)
		{
			INS.ins_flag=1;//��Ԫ���������������ٶ�Ҳ�������������Կ�ʼ��������
			// ��ȡ��������
      INS.Pitch= -mahony.roll;
		  INS.Roll=mahony.pitch;
		  INS.Yaw=mahony.yaw;

		//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
		 ins_time++;
		}

    osDelay(5);
	}
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}





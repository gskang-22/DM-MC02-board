/*
 * gimbal_control_task.h
 *
 *  Created on: Jan 1, 2022
 *      Author: wx
 */

#ifndef TASKS_INC_GIMBAL_CONTROL_TASK_H_
#define TASKS_INC_GIMBAL_CONTROL_TASK_H_

void gimbal_control_task(void *argument);
void gimbal_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);
void gimbal_angle_control(motor_data_t *pitch_motor, motor_data_t *yaw_motor);

uint8_t limit_pitch(float *rel_pitch_angle, motor_data_t *pitch_motor);
void pitch_control(motor_data_t *pitch_motor);
void yaw_control(motor_data_t *yaw_motor);
void calculate_direct_pitch(motor_data_t *pitch_motor);
void calculate_linkage_pitch(motor_data_t *pitch_motor) ;

#endif /* TASKS_INC_GIMBAL_CONTROL_TASK_H_ */

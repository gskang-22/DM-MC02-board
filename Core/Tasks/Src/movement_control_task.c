/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"
#include "bsp_hall.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;

extern remote_cmd_t g_remote_cmd;
extern motor_data_t g_can_motors[24];
extern ref_game_robot_data_t ref_robot_data;
extern uint32_t ref_power_data_txno;
extern speed_shift_t gear_speed;
float g_chassis_yaw = 0;
uint8_t g_gimbal_state = 0;
extern uint8_t hall_state;
extern int g_spinspin_mode;

extern uint8_t charging_state;

uint8_t zero_start = 0;
uint32_t zeroing_start_time = 0;
int16_t current_rpm;

float motor_yaw_mult[4];
float debug5;
float debug6;

extern QueueHandle_t telem_motor_queue;

static float lvl_max_speed;
static float lvl_max_accel;
static float lvl_max_spin;
static float spin_accel = SPIN_ACCELERATION;

float act_forward = 0.0f;
float act_horizontal = 0.0f;
float act_yaw = 0.0f;

extern int supercap_dash;
extern int supercap_enabled;

void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;

	while (1) {

#ifndef CHASSIS_MCU

		// prevents motors from moving if 1 motor disconnects
//		g_can_motors[FR_MOTOR_ID - 1].output = 0;
//		g_can_motors[FL_MOTOR_ID - 1].output = 0;
//		g_can_motors[BL_MOTOR_ID - 1].output = 0;
//		g_can_motors[BR_MOTOR_ID - 1].output = 0;

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
		pdTRUE,
		portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {

#ifdef HALL_ZERO
			if (check_yaw()){ g_gimbal_state = 1; }

			if (g_gimbal_state){
				if (hall_state == HALL_ON){
				yaw_zeroing(g_can_motors + FR_MOTOR_ID - 1,
						g_can_motors + FL_MOTOR_ID - 1,
						g_can_motors + BL_MOTOR_ID - 1,
						g_can_motors + BR_MOTOR_ID - 1);
				} else {
#endif
					chassis_motion_control(g_can_motors + FR_MOTOR_ID - 1,
							g_can_motors + FL_MOTOR_ID - 1,
							g_can_motors + BL_MOTOR_ID - 1,
							g_can_motors + BR_MOTOR_ID - 1);

#ifdef HALL_ZERO
				}
			}
#endif

			} else {
				g_can_motors[FR_MOTOR_ID - 1].output = 0;
				g_can_motors[FL_MOTOR_ID - 1].output = 0;
				g_can_motors[BL_MOTOR_ID - 1].output = 0;
				g_can_motors[BR_MOTOR_ID - 1].output = 0;

			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			g_can_motors[FR_MOTOR_ID - 1].output = 0;
			g_can_motors[FL_MOTOR_ID - 1].output = 0;
			g_can_motors[BL_MOTOR_ID - 1].output = 0;
			g_can_motors[BR_MOTOR_ID - 1].output = 0;
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

}


float filtered_rpm_fr;
float filtered_rpm_fl;
float filtered_rpm_bl;
float filtered_rpm_br;
float vforwardrpm;
float vyaw;
float vhorizontal;

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = g_can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	float yaw_rpm[4] = { 0, };

	// Setting translational and rotational speed and acceleration base on robot level
	level_config(&lvl_max_speed, &lvl_max_accel, &lvl_max_spin);

	// Sets maximum wheel RPM (used to cap output)
	float max_rpm = M3508_MAX_RPM;

	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!
	float speed_limit = lvl_max_speed;
	float spin_limit = lvl_max_spin;

	// Increase speed when spinspin mode is deactivated
	if (g_spinspin_mode == 0) {
		speed_limit += CHASSIS_SPEED_BOOST;
	}

	//Clamp the values between -limit to limit
	float limit_forward = fmaxf(-speed_limit, fminf(chassis_ctrl_data.forward, speed_limit));
	float limit_horizontal = fmaxf(-speed_limit, fminf(chassis_ctrl_data.horizontal, speed_limit));
	float limit_yaw = fmaxf(-spin_limit, fminf(chassis_ctrl_data.yaw, spin_limit));

	// Smooths speed changes over time using acceleration constraints
	act_forward = rpm_ramp(limit_forward * gear_speed.trans_mult, act_forward, &lvl_max_accel);  //gear shifter multipliers
	act_horizontal = rpm_ramp(limit_horizontal * gear_speed.trans_mult, act_horizontal, &lvl_max_accel);
	act_yaw = rpm_ramp(limit_yaw * gear_speed.spin_mult, act_yaw, &spin_accel);

	// translation and rotation speed of chassis for chassis yaw angle relative to gimbal
	float rel_forward = ((-act_horizontal * sin(-rel_angle))
			+ (act_forward * cos(-rel_angle)));
	float rel_horizontal = ((-act_horizontal * cos(-rel_angle))
			+ (act_forward * -sin(-rel_angle)));
	float rel_yaw = act_yaw;

	// calculate theoretical wheel rpm for chassis translation
	translation_rpm[0] = ((rel_forward * FR_VY_MULT)
			+ (rel_horizontal * FR_VX_MULT));
	translation_rpm[1] = ((rel_forward * FL_VY_MULT)
			+ (rel_horizontal * FL_VX_MULT));
	translation_rpm[2] = ((rel_forward * BL_VY_MULT)
			+ (rel_horizontal * BL_VX_MULT));
	translation_rpm[3] = ((rel_forward * BR_VY_MULT)
			+ (rel_horizontal * BR_VX_MULT));

	yaw_rpm[0] = rel_yaw * motor_yaw_mult[0];
	yaw_rpm[1] = rel_yaw * motor_yaw_mult[1];
	yaw_rpm[2] = rel_yaw * motor_yaw_mult[2];
	yaw_rpm[3] = rel_yaw * motor_yaw_mult[3];

	float rpm_mult = 1;
	float rpm_sum = 0;
	for (uint8_t i = 0; i < 4; i++) {
		float temp_add = fabs(yaw_rpm[i] + translation_rpm[i]);
		rpm_sum = rpm_sum + temp_add;  // total combined magnitude of all wheels' RPMs
		if (temp_add > rpm_mult){	   // the maximum RPM among the four wheels
			rpm_mult = temp_add;
		}
	}

	// ensures that individual rpm will not be more than max_rpm
	for (uint8_t j = 0; j < 4; j++) {
		translation_rpm[j] = (translation_rpm[j]			// sum theoretical wheel rpm for translation and yaw
					+ yaw_rpm[j]) * max_rpm / rpm_mult;		// for no spinning modulate wheel rpm by dividing by highest rpm
	}

	// todo: maybe better to change the values of PID here instead of center_yaw()?
	speed_pid(translation_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	speed_pid(translation_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	speed_pid(translation_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	speed_pid(translation_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}

void level_config(float *lvl_max_speed, float *lvl_max_accel, float *lvl_max_spin) {
#ifdef LVL_TUNING
	//	static uint8_t prev_robot_level = -1;

	//	// Hopefully with this, we can adjust pid values without it being overwritten all the time
	//	if (prev_robot_level == ref_robot_data.robot_level) return;
	//	prev_robot_level = ref_robot_data.robot_level;
	uint8_t curr_level = ref_robot_data.robot_level;

	if (supercap_dash && supercap_enabled) {
		curr_level += 10;
	}

	switch (curr_level) {
		case 1:
			*lvl_max_speed = LV1_MAX_SPEED;
			*lvl_max_accel = LV1_MAX_ACCEL;
			*lvl_max_spin  = LV1_CHASSIS_YAW_MAX_RPM;
			break;

		case 2:
			*lvl_max_speed = LV2_MAX_SPEED;
			*lvl_max_accel = LV2_MAX_ACCEL;
			*lvl_max_spin  = LV2_CHASSIS_YAW_MAX_RPM;
			break;

		case 3:
			*lvl_max_speed = LV3_MAX_SPEED;
			*lvl_max_accel = LV3_MAX_ACCEL;
			*lvl_max_spin  = LV3_CHASSIS_YAW_MAX_RPM;
			break;

		case 4:
			*lvl_max_speed = LV4_MAX_SPEED;
			*lvl_max_accel = LV4_MAX_ACCEL;
			*lvl_max_spin  = LV4_CHASSIS_YAW_MAX_RPM;
			break;

		case 5:
			*lvl_max_speed = LV5_MAX_SPEED;
			*lvl_max_accel = LV5_MAX_ACCEL;
			*lvl_max_spin  = LV5_CHASSIS_YAW_MAX_RPM;
			break;

		case 6:
			*lvl_max_speed = LV6_MAX_SPEED;
			*lvl_max_accel = LV6_MAX_ACCEL;
			*lvl_max_spin  = LV6_CHASSIS_YAW_MAX_RPM;
			break;

		case 7:
			*lvl_max_speed = LV7_MAX_SPEED;
			*lvl_max_accel = LV7_MAX_ACCEL;
			*lvl_max_spin  = LV7_CHASSIS_YAW_MAX_RPM;
			break;

		case 8:
			*lvl_max_speed = LV8_MAX_SPEED;
			*lvl_max_accel = LV8_MAX_ACCEL;
			*lvl_max_spin  = LV8_CHASSIS_YAW_MAX_RPM;
			break;

		case 9:
			*lvl_max_speed = LV9_MAX_SPEED;
			*lvl_max_accel = LV9_MAX_ACCEL;
			*lvl_max_spin  = LV9_CHASSIS_YAW_MAX_RPM;
			break;

		case 10:
			*lvl_max_speed = LV10_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 11:
			*lvl_max_speed = LV11_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 12:
			*lvl_max_speed = LV12_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 13:
			*lvl_max_speed = LV13_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 14:
			*lvl_max_speed = LV14_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 15:
			*lvl_max_speed = LV15_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 16:
			*lvl_max_speed = LV16_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 17:
			*lvl_max_speed = LV17_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 18:
			*lvl_max_speed = LV18_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 19:
			*lvl_max_speed = LV19_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		case 20:
			*lvl_max_speed = LV20_MAX_SPEED;
			*lvl_max_accel = LV10_MAX_ACCEL;
			*lvl_max_spin  = LV10_CHASSIS_YAW_MAX_RPM;
			break;

		default:
			*lvl_max_speed = LV1_MAX_SPEED;
			*lvl_max_accel = LV1_MAX_ACCEL;
			*lvl_max_spin  = LV1_CHASSIS_YAW_MAX_RPM;
	}
#else

	*lvl_max_speed = MAX_SPEED;
	*lvl_max_accel = MAX_ACCEL;
	*lvl_max_spin  = CHASSIS_YAW_MAX_RPM;

#endif
	*lvl_max_speed = (*lvl_max_speed < 0) ? 0 : *lvl_max_speed; //Make sure is within 0 - 1 since it is a percentage
	*lvl_max_speed = (*lvl_max_speed > 1) ? 1 : *lvl_max_speed; // Cap the max speed of motor
}


float rpm_ramp(float target_value, float current_value, float *lvl_max_accel) {
	double dt = CHASSIS_DELAY / 1000.0; // Converting dt to minutes
	double accel = *lvl_max_accel; //Default Chassis_Accel_max is LV1_ACCEL_MAX

	double ramp_rate = accel * dt; //Calc ramp_rate from max_accel
	float delta = target_value - current_value;

	if (target_value == 0) {
		return 0; //Instantly stop the robot;
	} else if (fabs(delta) < ramp_rate) {
        return target_value;  // close enough, just snap to target
    } else {
        return current_value + (delta > 0 ? ramp_rate : -ramp_rate);
    }
}



#ifdef HALL_ZERO
void yaw_zeroing(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr){
	if (!zero_start && (g_remote_cmd.right_switch == ge_RSW_ALL_ON)) {
		zeroing_start_time = HAL_GetTick();
		zero_start = 1;
	}
	if (zero_start && (HAL_GetTick() - zeroing_start_time > HALL_TIMEOUT)){
		hall_int();
		g_can_motors[YAW_MOTOR_ID-1].angle_data.center_ang = 0;
	}
	float yaw_rpm[4];
	yaw_rpm[0] = ZERO_SPEED * motor_yaw_mult[0];
	yaw_rpm[1] = ZERO_SPEED * motor_yaw_mult[1];
	yaw_rpm[2] = ZERO_SPEED * motor_yaw_mult[2];
	yaw_rpm[3] = ZERO_SPEED * motor_yaw_mult[3];

	speed_pid(yaw_rpm[0], motorfr->raw_data.rpm, &motorfr->rpm_pid);
	speed_pid(yaw_rpm[1], motorfl->raw_data.rpm, &motorfl->rpm_pid);
	speed_pid(yaw_rpm[2], motorbl->raw_data.rpm, &motorbl->rpm_pid);
	speed_pid(yaw_rpm[3], motorbr->raw_data.rpm, &motorbr->rpm_pid);

	motorfr->output = motorfr->rpm_pid.output;
	motorfl->output = motorfl->rpm_pid.output;
	motorbl->output = motorbl->rpm_pid.output;
	motorbr->output = motorbr->rpm_pid.output;
}
#endif


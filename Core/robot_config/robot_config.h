

#ifndef TASKS_INC_ROBOT_CONFIG_H_
#define TASKS_INC_ROBOT_CONFIG_H_

#include "typedefs.h"

//#include "robot_config_HERO_GIMBAL.h"
//#include "robot_config_dm.h"
//#include "robot_config_hero.h"
//#include "robot_config_NEW_HERO.h"

#include "robot_config_kirbee.h"
//#include "robot_config_sentry.h"
//#include "robot_config_TALI_LK.h"
//#include "robot_config_waddledee.h"


// COMMON CONFIGURATION

#define SPINSPIN_RANDOM_DELAY 50
#define MOTOR_ONLINE_CHECK 	-1
//1 for annoying beep sound, 0 for some error beeps every 3s, -1 for absolute peace and tranquility (with pancik
#define ARM_SWITCH 			0			//set to 1 to enable remote up and down arming switch
#define FIRING_DISABLE		0			//set to 1 to stop firing
#define LAUNCHER_SAFETY		0			//set to 1 to enable launcher arm safety
#define FRICTION_SB_SPIN_ON	1			// set to allow flywheels to spin during standby; 2(always spin), 1 (spin when in comp)
//(between mode switch, remote off, power on, left switch MUST be down)


//if no overrides in the respective configs
#ifndef CONTROL_DEFAULT
//#define CONTROL_DEFAULT 		KEYBOARD_CTRL_MODE
#define CONTROL_DEFAULT			REMOTE_CTRL_MODE
//#define CONTROL_DEFAULT			SBC_CTRL_MODE
#endif

#ifndef DAMAGE_TIMEOUT
#define DAMAGE_TIMEOUT 5000
#endif

#ifndef PITCH_SURVEILLANCE
#define PITCH_SURVEILLANCE -0.1
#endif

/*********************** OTHERS ***********************/
#define MOTOR_TIMEOUT_MAX	1000000

#define RC_LIMITS			660
#define HITEMP_WARNING  	70




#define KEY_OFFSET_W        ((uint16_t)0x01<<0)
#define KEY_OFFSET_S        ((uint16_t)0x01<<1)
#define KEY_OFFSET_A 		((uint16_t)0x01<<2)
#define KEY_OFFSET_D        ((uint16_t)0x01<<3)
#define KEY_OFFSET_Q        ((uint16_t)0x01<<6)
#define KEY_OFFSET_E        ((uint16_t)0x01<<7)
#define KEY_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define KEY_OFFSET_CTRL     ((uint16_t)0x01<<5)

#endif /* TASKS_INC_ROBOT_CONFIG_H_ */


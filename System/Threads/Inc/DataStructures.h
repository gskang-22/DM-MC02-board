/*
 * DataStructs.h
 *
 *  Created on: Sep 28, 2023
 *      Author: YassineBakkali
 */

#ifndef UTILS_DATASTRUCTURES_H_
#define UTILS_DATASTRUCTURES_H_

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string.h>

struct Vector3 {
	float x;
	float y;
	float z;

	char* toString(char* buffer) {
		sprintf(buffer, "[%+.3f %+.3f %+.3f]", x, y, z);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer + 0) = x;
		*(float*)(buffer + 1*4) = y;
		*(float*)(buffer + 2*4) = z;
		return buffer;
	}

};

struct Quaternion {
	float w;
	float x;
	float y;
	float z;
	char* toString(char* buffer) {
		sprintf(buffer, "[%+.3f %+.3f %+.3f %+.3f]", w, x, y, z);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer + 0) = w;
		*(float*)(buffer + 1*4) = x;
		*(float*)(buffer + 2*4) = y;
		*(float*)(buffer + 3*4) = z;
		return buffer;
	}

};

struct DummyData {
    int data;
    char* toString(char* buffer) {
        sprintf(buffer, "Data: %d", data); // beware of the type: (%d, %f, ...)
        return buffer;
    }

    uint8_t* toArray(uint8_t* buffer){
        *(int*)(buffer) = data;
        return buffer;
    }
};

struct IMUData {
	Vector3 accel;
	Vector3 gyro;
	Vector3 orientation;

	char* toString(char* buffer) {
		static char buf_accel[32];
		static char buf_gyro[32];
		static char buf_quat[32];
		sprintf(buffer, "Acc: %s \t Gyro: %s \t Quat: %s", accel.toString(buf_accel), gyro.toString(buf_gyro), orientation.toString(buf_quat));
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		accel.toArray(buffer);
		gyro.toArray(buffer + 3*4);
		orientation.toArray(buffer + 6*4);
		return buffer;
	}
};

struct MagData {
	Vector3 mag;
	Vector3 mag_raw;

	char* toString(char* buffer) {
		static char buf_mag[32];
		static char buf_mag_raw[32];
		sprintf(buffer, "Mag: %s [uT] \t Mag raw: %s [uT]", mag.toString(buf_mag), mag_raw.toString(buf_mag_raw));
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		mag.toArray(buffer);
		mag_raw.toArray(buffer + 3*4);
		return buffer;
	}
};

struct gimbalJointData {
	float yaw_gimbal;
	float pitch_gimbal;

	char* toString(char* buffer) {
		static char buf_ticks[32];
		sprintf(buffer, "gimbal_yaw: %+.3f , gimbal_pitch: %+.3f", yaw_gimbal, pitch_gimbal);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer + 0) = yaw_gimbal;
		*(float*)(buffer + 4) = pitch_gimbal;
		return buffer;
	}
};

struct dummyData {
	int num[3];

	char* toString(char* buffer) {
		sprintf(buffer, "ticks: %d %d %d", num[0], num[1], num[2]);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(int*)(buffer) = num[0];
		*(int*)(buffer + 4) = num[1];
		*(int*)(buffer + 8) = num[2];
		return buffer;
	}
};

struct imuData {
	float pitch;
	float yaw;

	char* toString(char* buffer) {
		sprintf(buffer, "Pitch: %.3f \t Yaw: %.3f", pitch, yaw);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(int*)(buffer) = pitch;
		*(int*)(buffer + 4) = yaw;
		return buffer;
	}
};

struct chassisSpeedCommandData {
	float speed_horz;
	float speed_lat;
	float speed_yaw;

	char* toString(char* buffer) {
		sprintf(buffer, "speed_horz: %.3f \t speed_lat: %.3f \t speed_yaw: %.3f", speed_horz, speed_lat, speed_yaw);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(int*)(buffer) = speed_horz;
		*(int*)(buffer + 4) = speed_lat;
		*(int*)(buffer + 8) = speed_yaw;
		return buffer;
	}
};

struct gimbalAngleCommandData {
	float pitch;
	float yaw;

	char* toString(char* buffer) {
		sprintf(buffer, "gimbal_pitch: %.3f \t gimbal_yaw: %.3f", pitch, yaw);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer) = pitch;
		*(float*)(buffer + 4) = yaw;
		return buffer;
	}
};

struct RobotStatusData {
	uint16_t game_progress; // Which stage the competition is in
	uint16_t time_left; // Time left in competition (s)
//	uint16_t red_base_hp; // HP of red base
//	uint16_t blue_base_hp; // HP of blue base
//	uint16_t virtual_shield_hp; // HP of virtual shield (% in integer)
	uint16_t robot_id;
	uint16_t current_hp;
	uint16_t occupy_central;
	uint16_t ammo;

	char* toString(char* buffer) {
		sprintf(buffer, "Game progress: %u \t Time left: %u \t Robot id: %u \t\n Hp: %u \t Central occupation: %u \t Remaining ammo: %u",
				game_progress, time_left, robot_id, current_hp, occupy_central, ammo);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint16_t*)(buffer) = game_progress;
		*(uint16_t*)(buffer + 2) = time_left;
		*(uint16_t*)(buffer + 4) = robot_id;
		*(uint16_t*)(buffer + 6) = current_hp;
		*(uint16_t*)(buffer + 8) = occupy_central;
		*(uint16_t*)(buffer + 10) = ammo;
		return buffer;
	}
};

struct LeftTriggerPositionData {
	uint8_t trigger_position; // Position of the left trigger

	char* toString(char* buffer) {
		sprintf(buffer, "Left trigger position: %d",
				(int)trigger_position);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint8_t*)(buffer) = trigger_position;
		return buffer;
	}
};

struct RightTriggerPositionData {
	uint8_t trigger_position; // Position of the left trigger

	char* toString(char* buffer) {
		sprintf(buffer, "Right trigger position: %d",
				(int)trigger_position);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint8_t*)(buffer) = trigger_position;
		return buffer;
	}
};

struct gimbalAngleYawCommandData {
	float yaw; // Position of the left trigger

	char* toString(char* buffer) {
		sprintf(buffer, "yaw position: %d",
				(int)yaw);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer) = yaw;
		return buffer;
	}
};

struct gimbalAnglePitchCommandData {
	float pitch; // Position of the left trigger

	char* toString(char* buffer) {
		sprintf(buffer, "Right trigger position: %d",
				(int)pitch);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer) = pitch;
		return buffer;
	}
};

struct SideDialData {
	bool robot_mode; // Position of the left trigger

	char* toString(char* buffer) {
		sprintf(buffer, "Robot mode: %d",
				(int)robot_mode);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(bool*)(buffer) = robot_mode;
		return buffer;
	}
};

struct SuperCapData {
	float V_cap;
	float P_chassis;
	uint8_t charge_state;

	char* toString(char* buffer) {
			sprintf(buffer, "Capacitor voltage: %.3f \t Chassis Power : %.3f \t Charge state : %d",
					V_cap, P_chassis, charge_state);
			return buffer;
		}

	uint8_t* toArray(uint8_t* buffer) {
		*(float*)(buffer) = V_cap;
		*(float*)(buffer + 4) = P_chassis;
		*(uint8_t*)(buffer + 8) = charge_state;
		return buffer;
	}
};

struct MaxChassisPowerData {
	uint8_t maxChassisPower;

	char* toString(char* buffer) {
			sprintf(buffer, "Max Chassis Power: %d",
					maxChassisPower);
			return buffer;
		}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint8_t*)(buffer) = maxChassisPower;
		return buffer;
	}
};

struct competitionStatusData {
	  uint16_t game_progress; // Which stage the competition is in
	  uint16_t time_left; // Time left in competition (s)
	  uint16_t robot_id;
	  uint16_t current_hp;

	  uint16_t red_hero_hp;
	  uint16_t red_standard_hp;
	  uint16_t red_sentry_hp;
	  uint16_t blue_hero_hp;
	  uint16_t blue_standard_hp;
	  uint16_t blue_sentry_hp;

	char* toString(char* buffer) {
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint16_t*)(buffer) = game_progress;
		*(uint16_t*)(buffer + 2) = time_left;
		*(uint16_t*)(buffer + 4) = robot_id;
		*(uint16_t*)(buffer + 6) = current_hp;

		*(uint16_t*)(buffer + 8) = red_hero_hp;
		*(uint16_t*)(buffer + 10) = red_standard_hp;
		*(uint16_t*)(buffer + 12) = red_sentry_hp;
		*(uint16_t*)(buffer + 14) = blue_hero_hp;
		*(uint16_t*)(buffer + 16) = blue_standard_hp;
		*(uint16_t*)(buffer + 18) = blue_sentry_hp;

		return buffer;
	}
};

struct occupationStatusData {
	  /* Occupation status of the Central Buff Point
	   * a value of 0 indicates that it is not occupied;
	   * a value of 1 indicates that it is occupied by the own side;
	   * a value of 2 indicates that it is occupied by the opponent;
	   * a value of 3 indicates that it is occupied by both sides */
	  uint16_t central_occupation;
	  uint16_t resupply_occupation; // own side resupply occupation state

	char* toString(char* buffer) {
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(uint16_t*)(buffer) = central_occupation;
		*(uint16_t*)(buffer + 2) = resupply_occupation;

		return buffer;
	}
};

struct winStatusData {
	  bool win_state;

	char* toString(char* buffer) {
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(bool*)(buffer) = win_state;
		return buffer;
	}
};


// Aimbot commands
struct cvGimbalCommandData {
	float pitch;
	float yaw;

	char* toString(char* buffer) {
		sprintf(buffer, "Pitch: %.3f \t Yaw: %.3f", pitch, yaw);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(int*)(buffer) = pitch;
		*(int*)(buffer + 4) = yaw;
		return buffer;
	}
};

struct cvFiringCommandData {
	bool fire_state;

	char* toString(char* buffer) {
		sprintf(buffer, "Fire state: %d",
				(int)fire_state);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(bool*)(buffer) = fire_state;
		return buffer;
	}
};

struct cvAimCommandData {
	bool aim_state;

	char* toString(char* buffer) {
		sprintf(buffer, "Aim state: %d",
				(int)aim_state);
		return buffer;
	}

	uint8_t* toArray(uint8_t* buffer) {
		*(bool*)(buffer) = aim_state;
		return buffer;
	}
};



#endif /* UTILS_DATASTRUCTURES_H_ */

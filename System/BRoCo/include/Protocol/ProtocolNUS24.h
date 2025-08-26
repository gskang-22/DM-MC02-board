/*
 * ProtocolNUS24.h
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#ifndef BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_
#define BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_

#include "ProtocolMacros.h"

#include <cstdint>

#include <vector>

//RELIABLE_PACKET(IMUPacket,
//float acceleration[3];				//[m/s^2]
//float angular[3];					//[°/s]
//float orientation[3];				//[rad]
//)
//
//RELIABLE_PACKET(MagPacket,
//float mag[3];
//float mag_raw[3];
//)
//
//RELIABLE_PACKET(gimbalJointsPacket,
//float yaw_angle;
//float pitch_angle;
//)
//
//RELIABLE_PACKET(chassisJointsPacket,
//float front_right_angle;
//float front_left_angle;
//float back_right_angle;
//float back_left_angle;
//)
//
//
RELIABLE_IDENTIFIABLE_PACKET(dummyPacket,
  int num1;
  int num2;
  int num3;
)
//
//RELIABLE_PACKET(imuPacket,
//  float pitch;
//  float yaw;
//)
//
RELIABLE_PACKET(chassisSpeedCommandPacket,
  float V_horz;
  float V_lat;
  float V_yaw;
)
//
//RELIABLE_PACKET(gimbalAngleCommandPacket,
//  float pitch;
//  float yaw;
//)
//
//RELIABLE_PACKET(gimbalAnglePitchCommandPacket,
//  float pitch;
//  float yaw;
//)
//
//RELIABLE_PACKET(gimbalAngleYawCommandPacket,
//  float yaw;
//)
//
RELIABLE_PACKET(leftTriggerPositionPacket,
  uint8_t trigger_pos; // [0: Attack mode, 1: Balance mode, 2: Sandbox mode]
)
//
//RELIABLE_PACKET(RightTriggerPositionPacket,
//  uint8_t trigger_pos; // [0: Sentry control mode, 1: Manual control mode, 2: Sentry down]
//)
//
//RELIABLE_PACKET(FrontFiringPacket,
//bool fire_state; // Tells the front launcher to shoot or not (1 : True, 0 : False)
//)
//
//RELIABLE_PACKET(BackFiringPacket,
//bool fire_state; // Tells the back launcher to shoot or not (1 : True, 0 : False)
//)
//
//RELIABLE_PACKET(ChassisSpinCommandPacket,
//bool spining_state; // Tells the chassis to spin (1 : True, 0 : False)
//)

//RELIABLE_PACKET(RobotStatusPacket,
////  uint16_t competition_time; // Time of the match
//  uint16_t game_progress; // Which stage the competition is in
//  uint16_t time_left; // Time left in competition (s)
////  uint16_t red_base_hp; // HP of red base
////  uint16_t blue_base_hp; // HP of blue base
////  uint16_t virtual_shield_hp; // HP of virtual shield (% in integer)
//  uint16_t robot_id;
//  uint16_t current_hp;
//  uint16_t occupy_central;
//  uint16_t ammo;
//)
//
//RELIABLE_PACKET(SideDialPacket,
//  bool robot_mode;  // [0: Attack mode, 1: Balance mode]
//)
RELIABLE_PACKET(isNavigatingPacket,
  bool navigating_state; // [0: Not navigating, 1: Is navigating]
)

RELIABLE_PACKET(chassisSpinCommandPacket,
  bool spinning_state;
)

RELIABLE_IDENTIFIABLE_PACKET(SuperCapDataPacket,
  float V_cap;
  float P_chassis;
  uint8_t charge_state;
)

RELIABLE_IDENTIFIABLE_PACKET(MaxChassisPowerPacket,
  uint8_t max_chassis_power;
)


RELIABLE_PACKET(competitionStatusPacket,
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
)

RELIABLE_PACKET(occupationStatusPacket,
  uint16_t central_occupation;
  uint16_t resupply_occupation;
)

RELIABLE_PACKET(winStatusPacket,
  bool win_state;
)

// Aimbot packets
RELIABLE_PACKET(cvGimbalCommandPacket,
  float yaw;
  float pitch;
)

RELIABLE_PACKET(firingCommandPacket,
  bool fire_state; // [0: Stop firing, 1: Start firing]
)

RELIABLE_PACKET(aimCommandPacket,
  bool aim_state; // [0: Stop aiming, 1: Start aiming]
)

#endif /* BROCO_INCLUDE_PROTOCOL_PROTOCOLNUS24_H_ */

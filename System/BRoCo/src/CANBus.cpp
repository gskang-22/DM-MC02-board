/*
 * NetworkBus.cpp
 *
 *  Created on: 8 May 2020
 *      Author: Yassine
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_CAN_BUS


#include "CANBus.h"
#include "Protocol/Protocol.h"

CANBus::CANBus(IODriver* driver) : IOBus(driver, can_frame, sizeof(can_frame)) {

	// Sentry

//    define<IMUPacket>(1);
//    define<chassisJointsPacket>(2);
//    define<gimbalJointsPacket>(3);
    define<dummyPacket>(4);
//    define<imuPacket>(5);
//    define<chassisSpeedCommandPacket>(6);
//    define<gimbalAngleCommandPacket>(7);
//    define<LeftTriggerPositionPacket>(8);
//    define<RightTriggerPositionPacket>(9);
//    define<ChassisSpinCommandPacket>(10);
//    define<FrontFiringPacket>(11);
//    define<BackFiringPacket>(12);
//    define<RobotStatusPacket>(13);
//    define<gimbalAngleYawCommandPacket>(14);
//    define<gimbalAnglePitchCommandPacket>(15);
//    define<SideDialPacket>(16);
    define<SuperCapDataPacket>(17);
    define<MaxChassisPowerPacket>(18);
}

#endif /* BUILD_WITH_CAN_BUS */

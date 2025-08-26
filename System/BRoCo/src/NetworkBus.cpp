/*
 * NetworkBus.cpp
 *
 *  Created on: 8 May 2020
 *      Author: Arion, Yassine, Vincent
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_NETWORK_BUS


#include "NetworkBus.h"
#include "Protocol/Protocol.h"

NetworkBus::NetworkBus(IODriver* driver) : IOBus(driver, network_frame, sizeof(network_frame)) {

	// Sentry

   // define<IMUPacket>(1);
//    define<chassisJointsPacket>(2);
//    define<gimbalJointsPacket>(3);
   define<dummyPacket>(4);
//    define<imuPacket>(5);
   define<chassisSpeedCommandPacket>(6);
//    define<gimbalAngleCommandPacket>(7);
   define<leftTriggerPositionPacket>(8);
//    define<RightTriggerPositionPacket>(9);
   define<chassisSpinCommandPacket>(10);
   define<competitionStatusPacket>(11);
   define<cvGimbalCommandPacket>(12);
   define<firingCommandPacket>(13);
//    define<SideDialPacket>(15);
   define<aimCommandPacket>(16);
   define<isNavigatingPacket>(17);
   define<occupationStatusPacket>(19);
   define<winStatusPacket>(20);
}


#endif /* BUILD_WITH_NETWORK_BUS */

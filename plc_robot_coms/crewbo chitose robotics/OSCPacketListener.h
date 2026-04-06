/*

    Move Mitsubishi robot using position read over OSC from a controller 
	requires this library :- https://chitose-robotics.com/product and http://www.rossbencina.com/code/oscpack
	  
*/
#ifndef __ROBOOSC_PACKET_LISTENER
#define __ROBOOSC_PACKET_LISTENER

#include <iostream>
#include <optional>
#include <string>
#include "crewbo/crewbo.h"
#include "project_lib/joystick.h"
#include "project_lib/vision_assistant.h"
#include <math>

//----------------------------------------------------------------------
//	OSC ref:- https://note.com/prty/n/n4549edcef3e4?creator_urlname=prty
//----------------------------------------------------------------------
#include "osc/OscReceivedElements.h"
#include "osc/OscPacketListener.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include "ip/IpEndpointName.h"

class OSCReceiveHandler : public osc::OscPacketListener {
private:
public:
	OSCReceiveHandler(){}
	~OSCReceiveHandler(){}
protected:
	bool isOSCReceiveBegan = false;
	// choose mitsubishi melfa robot RV-4FR-L or RV-5AS robot
    const std::string ip_addr = "192.168.15.20";
	const unsigned int iport = 10000U;
#define USING_ROBOT 3                                                                // RV-5AS with specified ip address and port
#ifdef USING_ROBOT == 0	
    crewbo::robot::melfa::base_kit::MelfaRv4frl robot;
#endif
#ifdef USING_ROBOT == 3
    crewbo::robot::melfa::base_kit::MelfaRv5as robot(1.f, ip_addr, iport);
#endif
#ifdef USING_ROBOT == 1	
    crewbo::robot::melfa::base_kit::MelfaRv4frl robot(1.f, ip_addr, iport);
#endif
#ifdef USING_ROBOT == 2
    crewbo::robot::melfa::base_kit::MelfaRv5as robot;
#endif
    using namespace crewbo::literals;

    crewbo::robot::concurrent_updater::ConcurrentPoseUpdater updater(&robot, 0.2f, 1e-3f);
	virtual void ProcessMessage(const osc::ReceivedMessage& m, const IpEndpointName& remoteEndpoint)
	{
		(void)remoteEndpoint;
		try{
			if (strcmp(m.AddressPattern(), "/position") == 0){
				// if address is "/position"
				// osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                osc::ReceivedMessage::const_iterator arg = m.ArgumentsBegin();
                const char *messageFromIp = (arg++)->AsString();
            	float input_x = (arg++)->AsFloat32();
            	float input_y = (arg++)->AsFloat32();
            	float input_z = (arg++)->AsFloat32();
            	float input_ra = (arg++)->AsFloat32();
            	float input_rb = (arg++)->AsFloat32();
            	float input_rc = (arg++)->AsFloat32();
                if( arg != m.ArgumentsEnd() )
                    throw osc::ExcessArgumentException();
                const crewbo::Pose input_rel_pose{input_x, input_y, input_z, input_ra, input_rb, input_rc};	
                updater.updateGoalByRelPose_(input_rel_pose);				
			} else {
                std::cout << "unsupported osc packet" << satd::endl;			
			}
		}
		catch (osc::Exception& e){
			std::cout << "OSC Receive: Error while parsing message: "
				<< m.AddressPattern() << ": " << e.what() << "\\n";
		}
	}
};

#endif
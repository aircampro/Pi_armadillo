/*
   threads the osc packet listener to the robot action code 
*/

#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include "calories_def.h"
#include "util_thread.h"
#include "OSCPacketListener.h"
#define PORT 7000

// listen for OSC position message from anywhere and move specified mitsubishi melfa arm to the requested x.y.z position (pose)
void * osc_listener( void * )
{
    // instantiate listener
    OSCReceiveHandler listener;
	UdpListeningReceiveSocket receiveSocket( IpEndpointName( IpEndpointName::ANY_ADDRESS, PORT ), &listener );
    cerr << "OSC listener started on port: " << PORT << "..." << endl;
    // go!
    receiveSocket.RunUntilSigInt();
    return NULL;
}

int main(int argc, char *argv[])
{
  XThread thread;
  thread.start( osc_listener );	
  while (p_run == true) {
      // do what else you need 
  }
}
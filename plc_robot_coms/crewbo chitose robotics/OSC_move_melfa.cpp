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
    std::cout << "OSC listener started on port: " << PORT << "..." << std::endl;
    // go!
    receiveSocket.RunUntilSigInt();
    return NULL;
}

bool p_run = true;
void signal_callback_handler(int signum)
{
   std::cout << "Caught signal " << signum << std::endl;
   p_run = false;
}

int main(int argc, char *argv[])
{
  // declare handlers for clean-up 
  signal(SIGINT, signal_callback_handler);                     // ctl-calculate
  signal(SIGILL, signal_callback_handler);                     
  signal(SIGQUIT, signal_callback_handler);                    
  signal(SIGHUP, signal_callback_handler);                
  signal(SIGTRAP, signal_callback_handler);           
  signal(SIGABRT, signal_callback_handler);               
  signal(SIGBUS, signal_callback_handler);              
  signal(SIGFPE, signal_callback_handler);                   
  signal(SIGUSR1, signal_callback_handler);                   // user kills
  signal(SIGUSR2, signal_callback_handler);  
  signal(SIGSEGV, signal_callback_handler);  
  signal(SIGPIPE, signal_callback_handler);  
  signal(SIGTERM, signal_callback_handler);  
  signal(SIGSTKFLT, signal_callback_handler);  
  signal(SIGSTOP, signal_callback_handler);  
  signal(SIGTSTP, signal_callback_handler);  
  signal(SIGXCPU, signal_callback_handler); 
  signal(SIGSYS, signal_callback_handler);  
  signal(SIGPWR, signal_callback_handler); 
  XThread thread;
  thread.start( osc_listener );	
  while (p_run == true) {
      // do what else you need 
  }
}
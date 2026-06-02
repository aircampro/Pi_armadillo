/**
 * @file WritePos.cpp
 * @brief Example: Write PWM commands to servo type SCSSCL
 * https://github.com/Ar-Ray-code/feetech_ros2_driver
 * https://servodatabase.com/servos/feetech
 * 
 * Demonstrates usage of SCServo SDK functions for Feetech serial servos.
 */
#include <iostream>
#include <signal>
#include "SCServo.h"

SCSCL sc;
int run_s = 1;

void stop_loop() {
	run_s == 0;
}

int main(int argc, char **argv)
{
	int mode = 0;
	signal(SIGINT, stop_loop());
	signal(SIGUSR1, stop_loop());
	
	if(argc<3){
        std::cout<<"argc error! <prog> <portname> <mode>"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sc.begin(115200, argv[1])){
        std::cout<<"Failed to init scscl motor!"<<std::endl;
        return 0;
    }
	mode = stoi(argv[2]);
	switch mode {
		case 0:
		{
	        sc.PWMMode(1);
	        std::cout<<"mode = "<<1<<std::endl;
	        while( run_s == 1 ) {
		        sc.WritePWM(1, 500);
		        std::cout<<"pwm = "<<500<<std::endl;
		        sleep(2);
		        sc.WritePWM(1, 0);
		        std::cout<<"pwm = "<<0<<std::endl;
		        sleep(2);
		        sc.WritePWM(1, -500);
		        std::cout<<"pwm = "<<-500<<std::endl;
		        sleep(2);
		        sc.WritePWM(1,0);
		        std::cout<<"pwm = "<<0<<std::endl;
		        sleep(2);
	        }
		}
		break;
		case 1:
		{
	        while( run_s == 1 ) {
		        sc.WritePos(1, 1000, 0, 1500);                      
		        std::cout<<"pos = "<<1000<<std::endl;
		        usleep(754*1000);                                    
		        sc.WritePos(1, 20, 0, 1500);                       
		        std::cout<<"pos = "<<20<<std::endl;
		        usleep(754*1000);                                   
			}
		}
		break;
	}
	sc.end();
	return 1;
}
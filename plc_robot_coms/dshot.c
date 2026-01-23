/*

    Example of how to test motor using dshot with raspi 5 ref:- https://github.com/Marian-Vittek/raspberry-pi-dshot-pio/tree/main
	
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#define CMD0_ESC                                                                           /* uncomment for other type of ESC */
#define BLHELLI32

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;
extern void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int repeatCounter) ;
#define MAXN 	40
int motorPins[MAXN];
double throttles[MAXN];

int main(int argc, char **argv) {
    int i, n;
    float spin_speed = 0.0f;
	
    if (argc < 2) {
	    printf("usage:   ./d_shot <gpio_pin_1> ... <gpio_pin_n> <spin speed>\n");
	    printf("example: sudo ./d_shot 16 19 20 21 0.15\n" for 15%);
	    exit(1);
    }

    n = 0;
    for(i=1; i<argc; i++) {
        if (i<=4) {
	        motorPins[n] = atoi(argv[i]);
	        if (motorPins[n] > 0 && motorPins[n] < 28) {
	            n ++;
	        } else {
	            printf("pin %d out of range 1..27\n", motorPins[n]);
	        }
        } else {
		    spin_speed = atof(argv[5]);
        }		
    }
    
    motorImplementationInitialize(motorPins, n);

    printf("Initializing ESC / Arm, waiting 5 seconds.\n");
#if defined(CMD0_ESC)    
    // to arm, send DSHOT_CMD_MOTOR_STOP (cmd 0) for 5 seconds
    dshotRepeatSendCommand(motorPins, n, 0, 0, 5000);
#else    
    // or send 0 throttle during 5 seconds (depending on your ESC)
    for(i=0; i<n; i++) throttles[i] = 0;
    for(i=0; i<5000; i++) {
        motorImplementationSendThrottles(motorPins, n, throttles);
        usleep(1000);
    }
#endif

    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 0, 5000);  
    usleep(1000);	
    printf("Spinning forward %f %\n", spin_speed);
    // make motors spinning on ss% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = spin_speed;
    for(i=0; i<5000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
    }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);
	usleep(1000);
		
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 0, 5000);  
    usleep(1000);	
    printf("Spinning forward %f %\n", spin_speed*100.0f);
    // make motors spinning on ss% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = spin_speed;
    for(i=0; i<5000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
    }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 0, 5000);  
    usleep(1000);	
    printf("Spinning forward 100 %\n");
    // make motors spinning on ss% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 1.0f;
    for(i=0; i<5000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
    }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

	usleep(1000);
		
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 0, 5000);  
    usleep(1000);	
    printf("Spinning forward 100 %\n");
    // make motors spinning on ss% throttle during 5 seconds
    for(i=0; i<n; i++) throttles[i] = 1.0f;
    for(i=0; i<5000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
    }
    
    printf("Stop.\n");
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);

#if defined(BLHELLI32)
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED2_ON, 0, 5000);  
    usleep(1000);
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED2_OFF, 0, 5000);  
    usleep(1000);	
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED0_ON, 0, 5000);  
    usleep(1000);
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED0_OFF, 0, 5000);  
    usleep(1000);
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED1_ON, 0, 5000);  
    usleep(1000);
    dshotRepeatSendCommand(motorPins, n, DSHOT_CMD_LED1_OFF, 0, 5000);  
    usleep(1000);
#endif
	
    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}
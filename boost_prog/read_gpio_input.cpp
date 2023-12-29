// ---------------------------------------------------------------------------------------------------------------------------------
//
// simple PI GPIO Example
// Compiles as :- g++ -o read_gpio_input read_gpio_input.cpp -oflash -lwiringPi  for dev Add :- -lwiringPiDev
// or inside the main program code as 
// g++-8 -std=c++17 -o read_gpio_input read_gpio_input.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK
// -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lstdc++fs -DBOOST_BIN
// D_GLOBAL_PLACEHOLDERS -lwiringPi
// 
// Install
// sudo apt-get install libi2c-dev
// sudo apt-get install git-core
// cd ~
// git clone https://github.com/WiringPi/WiringPi.git
// cd WiringPi
// ./build
//
// ----------------------------------------------------------------------------------------------------------------------------------
#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

// set-up i/o
//
const int pin=21;
const int ip=7;

#define pin_03  8       // rsp board pin:03
#define pin_05  9       // rsp board pin:05
#define pin_07  7       // rsp board pin:07
#define pin_08  15      // rsp board pin:08
#define pin_10  16      // rsp board pin:10
#define pin_11  0       // rsp board pin:11
#define pin_12  1       // rsp board pin:12
#define pin_13  2       // rsp board pin:13
#define pin_15  3       // rsp board pin:15
#define pin_16  4       // rsp board pin:16
#define pin_18  5       // rsp board pin:18
#define pin_19  12      // rsp board pin:19
#define pin_21  13      // rsp board pin:21
#define pin_22  6       // rsp board pin:22
#define pin_23  14      // rsp board pin:23
#define pin_24  10      // rsp board pin:24
#define pin_26  11      // rsp board pin:26


// this is driving an output on the input circuit interrupt
// in our main code with shall add camera->capture_image();
// to take a photo on the input being set to low
//
void myInterrupt(void)
{
  digitalWrite(pin, 1);    // High 3.3V
  printf("interrupt active\n");
  delay(1000);
  digitalWrite(pin, 0);    // High 0.0V
}

void int_signal(void){
  printf("Signal-A\n");
}

int main() {

  // set-up the wiring pi
  // 
  if (wiringPiSetup() < 0)
  {
    fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
  }

  pinMode(pin, OUTPUT);      //define the pin as an output.
  pinMode(ip, INPUT);        //define the ip as an input.
  pinMode(pin_22, INPUT);

  pullUpDnControl(pin_22, PUD_UP);

  // set-up falling edge interrupt fall to low state 0V
  //
  if (wiringPiISR(ip, INT_EDGE_FALLING, &myInterrupt) < 0)
  {
    fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno)) ;
    return 1 ;
  }

  if (wiringPiISR(pin_22, INT_EDGE_FALLING, &int_signal) < 0)
  {
    fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno)) ;
    return 1 ;
  }

  for (;;) {
      delay(1000);
  }	  

}

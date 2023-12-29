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
const int ip=4;

// this is driving an output on the input circuit interrupt
// in our main code with shall add camera->capture_image();
// to take a photo on the input being set to low
//
void myInterrupt(void)
{
  digitalWrite(pin, 1);    // High 3.3V
  delay(1000);
  digitalWrite(pin, 0);    // High 0.0V
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

  // set-up falling edge interrupt fall to low state 0V
  //
  if (wiringPiISR(ip, INT_EDGE_FALLING, &myInterrupt) < 0)
  {
    fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno)) ;
    return 1 ;
  }

  for (;;) {
      delay(1000);
  }	  

}

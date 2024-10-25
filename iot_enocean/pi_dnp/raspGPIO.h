//
//  Raspberry Pi GPIO module
#ifndef GPIO_H
#define GPIO_H

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

//  gpio_init: GPIO 
void gpio_init();
//
//  （BCM2835）
#define GPIO_INPUT    0x0       
#define GPIO_OUTPUT   0x1       
#define GPIO_ALT0     0x4
#define GPIO_ALT1     0x5
#define GPIO_ALT2     0x6
#define GPIO_ALT3     0x7
#define GPIO_ALT4     0x3
#define GPIO_ALT5     0x2
// gpio_configure: Set the pin function (make sure to set it before using the pin)
//      pin : (P1) 2,3,4,7,8,9,10,11,14,15,17,18,22,23,24,25,27
//            (P5) 28,29,30,31
//      mode: GPIO_INPUT, _OUTPUT, _ALT0, _ALT1, _ALT2, _ALT3, _ALT4, _ALT5
void gpio_configure(int pin, int mode);
//
//  //  gpio_set/clear: Set pin (3.3V), Clear (0V)
void gpio_set(int pin);
void gpio_clear(int pin);
//
//  gpio_read: （3.3V->1, 0V->0)
int gpio_read (int pin);
//
//  define pulldown/up
#define GPIO_PULLNONE 0x0
#define GPIO_PULLDOWN 0x1
#define GPIO_PULLUP   0x2
//  gpio_configure_pull: 
void gpio_configure_pull(int pin, int pullmode);

#endif
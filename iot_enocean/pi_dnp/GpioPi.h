// ref :- https://www.ei.tohoku.ac.jp/xkozima/lab/raspTutorial3.html
//
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
//  Physical address of register block
#define PERI_BASE     0x20000000        //  0x3F000000 for RPi2
#define GPIO_BASE     (PERI_BASE + 0x200000)
#define BLOCK_SIZE    4096
//   gpio[n]: GPIO related register (volatile= always access real memory)
static volatile unsigned int *Gpio = NULL;
//  gpio_init: GPIO Initialization
void gpio_init ()
{
    //  Do nothing if already initialized
    if (Gpio) return;
    int fd;
    void *gpio_map;
    //  Open dev/mem (physical memory device) (requires sudo)
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        printf("error: cannot open /dev/mem (gpio_setup)\n");
        exit(-1);
    }
    //  Map GPIO (physical memory) to gpio_map (virtual memory) in / mmap
    gpio_map = mmap(NULL, BLOCK_SIZE,
                    PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd, GPIO_BASE );
    if ((int) gpio_map == -1) {
        printf("error: cannot map /dev/mem on the memory (gpio_setup)\n");
        exit(-1);
    }
    //  Close unnecessary fd after / mmap
    close(fd);
    //  Gpio[index]: Establish access to the register as an array of integers uint32
    Gpio = (unsigned int *) gpio_map;
}
//  Pin function (BCM2835)
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
void gpio_configure (int pin, int mode)
{
    if (pin < 0 || pin > 31) {
        printf("error: pin number out of range (gpio_configure)\n");
        exit(-1);
    }
    //  Generate register number (index) and 3-bit mask
    int index = pin / 10;
    unsigned int mask = ~(0x7 << ((pin % 10) * 3));
    //  Only the corresponding FSEL of GPFSEL0/1 (3bit) is rewritten.
    Gpio[index] = (Gpio[index] & mask) | ((mode & 0x7) << ((pin % 10) * 3));
}

//  gpio_set/clear: Set pin (3.3V), Clear (0V)
void gpio_set (int pin)
{
    if (pin < 0 || pin > 31) {
        printf("error: pin number out of range (gpio_set)\n");
        exit(-1);
    }
    //  Pin 1 to output (3.3V output)
    Gpio[7] = 0x1 << pin;   //  GPSET0
}
void gpio_clear (int pin)
{
    if (pin < 0 || pin > 31) {
        printf("error: pin number out of range (gpio_clear)\n");
        exit(-1);
    }
    //  Pin to output 0 (0V output)
    Gpio[10] = 0x1 << pin;  //  GPCLR0
}

int gpio_read (int pin)
{
    if (pin < 0 || pin > 31) {
        printf("error: pin number out of range (gpio_read)\n");
        exit(-1);
    }
    // Returns the voltage of the pin (input/output: 3.3V, 1, 0V, 0)
    return (Gpio[13] & (0x1 << pin)) != 0;  //  GPLEV0
}


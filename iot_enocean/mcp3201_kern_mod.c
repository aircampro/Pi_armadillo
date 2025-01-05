// Kernal module for MCP3201 ADC 4 channel mV inputADC on Cubieboard linux
// mcp3201_kern_mod.c
// 
/* ----------------------------------------------------------------------

cubie connection to mcp is as follows

26 PD21 (LCDD21) – D1 – in
28 PD23 (LCDD23) – D2 – in
29 PD24 (LCDCLK) – CS – out
32 PD25 (LCDDE) – clk – out
30 PD26 (LCDHSYNC)-VGA-HSYNC – D3 – in
27 PD27 (LCDVSYNC)-VGA-VSYNC – D0 – in
44 3.3V (nc in 2012-08-08). I have a revision of 09-09, so there is a voltage.
38 Ground

to install and test it.
[root@alarm adcs]# insmod mcp3201_kern_mod.ko
[root@alarm adcs]# ls -lR /sys/class/mcp3201
/sys/class/mcp3201:
-rw-rw-rw- 1 root root 4096 Mar 30 21:54 mcp3201_kern_mod

[root@alarm adcs]# cat /sys/class/mcp3201/ADCs
ADC: 0 0 0 7901 mV
[root@alarm adcs]# cat /sys/class/mcp3201/ADCs
ADC: 0 0 0 7934 mV
[root@alarm adcs]# cat /sys/class/mcp3201/ADCs
ADC: 0 0 0 7918 mV

----------------------------------------------------------------------------- */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/parport.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <asm/io.h>                                                         // ioread/iowrite
#include <linux/delay.h>                                                    
	
#define SW_PA_PORTC_IO_BASE 0x01C20800

#define LEN_MSG 160
static char buf_msg[ LEN_MSG + 1 ]="";
static void* gpio_map;                                                      // gpio map
static void* PDptr;                                                         // pointer to the D port register

// global variables showing the ADC values 
static int ADC0=0;
static int ADC1=0;
static int ADC2=0;
static int ADC3=0;

// read the 4 Analog inputs - LCD pins are being re-used (so NO LCD) Cubieboard 
// CS PD24 - out
// clk PD25 - out
// D1 PD21 - in
// D2 PD23 - in
// D3 PD26 - in
// D0 PD27 - in
void readADC(){

    int i;
    int PD = ioread32(PDptr);                       // Reading the old value
    PD=PD&(~(0x1<<24));                             // CS = 0;
    PD=PD&(~(0x1<<25));                             // CLK = 0; // forming new
    iowrite32(PD,PDptr);                            // write it back
    udelay(1);                                      // delay for processing ADC CS
    ADC0=0;
    ADC1=0;
    ADC2=0;
    ADC3=0;
    for (i=14;i>=0;i--){
        PD = ioread32(PDptr);
        PD=PD|(0x1<<25);                             // CLK=1
        iowrite32(PD,PDptr);
        udelay(1);                                   // delay for generating a high level of 500 kHz clock cycles
        ADC1=ADC1<<1;
        ADC2=ADC2<<1;
        ADC3=ADC3<<1;
        ADC0=ADC0<<1;
        PD=ioread32(PDptr);                          // now read the new data
        if (PD&(0x1<<21)){                           // compile the number(s) for each channel from the inputs
            ADC1=ADC1|1;
        }
        if (PD&(0x1<<23)){
            ADC2=ADC2|1;
        }
        if (PD&(0x1<<26)){
            ADC3=ADC3|1;
        }
        if (PD&(0x1<<27)){
            ADC0=ADC0|1;
        }
        PD = ioread32(PDptr);
        PD=PD&(~(0x1<<25));                                                             // CLK = 0;
        iowrite32(PD,PDptr);
        udelay(1);                                    
    }

    ADC0=((ADC0&0x00000FFF)*1000)/61;                                                   // calculate the mV from the raw signals
    ADC1=((ADC1&0x00000FFF)*1000)/61;
    ADC2=((ADC2&0x00000FFF)*1000)/61;
    ADC3=((ADC3&0x00000FFF)*1000)/61;
    PD = ioread32(PDptr);
    PD=PD|((0x1<<24));                                                                  // CS=1
    iowrite32(PD,PDptr);
}

// write the results out
static ssize_t mcp_show( struct class *class, struct class_attribute *attr, char *buf ) {
    readADC();                                                                          // read the raw values from the ADC
    sprintf(buf_msg, "ADC: %d %d %d %d mV\n",ADC0,ADC1,ADC2,ADC3);                      // write milli-volt from each cahnnel input
    strcpy( buf, buf_msg );                                                             // copy to the buffer
    return strlen( buf );
}

// store function is set to be a dummy with no action
static ssize_t mcp_store( struct class *class, struct class_attribute *attr, const char *buf, size_t count ) {

    return count;                                                  // we are not doing anything
}

CLASS_ATTR( ADCs, 0666, &mcp_show, &mcp_store);

static struct class *mcp3201;

int __init mcp_init(void) {
    int res;
    int PD_CFG2;
    int PD_CFG3;
    mcp3201 = class_create( THIS_MODULE, "mcp3201" );
    if( IS_ERR( mcp3201 ) ) printk( "bad class create MCP3201 ADC on SPI \n" );
    res = class_create_file( mcp3201, &class_attr_ADCs );
    printk("MCP3201 ADC on SPI : interface initialized ok! \n");

    gpio_map =ioremap(SW_PA_PORTC_IO_BASE, 4096);                 // initialize GPIO
    PDptr = ioremap(SW_PA_PORTC_IO_BASE+0x7C, 4);                 // PORTD

    PD_CFG2 = ioread32(gpio_map+0x74);                            // saving the initial values
    PD_CFG3 = ioread32(gpio_map+0x78);
    //CS PD24 - out
    //clk PD25 - out
    //D3 PD26 - in
    //D0 PD27 - in
    //PD_CFG3                                                      is all ours, so you can write it in full
    PD_CFG3=0x00000011;
    //D1 PD21 - in
    //D2 PD23 - in

    PD_CFG2=PD_CFG2&0x0F0FFFFF;                                  // resetting the required input data while maintaining the remaining settings

    iowrite32(PD_CFG2,gpio_map+0x74);                            // writing settings to registers
    iowrite32(PD_CFG3,gpio_map+0x78);
    printk("ADC module initialized\n");
    iounmap(gpio_map);                                           // this pointer will no longer be needed
    return 0;
}

void mcp_cleanup(void) {
    iounmap(PDptr);                                              // forgetting the pointer to the D port register
    // remove the driver files and cleanup memory
    class_remove_file( mcp3201, &class_attr_ADCs );
    class_destroy( mcp3201 );
    return;
}

module_init( mcp_init );
module_exit( mcp_cleanup );
MODULE_LICENSE( "GPL" );
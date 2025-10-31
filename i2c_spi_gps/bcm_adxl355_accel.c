// example using bcm library for easpi to read 3-axis accelerometer ADXL355
// ref:- https://marsee101.blog.fc2.com/blog-entry-5031.html (this example is a bare metal implemetation for zybo board)
//
#include <bcm2835.h>                                     // install from https://www.airspayce.com/mikem/bcm2835/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

uint16_t clk_div = BCM2835_I2C_CLOCK_DIVIDER_148;
uint8_t slave_address = 0x3a;                            // default address for ic 
uint32_t len = 0;

#define MAX_LEN 32
char wbuf[MAX_LEN];
char buf[MAX_LEN];
int i;
uint8_t data;
int32_t read_data, data_ready;

int main(int argc, char **argv) {
 
    printf(" ... Reading ADXL355 Accelerometer ... \n");

    if (!bcm2835_init())
    {
      printf("bcm2835_init failed. Are you running as root??\n");
      return 1;
    }
       
    if (!bcm2835_i2c_begin())
    {
        printf("bcm2835_i2c_begin failed. Are you running as root??\n");
        return 1;
    }

    bcm2835_i2c_setSlaveAddress(slave_address);
    bcm2835_i2c_setClockDivider(clk_div);
    printf("Clock divider set to: %d\n", clk_div);
    printf("Slave address set to: %d\n", slave_address);   
    
    wbuf[0] = 0x2d;                                                      // stanby clear
	wbuf[1] = 0x0;
	len = 2;
    data = bcm2835_i2c_write(wbuf, len);
    printf("Write Result = %d\n", data);
    buf[0] = 0x04;                                                        // wait for data ready
    len = 1;
    do{		
       read_data = bcm2835_i2c_read(buf, len);
       data_ready = read_data & 0x01;
    } while(data_ready != 0x01);

    buf[0] = 0x0;                                                        // read a dummy
    len = 4;	
    int32_t temp = bcm2835_i2c_read(buf, len);  
	
    int32_t dataX, dataY, dataZ;
    buf[0] = 0x08;
    len = 1;
    dataX = bcm2835_i2c_read(buf, len) << 12;                             // XDATA3
    buf[0] = 0x09;
    len = 1;
    dataX |= (bcm2835_i2c_read(buf, len)) << 4);                          // XDATA2
    buf[0] = 0x0a;
    len = 1;	
    dataX |= ((bcm2835_i2c_read(buf, len)) & 0xf0) >> 4);                 // XDATA1
    if(dataX & 0x80000)                                                   // Is the 19th bit 1?
        dataX |= 0xfff00000;                                              // sign extension
    buf[0] = 0x0b;
    len = 1;		
    dataY = bcm2835_i2c_read(buf, len) << 12;                             // YDATA3
    buf[0] = 0x0c;
    len = 1;	
    dataY |= (bcm2835_i2c_read(buf, len)) << 4);                          // YDATA2
    buf[0] = 0x0d;
    len = 1;	
    dataY |= ((bcm2835_i2c_read(buf, len)) &0xf0) >> 4);                  // YDATA1
    if(dataY & 0x80000)                                                   // Is the 19th bit 1?
        dataY |= 0xfff00000;                                              // sign extension  
    buf[0] = 0x0e;
    len = 1;		
    dataZ = bcm2835_i2c_read(buf, len) << 12;                             // ZDATA3
    buf[0] = 0x0f;
    len = 1;	
    dataZ |= (bcm2835_i2c_read(buf, len) << 4);                           // ZDATA2
    buf[0] = 0x10;
    len = 1;	
    dataZ |= ((bcm2835_i2c_read(buf, len) &0xf0) >> 4); ZDATA1
    if(dataZ & 0x80000)                                                    // Is the 19th bit 1?
        dataZ |= 0xfff00000; sign extension
    printf("datax = %x, dataY = %x, dataZ = %x\n", (int)dataX, (int)dataY, (int)dataZ);

    // This I2C end is done after a transfer if specified
    bcm2835_i2c_end();   
    bcm2835_close();
    printf("... done!\n");
    return 0;
}
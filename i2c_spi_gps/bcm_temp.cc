//
// Example using the BCM library from airspayce
// ref:- https://github.com/soramimi/bme280/tree/master
//
#include <stdio.h>
#include <unistd.h>
#include "bcm2835.h"                                                    // get from https://www.airspayce.com/mikem/bcm2835/

// GPIO pin assignment for raspi I2C clock and data lines
#define I2C_SCL_PIN RPI_V2_GPIO_P1_05
#define I2C_SDA_PIN RPI_V2_GPIO_P1_03
#define ADT7410_SLAVE_ADDR		(0x48)                                  // ADT7410 temperature reading
#define IIC_SCLK_RATE		1/(100000)
#define 24FC1025_SLAVE_ADDR     (0x50)                                  // 24FC1025 memory

// macros for converting endianness if you require it
//Load unaligned 16-bit integer (little-endian encoding)
#define LOAD16LE(p) ( \
   ((uint16_t)(((uint8_t *)(p))[0]) << 0) | \
   ((uint16_t)(((uint8_t *)(p))[1]) << 8))

//Load unaligned 16-bit integer (big-endian encoding)
#define LOAD16BE(p) ( \
   ((uint16_t)(((uint8_t *)(p))[0]) << 8) | \
   ((uint16_t)(((uint8_t *)(p))[1]) << 0))

//Load unaligned 24-bit integer (little-endian encoding)
#define LOAD24LE(p) ( \
   ((uint32_t)(((uint8_t *)(p))[0]) << 0)| \
   ((uint32_t)(((uint8_t *)(p))[1]) << 8) | \
   ((uint32_t)(((uint8_t *)(p))[2]) << 16))

//Load unaligned 24-bit integer (big-endian encoding)
#define LOAD24BE(p) ( \
   ((uint32_t)(((uint8_t *)(p))[0]) << 16) | \
   ((uint32_t)(((uint8_t *)(p))[1]) << 8) | \
   ((uint32_t)(((uint8_t *)(p))[2]) << 0))

//Load unaligned 32-bit integer (little-endian encoding)
#define LOAD32LE(p) ( \
   ((uint32_t)(((uint8_t *)(p))[0]) << 0) | \
   ((uint32_t)(((uint8_t *)(p))[1]) << 8) | \
   ((uint32_t)(((uint8_t *)(p))[2]) << 16) | \
   ((uint32_t)(((uint8_t *)(p))[3]) << 24))

//Load unaligned 32-bit integer (big-endian encoding)
#define LOAD32BE(p) ( \
   ((uint32_t)(((uint8_t *)(p))[0]) << 24) | \
   ((uint32_t)(((uint8_t *)(p))[1]) << 16) | \
   ((uint32_t)(((uint8_t *)(p))[2]) << 8) | \
   ((uint32_t)(((uint8_t *)(p))[3]) << 0))

//Load unaligned 48-bit integer (little-endian encoding)
#define LOAD48LE(p) ( \
   ((uint64_t)(((uint8_t *)(p))[0]) << 0) | \
   ((uint64_t)(((uint8_t *)(p))[1]) << 8) | \
   ((uint64_t)(((uint8_t *)(p))[2]) << 16) | \
   ((uint64_t)(((uint8_t *)(p))[3]) << 24) | \
   ((uint64_t)(((uint8_t *)(p))[4]) << 32) | \
   ((uint64_t)(((uint8_t *)(p))[5]) << 40)

//Load unaligned 48-bit integer (big-endian encoding)
#define LOAD48BE(p) ( \
   ((uint64_t)(((uint8_t *)(p))[0]) << 40) | \
   ((uint64_t)(((uint8_t *)(p))[1]) << 32) | \
   ((uint64_t)(((uint8_t *)(p))[2]) << 24) | \
   ((uint64_t)(((uint8_t *)(p))[3]) << 16) | \
   ((uint64_t)(((uint8_t *)(p))[4]) << 8) | \
   ((uint64_t)(((uint8_t *)(p))[5]) << 0))

//Load unaligned 64-bit integer (little-endian encoding)
#define LOAD64LE(p) ( \
   ((uint64_t)(((uint8_t *)(p))[0]) << 0) | \
   ((uint64_t)(((uint8_t *)(p))[1]) << 8) | \
   ((uint64_t)(((uint8_t *)(p))[2]) << 16) | \
   ((uint64_t)(((uint8_t *)(p))[3]) << 24) | \
   ((uint64_t)(((uint8_t *)(p))[4]) << 32) | \
   ((uint64_t)(((uint8_t *)(p))[5]) << 40) | \
   ((uint64_t)(((uint8_t *)(p))[6]) << 48) | \
   ((uint64_t)(((uint8_t *)(p))[7]) << 56))

//Load unaligned 64-bit integer (big-endian encoding)
#define LOAD64BE(p) ( \
   ((uint64_t)(((uint8_t *)(p))[0]) << 56) | \
   ((uint64_t)(((uint8_t *)(p))[1]) << 48) | \
   ((uint64_t)(((uint8_t *)(p))[2]) << 40) | \
   ((uint64_t)(((uint8_t *)(p))[3]) << 32) | \
   ((uint64_t)(((uint8_t *)(p))[4]) << 24) | \
   ((uint64_t)(((uint8_t *)(p))[5]) << 16) | \
   ((uint64_t)(((uint8_t *)(p))[6]) << 8) | \
   ((uint64_t)(((uint8_t *)(p))[7]) << 0))

//Store unaligned 16-bit integer (little-endian encoding)
#define STORE16LE(a, p) \
   ((uint8_t *)(p))[0] = ((uint16_t)(a) >> 0) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint16_t)(a) >> 8) & 0xFFU

//Store unaligned 16-bit integer (big-endian encoding)
#define STORE16BE(a, p) \
   ((uint8_t *)(p))[0] = ((uint16_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint16_t)(a) >> 0) & 0xFFU

//Store unaligned 24-bit integer (little-endian encoding)
#define STORE24LE(a, p) \
   ((uint8_t *)(p))[0] = ((uint32_t)(a) >> 0) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint32_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint32_t)(a) >> 16) & 0xFFU

//Store unaligned 24-bit integer (big-endian encoding)
#define STORE24BE(a, p) \
   ((uint8_t *)(p))[0] = ((uint32_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint32_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint32_t)(a) >> 0) & 0xFFU

//Store unaligned 32-bit integer (little-endian encoding)
#define STORE32LE(a, p) \
   ((uint8_t *)(p))[0] = ((uint32_t)(a) >> 0) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint32_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint32_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint32_t)(a) >> 24) & 0xFFU

//Store unaligned 32-bit integer (big-endian encoding)
#define STORE32BE(a, p) \
   ((uint8_t *)(p))[0] = ((uint32_t)(a) >> 24) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint32_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint32_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint32_t)(a) >> 0) & 0xFFU

//Store unaligned 48-bit integer (little-endian encoding)
#define STORE48LE(a, p) \
   ((uint8_t *)(p))[0] = ((uint64_t)(a) >> 0) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint64_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint64_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint64_t)(a) >> 24) & 0xFFU, \
   ((uint8_t *)(p))[4] = ((uint64_t)(a) >> 32) & 0xFFU, \
   ((uint8_t *)(p))[5] = ((uint64_t)(a) >> 40) & 0xFFU,

//Store unaligned 48-bit integer (big-endian encoding)
#define STORE48BE(a, p) \
   ((uint8_t *)(p))[0] = ((uint64_t)(a) >> 40) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint64_t)(a) >> 32) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint64_t)(a) >> 24) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint64_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[4] = ((uint64_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[5] = ((uint64_t)(a) >> 0) & 0xFFU

//Store unaligned 64-bit integer (little-endian encoding)
#define STORE64LE(a, p) \
   ((uint8_t *)(p))[0] = ((uint64_t)(a) >> 0) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint64_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint64_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint64_t)(a) >> 24) & 0xFFU, \
   ((uint8_t *)(p))[4] = ((uint64_t)(a) >> 32) & 0xFFU, \
   ((uint8_t *)(p))[5] = ((uint64_t)(a) >> 40) & 0xFFU, \
   ((uint8_t *)(p))[6] = ((uint64_t)(a) >> 48) & 0xFFU, \
   ((uint8_t *)(p))[7] = ((uint64_t)(a) >> 56) & 0xFFU

//Store unaligned 64-bit integer (big-endian encoding)
#define STORE64BE(a, p) \
   ((uint8_t *)(p))[0] = ((uint64_t)(a) >> 56) & 0xFFU, \
   ((uint8_t *)(p))[1] = ((uint64_t)(a) >> 48) & 0xFFU, \
   ((uint8_t *)(p))[2] = ((uint64_t)(a) >> 40) & 0xFFU, \
   ((uint8_t *)(p))[3] = ((uint64_t)(a) >> 32) & 0xFFU, \
   ((uint8_t *)(p))[4] = ((uint64_t)(a) >> 24) & 0xFFU, \
   ((uint8_t *)(p))[5] = ((uint64_t)(a) >> 16) & 0xFFU, \
   ((uint8_t *)(p))[6] = ((uint64_t)(a) >> 8) & 0xFFU, \
   ((uint8_t *)(p))[7] = ((uint64_t)(a) >> 0) & 0xFFU

//Swap a 16-bit integer
#define SWAPINT16(x) ( \
   (((uint16_t)(x) & 0x00FFU) << 8) | \
   (((uint16_t)(x) & 0xFF00U) >> 8))

//Swap a 32-bit integer
#define SWAPINT32(x) ( \
   (((uint32_t)(x) & 0x000000FFUL) << 24) | \
   (((uint32_t)(x) & 0x0000FF00UL) << 8) | \
   (((uint32_t)(x) & 0x00FF0000UL) >> 8) | \
   (((uint32_t)(x) & 0xFF000000UL) >> 24))

//Swap a 64-bit integer
#define SWAPINT64(x) ( \
   (((uint64_t)(x) & 0x00000000000000FFULL) << 56) | \
   (((uint64_t)(x) & 0x000000000000FF00ULL) << 40) | \
   (((uint64_t)(x) & 0x0000000000FF0000ULL) << 24) | \
   (((uint64_t)(x) & 0x00000000FF000000ULL) << 8) | \
   (((uint64_t)(x) & 0x000000FF00000000ULL) >> 8) | \
   (((uint64_t)(x) & 0x0000FF0000000000ULL) >> 24) | \
   (((uint64_t)(x) & 0x00FF000000000000ULL) >> 40) | \
   (((uint64_t)(x) & 0xFF00000000000000ULL) >> 56))

// define a structure to contain all data on the i2c line connected on SCL pi5, SDA pi3
typedef struct  {
    double Temperature;                                                        
    unsigned int SetPoint; 
    unsigned int MemState1;	
	float MemReal1;
} i2cData_t; 


// software I2C
class I2C {
private:

	void delay()
	{
		// Bit-bang
		// usleep(10);
	}

	// assign pins to SDA SCL lines
	void init_i2c()
	{
		// SCL bind to pin number 
		bcm2835_gpio_fsel(I2C_SCL_PIN, BCM2835_GPIO_FSEL_INPT);
		// SDA bind to pin number 
		bcm2835_gpio_fsel(I2C_SDA_PIN, BCM2835_GPIO_FSEL_INPT);
		// SCL LOW
		bcm2835_gpio_clr(I2C_SCL_PIN);
		// SDA LOW
		bcm2835_gpio_clr(I2C_SDA_PIN);
	}

	void i2c_cl_0()
	{
		// SCL LOW
		bcm2835_gpio_fsel(I2C_SCL_PIN, BCM2835_GPIO_FSEL_OUTP);
	}

	void i2c_cl_1()
	{
		// SCL HIGH
		bcm2835_gpio_fsel(I2C_SCL_PIN, BCM2835_GPIO_FSEL_INPT);
	}

	void i2c_da_0()
	{
		// SDA LOW
		bcm2835_gpio_fsel(I2C_SDA_PIN, BCM2835_GPIO_FSEL_OUTP);
	}

	void i2c_da_1()
	{
		// SDA HIGH
		bcm2835_gpio_fsel(I2C_SDA_PIN, BCM2835_GPIO_FSEL_INPT);
	}

	int i2c_get_da()
	{
		// SDA
		return bcm2835_gpio_lev(I2C_SDA_PIN) ? 1 : 0;
	}


	void i2c_start()
	{
		i2c_da_0(); // SDA=0
		delay();
		i2c_cl_0(); // SCL=0
		delay();
	}

	void i2c_stop()
	{
		i2c_cl_1(); // SCL=1
		delay();
		i2c_da_1(); // SDA=1
		delay();
	}

	void i2c_repeat()
	{
		i2c_cl_1(); // SCL=1
		delay();
		i2c_da_0(); // SDA=0
		delay();
		i2c_cl_0(); // SCL=0
		delay();
	}

	bool i2c_write(int c)
	{
		int i;
		bool nack;

		delay();

		// foreach of the 8 bits
		for (i = 0; i < 8; i++) {
			if (c & 0x80) {
				i2c_da_1();                   // SCL=1
			} else {
				i2c_da_0();                   // SCL=0
			}
			c <<= 1;
			delay();
			i2c_cl_1();                       // SCL=1
			delay();
			i2c_cl_0();                       // SCL=0
			delay();
		}

		i2c_da_1();                           // SDA=1
		delay();

		i2c_cl_1();                           // SCL=1
		delay();
		// NACK
		nack = i2c_get_da();
		i2c_cl_0();                           // SCL=0

		return nack;
	}

	int i2c_read(bool nack)
	{
		int i, c;

		i2c_da_1();                          // SDA=1
		delay();

		c = 0;

		for (i = 0; i < 8; i++) {
			i2c_cl_1();                     // SCL=1
			delay();
			c <<= 1;
			if (i2c_get_da()) {             // SDA value
				c |= 1;
			}
			i2c_cl_0();                     // SCL=0
			delay();
		}

		// NACK
		if (nack) {
			i2c_da_1();                     // SDA=1
		} else {
			i2c_da_0();                     // SDA=0
		}
		delay();
		i2c_cl_1();                         // SCL=1
		delay();
		i2c_cl_0();                         // SCL=0
		delay();

		return c;
	}

	int address;                            // I2C address
	int address2;                           // I2C second address
	
public:
	I2C(int address, int address2)          // allows 2 ic connedted on this i2c bus
		: address(address),
		: address2(address2),
	{
		init_i2c();
	}

	virtual void write(int reg, int data)
	{
		i2c_start();                  
		i2c_write(address << 1);      
		i2c_write(reg);               
		i2c_write(data);              
		i2c_stop();                   
	}
	virtual int read(int reg)
	{
		int data;
		i2c_start();                   
		i2c_write(address << 1);       
		i2c_write(reg);                
		i2c_repeat();                  
		i2c_write((address << 1) | 1); 
		data = i2c_read(true);         
		i2c_stop();                    
		return data;
	}
	
	virtual void write2(int reg, int data)
	{
		i2c_start();                  
		i2c_write(address2 << 1);      
		i2c_write(reg);               
		i2c_write(data);              
		i2c_stop();                   
	}
	virtual int read2(int reg)
	{
		int data;
		i2c_start();                   
		i2c_write(address2 << 1);       
		i2c_write(reg);                
		i2c_repeat();                  
		i2c_write((address2 << 1) | 1); 
		data = i2c_read(true);         
		i2c_stop();                    
		return data;
	}
};

// create instance pointer of the I2C class called wire 
I2C *wire;

// read 16 bit temperature device (adt7140)
double read_adt7140() {
	
	char registers[2] = { 0x00, 0x01 };                                    // Register Address for Temperature MSB and LSB
	char RecvBuffer[2] = { 0x00, 0x00 };
	int result = 0;
    double atmp = 0;
	for(int idx=0; idx < 2; idx++) {                                       // enable repeated start condition
		RecvBuffer[idx] = wire->read(registers[idx]);
		result |= RecvBuffer[idx];
		if (idx == 0) { 
			result <<= 8;
		}
		usleep(100);
	}
	result >>= 3; 

	atmp = (float)result / 16; 
	//printf("Temp=%f\n", atmp);
	return atmp;
}

// functions for I2C access reading/writing the EEPROM 24FC1025 is shown below.
unsigned int read_24fc1025( unsigned char* registers ) {

	char RecvBuffer[2];
	unsigned int result = 0;
    double atmp = 0;
	for(int idx=0; idx < 2; idx++) {                                       
		RecvBuffer[idx] = wire->read2(*registers[idx]);
		result |= RecvBuffer[idx];
		if (idx == 0) { 
			result <<= 8;
		}
		usleep(100);
	}
	return result;
}

void write_24fc1025( unsigned char* registers, unsigned char* dataV, unsigned int datalen ) {

	char RecvBuffer[datalen];
	unsigned int result = 0;
    double atmp = 0;
	for(int idx=0; idx < 2; idx++) {                                       // write memory address
		wire->write2(*registers[idx]);
		usleep(100);
	}
	for(int idx=0; idx < datalen; idx++) {                                 // write data
		wire->write2(*dataV[idx]);
		usleep(100);
	}
}

float read_24fc1025_real( unsigned char* registers ) {

	char RecvBuffer[4];
    double atmp = 0;
	union { float f; unsigned int i; } t;
	for(int idx=0; idx < 4; idx++) {                                       
		RecvBuffer[idx] = wire->read2(*registers[idx]);			
		usleep(100);
	}
	t.i = RecvBuffer[0] << 24 | RecvBuffer[1] << 16 | RecvBuffer[2] << 8 | RecvBuffer[3];
	return t.f;
}

void setup()
{
	bcm2835_init();
	// set if you need to change the clock                     bcm2835_i2c_setClockDivider(IIC_SCLK_RATE);
	wire = new I2C(ADT7410_SLAVE_ADDR, 24FC1025_SLAVE_ADDR);	
}

i2cData_t loop()
{
	i2cData_t i2_values;
    double v = read_adt7140();
	i2_values.Temperature = v;
	unsigned char registers[2] = { 0x00, 0x00 };                           // memory address MSB and LSB
	unsigned int ii = read_24fc1025(&registers);
	i2_values.SetPoint = ii;
	registers[2] = { 0x00, 0x04 };                                        // memory address MSB and LSB
	ii = read_24fc1025(&registers);
	i2_values.MemState1 = ii;
	registers[2] = { 0x00, 0x08 };                                         // memory storing a float number	
	i2_values.MemReal1 = read_24fc1025_real(&registers);
	return i2_values;
}

int main()
{
    setup();
    unsigned char dataArr[2] = { 0x1, 0x1 };                               // data to write to memeory 1
	unsigned char registers[2] = { 0x00, 0x00 };                           // memory address MSB and LSB
    write_24fc1025( &registers, &dataArr, 2 );
    dataArr[2] = { 0x0F, 0xF0 };                                           // data to write to memeory 2
	registers[2] = { 0x00, 0x04 };                                         // memory address MSB and LSB
    write_24fc1025( &registers, &dataArr, 2 );
	registers[2] = { 0x00, 0x08 };                                         // memory register 3 where we will store a float number
    union { float f; int i; } a;
    a.f = -2.5f;	                                                       // write a float value to memory
    printf( "float %X\n", ( a.i >> 24 ) & 0xFF );
    printf( "float %X\n", ( a.i >> 16 ) & 0xFF );
    printf( "float %X\n", ( a.i >> 8 ) & 0xFF );
    printf( "float %X\n", ( a.i ) & 0xFF );
    unsigned char dataArr2[4] = { ( a.i >> 24 ) & 0xFF, ( a.i >> 16 ) & 0xFF, ( a.i >> 8 ) & 0xFF, ( a.i ) & 0xFF};       // data to write to memeory 3	
    write_24fc1025( &registers, &dataArr2, 4 );
	while (1) {
		i2cData_t vals=loop();
		printf("Temp=%f Spt=%d state=%d realV=%f\n", vals.Temperature, vals.SetPoint, vals.MemState1, vals.MemReal1);
	}

	return 0;
}


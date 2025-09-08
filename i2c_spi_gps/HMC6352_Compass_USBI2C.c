/***************************************************************************************
* Linux example code for the Honeywell HMC6352 (2-axis, I2C)                           * 
* Digital Compass and USBI2C	                                                       *
* ref :- https://www.robot-electronics.co.uk/products/usb-to-i2c-interface-module.html *
* https://www.robot-electronics.co.uk/htm/linux_examples.htm                           *
*						                                                               *
*Compliles using gcc.		                                                           *
*Tested on Ubuntu 16.04 LTS.			                                               *
*						                                                               *
*Opens a port to communicate with the USBI2C and                                       *
*reads the Honeywell HMC6352 and bearing data.		                                   *				                                                           *
*						                                                               *
*By James Henderson, 2016. Modified for HMC6352 Compass by ACP	                       *
****************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/types.h>

#define I2CDLAY 6000
#define USBPORT "/dev/ttyUSB0"             // Port which USB-I2C is attached to
#define P_ERROR	-1						   // to check for errors when dealing with the port

// function prototypes
int openPort(void);
void closePort(int fd);
void writeData(int fd, int nbytes);
void readData(int fd, int nbytes);
struct termios options;

// global data buffer 
unsigned char sbuf[10];						// Stores data to be read and written

int main(int argc, char *argv[])
{
    int fd;								    // file descriptor of open port
    float azimuth = 0.0;                    // variable to store azimuth returned from the compass
	fd = openPort();						// Open port to USBi2C	
	
	if(fd > P_ERROR)						// Only do the folloing if there has not been an error opening the port
	{
		sbuf[0] = 0x5A;	                    // This brings back the Firmware Revision Number for the USB-I2C interface	                				
		sbuf[1] = 0x01;	                    													
		writeData(fd, 2);					// Write this data to the USBI2C

		readData(fd, 3);					// Read back the returned byte into sbuf[0] this should be non 0 for success

		usleep(I2CDLAY);				    // Wait 
		
		if(!sbuf[0])						// If it is 0 report an error
		{
			printf("USBI2C error writing to HMC6352");
		} 
		else
		{
			printf("USBI2C HMC6352 Module ID: %d rev %d", sbuf[0], sbuf[1]);
        }
		
		sbuf[0] = 0x53;		                // USB-I2C (I2C_SGL): 1				
		sbuf[1] = 0x42;	                    // HMC6352 I2C address: 0x42 + 0(Write))					
		sbuf[2] = 0x41;	                    // 'A'									
		writeData(fd, 3);					// Write this data to the USBI2C

		readData(fd, 1);					// Read back the returned byte into sbuf[0] this should be non 0 for success

		if(!sbuf[0])						// If it is 0 report an error
		{
			printf("USBI2C error writing to HMC6352");
		}	

		usleep(I2CDLAY);						// Wait for the set-up to finish
		
		sbuf[0] = 0x54;						// USB-I2C (I2C-MUL)
		sbuf[1] = 0x43;		                // HMC6352 I2C address: 0x42 7bit +1(Read)				
		sbuf[2] = 0x02;											
		writeData(fd, 3);					// Write this data to the USBI2C
		
		readData(fd, 3);					                        // Read back the 3 bytes as requested
		
		azimuth = (float)(((sbuf[2] << 8) + sbuf[3]))/10.0;		    // Calculate azimuth from high and low bytes
		
		printf("HMC6352 SUCCESS: %u\n", sbuf[0]);			        // Display the data to the screen
		printf("Azimuth : %f\n\n", azimuth);

        closePort(fd);
	}
	return 0;
}

// opens USB Port
int openPort(void)
{
    int fd;										        // File descriptor for the port
	fd = open(USBPORT, O_RDWR | O_NOCTTY );				// Open port for read and write not making it a controlling terminal
	if (fd == P_ERROR)
	{
		perror("openPort: Unable to open /dev/ttyUSB0 - ");		// If open() returns an error
	} 
	else
	{
		tcgetattr(fd, &options);
		cfmakeraw(&options);
		cfsetispeed(&options, B19200);					// Set the baud rates to 19200
		options.c_cc[VMIN]  = 4;
		options.c_cc[VTIME] = 50; 
		tcsetattr(fd, TCSANOW, &options);				// Set the new options for the port
	}
	return (fd);
}

// closes USB Port
void closePort(int fd)
{	
	if(close(fd) == P_ERROR)						// Close the port if it returns an error then display an error report
	{	
		perror("closePort: Unable to close /dev/ttyUSB0 - ");
	}
}

// write data
void writeData(int fd, int nbytes)
{
    int bytes;
	bytes = write(fd, sbuf, nbytes);			    // Write nbytes of data from wbuf
	if(bytes == P_ERROR)							// If write returns an error (-1)
	{
		perror("writeData: Error while trying to write data - ");	
	}
	else if(bytes != nbytes)
	{
		printf("only %u bytes written out of %u requested\n", bytes, nbytes);	
	}

}

// read data
void readData(int fd, int nbytes)
{
    int bytes;
	bytes = read(fd, sbuf, nbytes);						// Read nbytes of data into rbuf
	if(bytes == P_ERROR)							    // If read returns and error (-1)
	{
		perror("readData: Error while trying to read data - ");
	}
	else if(bytes != nbytes)
	{
		printf("Only %u bytes read out of %u requested\n\n", bytes, nbytes);
	}
}
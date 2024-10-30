#ifndef __I2C_DEV_WRAP_H
#define __I2C_DEV_WRAP_H

/* -----------------------------------------------------------------------------
 If you are using a Raspberry Pi, please note that the Raspberry Pi’s hardware I²C module 
 has a bug that causes this code to not work reliably. 
 As a workaround, we recommend enabling the i2c-gpio overlay and using the I²C device 
 that it provides. To do this, add the line dtoverlay=i2c-gpio to /boot/config.txt and reboot. 
 The overlay documentation has information about the parameters you can put on that line,
 but those parameters are not required. 
 Connect the i2c devices SDA line to GPIO23 and connect the Tic’s SCL line to GPIO24. 
 The i2c-gpio overlay creates a new I²C device which is usually named /dev/i2c-3, and the 
 code below uses that device. To give your user permission to 
 access I²C busses without being root, you might have to add yourself to the i2c group by 
 running sudo usermod -a -G i2c $(whoami) and restarting.
------------------------------------------------------------------------------*/

/* -----------------------------------------------------------------------------
 Include
------------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

typedef char C08;
typedef unsigned char BOOL;
typedef unsigned char U08;
typedef unsigned short U16;
typedef unsigned long U32;
typedef unsigned long long U64;
typedef char S08;
typedef short S16;
typedef long S32;
typedef long long S64;
typedef float F32;
typedef double D64;

/* -----------------------------------------------------------------------------
 Define
------------------------------------------------------------------------------*/
#define I2C_DEVICE ("/dev/i2c-1")               // or which ever device driver you are using
#define FD_INIT_VAL (0xFFFFFFFF)

/* -----------------------------------------------------------------------------
 Global
------------------------------------------------------------------------------*/
static S32 _fd = FD_INIT_VAL;
static S32 _pca_fd = FD_INIT_VAL;

/* -----------------------------------------------------------------------------
 Function   : I2C Init
 Memo       : I2C initialise
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
BOOL I2cCtl_Init(U08 bus_no)
{
    BOOL status = true;
    char dev_path[16] {};
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", bus_no);
	
    /* open if not already open */
    if (_fd == (S32)FD_INIT_VAL)
    {
        _fd = open(dev_path, O_RDWR);
        if (_fd < 0)
        {
            perror(dev_path);
            status = false;
        }
    }

    return status;
}

/* -----------------------------------------------------------------------------
 Function   : I2cCtl_Close
 Memo       : I2C close
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
void I2cCtl_Close()
{
    close(_fd);	
}

/* -----------------------------------------------------------------------------
 Function   : I2C PcaInit
 Memo       : I2C initialise for Pca
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
BOOL I2cCtl_PcaInit(U08 bus_no)
{
    BOOL status = true;
    char dev_path[16] {};
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", bus_no);
	
    /* open if not already open */
    if (_pca_fd == (S32)FD_INIT_VAL)
    {
        _pca_fd = open(dev_path, O_RDWR);
        if (_pca_fd < 0)
        {
            perror(dev_path);
            status = false;
        }
    }

    return status;
}

/* -----------------------------------------------------------------------------
 Function   : I2cCtl_PcaClose for pca
 Memo       : I2C close
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
void I2cCtl_PcaClose()
{
    close(_pca_fd);	
}

/* -----------------------------------------------------------------------------
 Function   : I2C Write
 Memo       : write to device on i2c
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
BOOL I2cCtl_Write(U08 dev_adr, void *buf, U32 buf_length, u08 bus_no)
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data packets;
    S32 ret;
    BOOL status = true;
	char dev_path[16] {};
	snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", bus_no);
	
    /* make i2c message */
    msg.addr = dev_adr;
    msg.flags = 0;
    msg.len = buf_length;
    msg.buf = buf;

    /* Send i2c packet */
    packets.msgs = &msg;
    packets.nmsgs = 1; 
    ret = ioctl(_fd, I2C_RDWR, &packets);
    if(ret < 0)
    {
        perror(dev_path);
        status = false;
    }

    return status;
}

/* -----------------------------------------------------------------------------
 Function   : I2C Read
 Memo       : read device on i2c
 Date       : 2021.08.28
------------------------------------------------------------------------------*/
BOOL I2cCtl_Read(U08 dev_adr, void *buf, U32 buf_length, U08 bus_no)
{
    struct i2c_msg msg;
    struct i2c_rdwr_ioctl_data packets;
    S32 ret;
    BOOL status = true;
	char dev_path[16] {};
	snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", bus_no);
	
     /* (Data Read) */
    msg.addr = dev_adr;
    msg.flags = I2C_M_RD;
    msg.len = buf_length;
    msg.buf = buf;

    packets.msgs = &msg;
    packets.nmsgs = 1; 
    ret = ioctl(_fd, I2C_RDWR, &packets);
    if(ret < 0)
    {
        perror(dev_path);
        status = false;
    }

    return status;
}



#endif


#ifndef __I2C_DEV_WRAP_H
#define __I2C_DEV_WRAP_H

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
BOOL I2cCtl_Close()
{
    close(_fd);	
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


#include "pca9685.h"

/* -----------------------------------------------------------------------------
 Global
------------------------------------------------------------------------------*/
#define FD_INIT_VAL (0xFFFFFFFF)

namespace pwm_obj {

const int RESOLUTION = 4096;
const uint8_t LED_ONOFF_START_ADDRESS = 0x08;
const uint8_t NUM_REGISTERS_PER_CHANNEL = 4;
const uint8_t _retries = 0;
const uint8_t _ERROR -1

PCA9685::PCA9685(uint32_t period_us, uint8_t adapter_number, uint8_t device_address, uint32_t oscillator_frequency)
 :period_us(period_us),
 _adapter_number(adapter_number),
 _device_address(device_address),
 _oscillator_frequency(oscillator_frequency)
{
    //int freq = std::round(static_cast<double>(1000 * 1000) / period_us);
    //int8_t pre_scale = static_cast<uint8_t>(std::round((oscillator_frequency / (RESOLUTION * freq)) - 1));

    //set PRE_SCALE
    //device.write(0xFE, pre_scale);
	I2cCtl_PcaInit(_adapter_number);                     // open the device driver
}

int PCA9685::close( )
{
    I2cCtl_PcaClose();
}

int PCA9685::init( )
{
	
    int freq = std::round(static_cast<double>(1000 * 1000) / period_us);
    uint8_t pre_scale = static_cast<uint8_t>(std::round((_oscillator_frequency / (RESOLUTION * freq)) - 1));
	
	int ret = _ERROR;
	uint8_t retry_count = 0;

	if (_pca_fd < 0) {
		return _ERROR;
	}

	do {

		uint8_t msgs = 0;
		struct i2c_msg msgv;                                     // sends to messages to the PWM chip
        uint8_t bb[2];                                           // bytes to send to the device
		
        bb[0] = 0xFE;
        bb[0] = pre_scale;
		
        msgv.addr = _device_address;
        msgv.flags = 0;
        msgv.buf = &bb;                                         // set PRE_SCALE
        msgv.len = 2;

		struct i2c_rdwr_ioctl_data packets;
		packets.msgs  = &msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_pca_fd, I2C_RDWR, &packets);

		if (ret_ioctl == -1) {
			ret = _ERROR;

		} else {
			// success
			ret = 1;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}

int PCA9685::start()
{
	int ret = _ERROR;
	uint8_t retry_count = 0;

	if (_pca_fd < 0) {
		return _ERROR;
	}

	do {

		uint8_t msgs = 0;
		struct i2c_msg msgv;                                     // sends to messages to the PWM chip
        uint8_t bb[2];                                                  // bytes to send to the device
		
        bb[0] = 0x00;
        bb[0] = 0x01;
		
        msgv.addr = _device_address;
        msgv.flags = 0;
        msgv.buf = &bb;                                         // turn off SLEEP bit on MODE1
        msgv.len = 2;

		struct i2c_rdwr_ioctl_data packets;
		packets.msgs  = &msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_pca_fd, I2C_RDWR, &packets);

		if (ret_ioctl == -1) {
			ret = _ERROR;

		} else {
			// success
			ret = 1;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}


int PCA9685::reset()
{
	int ret = _ERROR;
	uint8_t retry_count = 0;

	if (_pca_fd < 0) {
		return _ERROR;
	}

	do {

		uint8_t msgs = 0;
		struct i2c_msg msgv;                                     // sends to messages to the PWM chip
        uint8_t bb[2];                                                  // bytes to send to the device
		
        bb[0] = 0x00;
        bb[0] = 0x91;
        
        msgv.addr = _device_address;
        msgv.flags = 0;
        msgv.buf = &bb;                                         // turn on RESTART bit on MODE1
        msgv.len = 2;

		struct i2c_rdwr_ioctl_data packets;
		packets.msgs  = &msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_pca_fd, I2C_RDWR, &packets);

		if (ret_ioctl == -1) {
			ret = _ERROR;

		} else {
			// success
			ret = 1;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}

// multiple message write
int PCA9685::set_pulse(uint8_t channel, uint32_t width_us)
{
	int ret = _ERROR;
	uint8_t retry_count = 0;

	if (_pca_fd < 0) {
		return _ERROR;
	}

    float duty_rate = width_us/static_cast<double>(period_us);
    uint16_t reg_value = static_cast<uint16_t>(std::round(RESOLUTION * duty_rate - 1));
    uint8_t register_address = LED_ONOFF_START_ADDRESS + (NUM_REGISTERS_PER_CHANNEL * channel);

	do {

		uint8_t msgs = 0;
		struct i2c_msg msgv[2] {};                                      // sends to messages to the PWM chip
        uint8_t bb[2];                                                  // bytes to send to the device
		
        bb[0] = register_address;
        bb[0] = reg_value & 0xFF;
		
        msgv[msgs].addr = _device_address;
        msgv[msgs].flags = 0;
        msgv[msgs].buf = &bb;                                          // set LED{channel}_OFF_L
        msgv[msgs].len = 2;
        msgs++;

        bb[0] = register_address + 1;
        bb[0] = reg_value >> 8;
		
        msgv[msgs].addr = _device_address;
        msgv[msgs].flags = 0;
        msgv[msgs].buf = &bb;                                          // set LED{channel}_OFF_H
        msgv[msgs].len = 2;
        msgs++;

		struct i2c_rdwr_ioctl_data packets{};
		packets.msgs  = &msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_pca_fd, I2C_RDWR, &packets);

		if (ret_ioctl == -1) {
			ret = _ERROR;
		} else {
			// success
			ret = 1;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}

int PCA9685::read(uint8_t register_address, uint8_t &value)
{
	int ret = _ERROR;
	uint8_t retry_count = 0;

	if (_pca_fd < 0) {
		return _ERROR;
	}

	do {

		uint8_t msgs = 0;
		struct i2c_msg msgv[2] {};                                      // sends to messages to the PWM chip
		
        bb[0] = register_address;
        bb[0] = reg_value & 0xFF;
		
        msgv[msgs].addr = _device_address;
        msgv[msgs].flags = 0;
        msgv[msgs].buf = &register_address;                              // set register to read
        msgv[msgs].len = 1;
        msgs++;
		
        msgv[msgs].addr = _device_address;
        msgv[msgs].flags = I2C_M_RD;
        msgv[msgs].buf = &value;                                          // set LED{channel}_OFF_H
        msgv[msgs].len = 1;
        msgs++;

		struct i2c_rdwr_ioctl_data packets{};
		packets.msgs  = &msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_pca_fd, I2C_RDWR, &packets);

		if (ret_ioctl == -1) {
			ret = _ERROR;
		} else {
			// success
			ret = 1;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}

}//namespace pwm_obj
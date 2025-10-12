/*
 *
 * 	Demonstrates using Electronut Labs hackaBLE Nordic nRF52832 dev board 
 *  with Zephy RTOS.
 * 
 * 	ref:- https://github.com/electronut/ElectronutLabs-hackaBLE/blob/master/code/air_quality_monitor/main.c
 * 
 */
#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <gpio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/byteorder.h>
#include <sensor.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"

#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "stdlib.h"
#include "time.h"
#include <nrf_drv_twi.h>

// clean robot set-up comment to not use 
#define _CLEAN_ROBOT

// ref:- https://github.com/arduino-libraries/MadgwickAHRS 
#include <MadgwickAHRS.h>

// global madgewick filter
Madgwick MadgwickFilter;

// i2c driver instance 
const nrf_drv_twi_t p_twi_sensors = NRF_DRV_TWI_INSTANCE(0);

// define which type of nRf board we are using
#define STD_CONF
// define i2c physical connection
#if defined(STD_CONF)
#define SCL_PIN 10
#define SDA_PIN 9
#elif defined(AONODON20191113)
// aonodon20181113 
#define SCL_PIN 30
#define SDA_PIN 31
#elif defined(AONODON20190303)
// aonodon20190303
#define SCL_PIN 27
#define SDA_PIN 26
#elif defined(AONODON2019-GROVE)
// aonodon2019-grove
# define SCL_PIN 5
# define SDA_PIN 7
#endif

// device address
#define DEVICE_ADDR    0x6BU
#define MEASURING_FREQ (1660)

/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_TEST_PAGE  			0X00
#define LSM6DS3_ACC_GYRO_RAM_ACCESS  			0X01
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME  		0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN  		0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1  			0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2  			0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3  			0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4  			0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5  			0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G  			0X0B
#define LSM6DS3_ACC_GYRO_REFERENCE_G  			0X0C
#define LSM6DS3_ACC_GYRO_INT1_CTRL  			0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL  			0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG  			0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL  			0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G  			0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C  			0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C  			0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C  			0X14
#define LSM6DS3_ACC_GYRO_CTRL6_G  			0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  			0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  			0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  			0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  			0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  		0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  			0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  			0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  			0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  			0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L  			0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H  			0X21
#define LSM6DS3_ACC_GYRO_OUTX_L_G  			0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G  			0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G  			0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G  			0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G  			0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G  			0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL  			0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL  			0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL  			0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL  			0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL  			0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL  			0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  		0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  		0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  		0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  		0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  		0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  		0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  		0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  		0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  		0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  		0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  		0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  		0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  			0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  			0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  			0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  			0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  		0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  		0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  		0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  		0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  		0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  		0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  		0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC  			0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1  			0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  			0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  			0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  			0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  			0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  			0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  			0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  			0X5F

/************** Access Device RAM  *******************/
#define LSM6DS3_ACC_GYRO_ADDR0_TO_RW_RAM         0x62
#define LSM6DS3_ACC_GYRO_ADDR1_TO_RW_RAM         0x63
#define LSM6DS3_ACC_GYRO_DATA_TO_WR_RAM          0x64
#define LSM6DS3_ACC_GYRO_DATA_RD_FROM_RAM        0x65

#define LSM6DS3_ACC_GYRO_RAM_SIZE                4096

/************** Embedded functions register mapping  *******************/
#define LSM6DS3_ACC_GYRO_SLV0_ADD                     0x02
#define LSM6DS3_ACC_GYRO_SLV0_SUBADD                  0x03
#define LSM6DS3_ACC_GYRO_SLAVE0_CONFIG                0x04
#define LSM6DS3_ACC_GYRO_SLV1_ADD                     0x05
#define LSM6DS3_ACC_GYRO_SLV1_SUBADD                  0x06
#define LSM6DS3_ACC_GYRO_SLAVE1_CONFIG                0x07
#define LSM6DS3_ACC_GYRO_SLV2_ADD                     0x08
#define LSM6DS3_ACC_GYRO_SLV2_SUBADD                  0x09
#define LSM6DS3_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define LSM6DS3_ACC_GYRO_SLV3_ADD                     0x0B
#define LSM6DS3_ACC_GYRO_SLV3_SUBADD                  0x0C
#define LSM6DS3_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define LSM6DS3_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_IIR              0x10
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_ACOS             0x11
#define LSM6DS3_ACC_GYRO_CONFIG_TILT_WTIME            0x12
#define LSM6DS3_ACC_GYRO_SM_STEP_THS                  0x13
#define LSM6DS3_ACC_GYRO_MAG_SI_XX                    0x24
#define LSM6DS3_ACC_GYRO_MAG_SI_XY                    0x25
#define LSM6DS3_ACC_GYRO_MAG_SI_XZ                    0x26
#define LSM6DS3_ACC_GYRO_MAG_SI_YX                    0x27
#define LSM6DS3_ACC_GYRO_MAG_SI_YY                    0x28
#define LSM6DS3_ACC_GYRO_MAG_SI_YZ                    0x29
#define LSM6DS3_ACC_GYRO_MAG_SI_ZX                    0x2A
#define LSM6DS3_ACC_GYRO_MAG_SI_ZY                    0x2B
#define LSM6DS3_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define LSM6DS3_ACC_GYRO_MAG_OFFX_L                   0x2D
#define LSM6DS3_ACC_GYRO_MAG_OFFX_H                   0x2E
#define LSM6DS3_ACC_GYRO_MAG_OFFY_L                   0x2F
#define LSM6DS3_ACC_GYRO_MAG_OFFY_H                   0x30
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_L                   0x31
#define LSM6DS3_ACC_GYRO_MAG_OFFZ_H                   0x32

//This struct holds the settings the driver uses to do calculations
typedef struct _SensorSettings {
	// Gyro settings
	uint8_t gyroEnabled;
	uint16_t gyroRange;
	uint16_t gyroSampleRate;
	uint16_t gyroBandWidth;

	uint8_t gyroFifoEnabled;
	uint8_t gyroFifoDecimation;

	// Accelerometer settings
	uint8_t accelEnabled;
	uint8_t accelODROff;
	uint16_t accelRange;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;
	
	uint8_t accelFifoEnabled;
	uint8_t accelFifoDecimation;
	
	// Temperature settings
	uint8_t tempEnabled;
	
	// Non-basic mode settings
	uint8_t commMode;
	
	// FIFO control data
	uint16_t fifoThreshold;
	int16_t fifoSampleRate;
	uint8_t fifoModeWord;
	
};

typedef struct pos3d_t {
  float x;
  float y;
  float z;
};

double mes_time_ = 1.0 / (double) MEASURING_FREQ * 1000.0;

struct LSM6DS3DATASTRUCT{
    pos3d_t gyr;
    pos3d_t acc;
    pos3d_t ang;
};

void set_up_sensor( _SensorSettings *s) {
   s->gyroEnabled = 1;
   s->gyroRange = 2000;
   s->gyroSampleRate = 208;
   s->accelEnabled = 1;
   s->accelBandWidth = 400;
   s->accelRange = 4; 	
   s->accelSampleRate = 208;
}

#if defined(NON_BLOCK)
static volatile bool m_xfer_done = false;
static uint8_t m_sample;
__STATIC_INLINE void data_handler(uint8_t data)
{
    NRF_LOG_INFO("Device Current Register Data: %d", data);
}
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
#endif

void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
#if defined(NON_BLOCK)
    err_code = nrf_drv_twi_init(&p_twi_sensors, &twi_lm75b_config, twi_handler, NULL);
#else
    err_code = nrf_drv_twi_init(&p_twi_sensors, &twi_lm75b_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
    nrf_drv_twi_enable(&p_twi_sensors);
}

ret_code_t write_registers(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t *reg_data, uint8_t nb, bool no_stop)
{
  uint8_t tx_data[nb];
  tx_data[0] = register_addr;
  for (i=0; i<nb; i++) {
      tx_data[i+1] = *reg_data[i];
  }
  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, tx_data, sizeof(tx_data), no_stop);
  if(err_code != NRF_SUCCESS) {
    printk("error writing to register %d", register_addr);
  }
  return err_code;
}

ret_code_t write_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t reg_data, bool no_stop)
{
  uint8_t tx_data[2];
  tx_data[0] = register_addr;
  tx_data[1] = reg_data;
  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, tx_data, sizeof(tx_data), no_stop);
  if(err_code != NRF_SUCCESS) {
    printk("error writing to register %d", register_addr);
  }
  return err_code;
}

ret_code_t read_register(nrf_drv_twi_t twi_instance, uint8_t device_addr, uint8_t register_addr, uint8_t *p_data, uint8_t bytes, bool no_stop)
{
  ret_code_t err_code;

  err_code = nrf_drv_twi_tx(&twi_instance, device_addr, &register_addr, 1, no_stop);
  APP_ERROR_CHECK(err_code);

  if(err_code != NRF_SUCCESS) {
    printk("error writing to register %d", register_addr);
  }

  err_code = nrf_drv_twi_rx(&twi_instance, device_addr, p_data, bytes);
  return err_code;
}

//****************************************************************************//
//
//  Gyro section
//
//****************************************************************************//
//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
ret_code_t readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
    ret_code_t err_code;
	uint8_t myBuffer[2];
    err_code= read_register(p_twi_sensors, DEVICE_ADDR, offset,  myBuffer, sizeof(myBuffer), false);
    APP_ERROR_CHECK(err_code);
	*outputPointer = ((int16_t)(myBuffer[0]) | (int16_t)(myBuffer[1]<< 8));

	return err_code;
}
int16_t readRawGyroX( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );
    APP_ERROR_CHECK(err_code);
	return output;
}

int16_t readRawGyroY( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );
    APP_ERROR_CHECK(err_code);
	return output;
}

int16_t readRawGyroZ( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );
    APP_ERROR_CHECK(err_code);
	return output;
}

float calcGyro( int16_t input, _SensorSettings *s )
{
    float output;
	uint8_t gyroRangeDivisor = s->gyroRange / 125;
	if ( s->gyroRange == 245 ) {
		gyroRangeDivisor = 2;
	}

	output = (float)input * 4.375f * (gyroRangeDivisor) / 1000;
#if defined(_CLEAN_ROBOT)                                                                            //for clean robot
	output = (float)input*8.75f/1000;
#endif
	return output;
}

float readFloatGyroX( _SensorSettings *s )
{
	float output = calcGyro(readRawGyroX(), s);
	return output;
}

float readFloatGyroY( _SensorSettings *s )
{
	float output = calcGyro(readRawGyroY(), s);
	return output;
}

float readFloatGyroZ( _SensorSettings *s )
{
	float output = calcGyro(readRawGyroZ(), s);
	return output;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t readRawAccelX( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );
    APP_ERROR_CHECK(err_code);
	return output;
}

int16_t readRawAccelY( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );
    APP_ERROR_CHECK(err_code);
	return output;
}

int16_t readRawAccelZ( void )
{
	int16_t output;
	ret_code_t err_code = readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
    APP_ERROR_CHECK(err_code);
	return output;
}

float calcAccel( int16_t input,  _SensorSettings *s)
{
	float output = (float)input * 0.061f * (s->accelRange >> 1) / 1000;
	return output;
}

float readFloatAccelY( _SensorSettings *s )
{
	float output = calcAccel(readRawAccelY(), s);
	return output;
}

float readFloatAccelZ( _SensorSettings *s )
{
	float output = calcAccel(readRawAccelZ(), s);
	return output;
}

float readFloatAccelX( _SensorSettings *s )
{
	float output = calcAccel(readRawAccelX(), s);
	return output;
}

// periodic fetch data from the sensor 
struct LSM6DS3DATASTRUCT readLSM6DS3Data( _SensorSettings *s )
{
  ret_code_t err_code;
  int i = 0;

  uint8_t tx_data[1];
  tx_data[0] = 0xF7;
  uint8_t p_data[8];
  struct LSM6DS3DATASTRUCT Data;

  Data.gyr.x = readFloatGyroX(s);
  Data.gyr.y = readFloatGyroY(s);
  Data.gyr.z = readFloatGyroZ(s);
   
  Data.acc.x = readFloatAccelX(s);
  Data.acc.y = readFloatAccelY(s);
  Data.acc.z = readFloatAccelZ(s);

  MadgwickFilter.updateIMU(Data.gyr.x, Data.gyr.y, Data.gyr.z, Data.acc.x, Data.acc.y, Data.acc.z);
  Data.ang.x = MadgwickFilter.getRoll();
  Data.ang.y = MadgwickFilter.getPitch();
  Data.ang.z = MadgwickFilter.getYaw();

  return Data;
}

//****************************************************************************//
//
//  Temperature section
//
//****************************************************************************//
int16_t readRawTemp( void )
{
	int16_t output;
	readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUT_TEMP_L );
	return output;
}  

float readTempC( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset

	return output;

}

float readTempF( void )
{
	float output = (float)readRawTemp() / 16; //divide by 16 to scale
	output += 25; //Add 25 degrees to remove offset
	output = (output * 9) / 5 + 32;

	return output;
}

ret_code_t setup_i2c( _SensorSettings *s )
{
    uint8_t result;
	uint8_t dataToWrite = 0;  
    ret_code_t err_code;

    twi_init();                        // init the i2c
    MadgwickFilter.begin(100);         // create madgewick filter AHRS object @ 100Hz update

	//   Setup the accelerometer     ******************************
	dataToWrite = 0;                                             
	if ( s->accelEnabled == 1) {
		switch (s->accelBandWidth) {
		case 50:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
			break;
		case 100:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
			break;
		case 200:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
			break;
		default:  // set default case to max passthrough
		case 400:
			dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
			break;
		}
		// Next, patch in full scale
		switch (s->accelRange) {
		case 2:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
			break;
		case 4:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
			break;
		case 8:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
			break;
		default:  // set default case to 16(max)
		case 16:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
			break;
		}
		// Lastly, patch in accelerometer ODR
		switch (s->accelSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
			break;
		case 3330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
			break;
		case 6660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
			break;
		case 13330:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}

	// Now, write the patched together data
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite, false);
    APP_ERROR_CHECK(err_code);

	// Set the ODR bit
	//readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
    read_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL4_C,  &dataToWrite, sizeof(dataToWrite), false);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	if ( s->accelODROff == 1) {
		dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	}
	// writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite, false);
    APP_ERROR_CHECK(err_code);

	// Setup the gyroscope**********************************************
	dataToWrite = 0; 
	if ( s->gyroEnabled == 1) {
		// Build config reg
		// First, patch in full scale
		switch (s->gyroRange) {
		case 125:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
			break;
		case 245:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
			break;
		case 500:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_500dps;
			break;
		case 1000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
			break;
		default:                                             //Default to full 2000DPS range
		case 2000:
			dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
			break;
		}
		// Lastly, patch in gyro ODR
		switch (s->gyroSampleRate) {
		case 13:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
			break;
		case 26:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
			break;
		case 52:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
			break;
		default:  //Set default to 104
		case 104:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
			break;
		case 208:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
			break;
		case 416:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
			break;
		case 833:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
			break;
		case 1660:
			dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
			break;
		}
	}
	else
	{
		//dataToWrite already = 0 (powerdown);
	}
	// Write the byte
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite, false);
    APP_ERROR_CHECK(err_code);
	
	// TBD Setup the internal temperature sensor
	if ( s->tempEnabled == 1) {
	}

	// Return WHO AM I reg  
    err_code =  read_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_WHO_AM_I_REG, &result, sizeof(m_sample), false);              // WHO AM I .. Return 0x69 
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("WHO AM I %d.", result);
	   
#if defined(_CLEAN_ROBOT)                                                         // for clean robot
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x80);                              ODR = 1.66kHz, FS = +/- 245 dps
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL2_G, 0x80, false);
    APP_ERROR_CHECK(err_code);
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, 0x02);                              LPF1 Enable
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL4_C, 0x02, false);
    APP_ERROR_CHECK(err_code);
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x02);                              LPF1 Cutoff = 168Hz
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL6_G, 0x02, false);
    APP_ERROR_CHECK(err_code);
	//writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);                              Gyro High Performance Mode Enable	
    err_code = write_register(p_twi_sensors, DEVICE_ADDR, LSM6DS3_ACC_GYRO_CTRL7_G, 0x00, false);
    APP_ERROR_CHECK(err_code);
#endif
		
	return err_code;
}

#define _USE_LOG
#define ENVIRONMENTAL_SENSING_SERVICE 0x181A
#define P_CHARACTERISTIC 0x2A6F
#define R_CHARACTERISTIC 0x2A6H
#define Y_CHARACTERISTIC 0x2A6D

static struct bt_uuid_16 vnd_uuid = BT_UUID_INIT_16(ENVIRONMENTAL_SENSING_SERVICE);

static const struct bt_uuid_32 P_uuid = BT_UUID_INIT_32(P_CHARACTERISTIC);
static const struct bt_uuid_32 R_uuid = BT_UUID_INIT_32(R_CHARACTERISTIC);
static const struct bt_uuid_32 Y_uuid = BT_UUID_INIT_32(Y_CHARACTERISTIC);

static struct bt_gatt_ccc_cfg P_ccc_cfg[BT_GATT_CCC_MAX];
static struct bt_gatt_ccc_cfg R_ccc_cfg[BT_GATT_CCC_MAX];
static struct bt_gatt_ccc_cfg Y_ccc_cfg[BT_GATT_CCC_MAX];

static u8_t bPNotify;
static u8_t bRNotify;
static u8_t bYNotify;

static void P_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	bPNotify = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static void R_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	bRNotify = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static void Y_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	bYNotify = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

/* Vendor Primary Service Declaration */
static struct bt_gatt_attr vnd_attrs[] = {
	/* Vendor Primary Service Declaration */
	BT_GATT_PRIMARY_SERVICE(&vnd_uuid),
	BT_GATT_CHARACTERISTIC(&P_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(P_ccc_cfg, P_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(&R_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(R_ccc_cfg, R_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(&Y_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(Y_ccc_cfg, Y_ccc_cfg_changed)};

static struct bt_gatt_service vnd_svc = BT_GATT_SERVICE(vnd_attrs);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x1a, 0x18),
};

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err)
	{
		printk("Connection failed (err %u)\n", err);
	}
	else
	{
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_gatt_service_register(&vnd_svc);

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

uint32_t p_field;
uint32_t r_field;
uint32_t y_field;

// uncomment to awap endianess
// #define _SWAP_ENDIAN
// function to swap endian 
static inline uint32_t swapEndian32(uint32_t val) {
    return ((0xFF000000 & val) >> 24) | ((0x00FF0000 & val) >> 8) | ((0x0000FF00 & val) << 8) | ((0x000000FF & val) << 24);
}

// union to convert f32 float to u32 integer for sending 
union f2int
{
  int32_t i32;
  uint32_t u32;
  float f32;
};

// update the data to send over BLE
void update_sensor_data(float pitch, float roll, float yaw)
{
    union f2int f;
#if defined(_SWAP_ENDIAN)
	f.f32=pitch;
	p_field = (uint32_t)swapEndian32(f.u32);
	f.f32=roll;
	r_field = (uint32_t)swapEndian32(f.u32);
	f.f32=yaw;
	y_field = (uint32_t)swapEndian32(f.u32); 
#else
	f.f32=pitch;
	p_field = (uint32_t)(f.u32);
	f.f32=roll;
	r_field = (uint32_t)(f.u32);
	f.f32=yaw;
	y_field = (uint32_t)(f.u32); 
#endif
}

void main(void)
{
	struct device *port0 = device_get_binding("GPIO_0");
	gpio_pin_configure(port0, 17, GPIO_DIR_OUT);                     	/* Set LED pin as output */

	gpio_pin_write(port0, 17, 0);                                       // flash  LED
	k_sleep(500);
	gpio_pin_write(port0, 17, 1);
	k_sleep(500);

	// sensor

	k_sleep(500);
	update_sensor_data(lsm_data.ang.x, lsm_data.ang.y, lsm_data.ang.z);

	// set up BLE
	int err;
	err = bt_enable(bt_ready);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);
#if defined(_USE_LOG)
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
#endif
    _SensorSettings s;
    LSM6DS3DATASTRUCT lsm_data;
    float temperature = 0.0f;
    set_up_sensor(&s)
    ret_code_t rc = setup_i2c(&s);
    APP_ERROR_CHECK(rc);
	while (1)
	{
       lsm_data = readLSM6DS3Data(&s);
	   temperature = readTempC();
#if defined(_USE_LOG)
       NRF_LOG_INFO("ACC %f %f %f ", lsm_data.acc.x, lsm_data.acc.y, lsm_data.acc.z);
       NRF_LOG_FLUSH();
       NRF_LOG_INFO("GYRO %f %f %f ", lsm_data.gyr.x, lsm_data.gyr.y, lsm_data.gyr.z);
       NRF_LOG_FLUSH();
       NRF_LOG_INFO("ANG %f %f %f ", lsm_data.ang.x, lsm_data.ang.y, lsm_data.ang.z);
       NRF_LOG_FLUSH();
       NRF_LOG_INFO("TEMP %f ", temperature);
       NRF_LOG_FLUSH();
#endif
	    update_sensor_data(lsm_data.ang.x, lsm_data.ang.y, lsm_data.ang.z);

		bt_gatt_notify(NULL, &vnd_attrs[2], &p_field, sizeof(p_field));
		bt_gatt_notify(NULL, &vnd_attrs[4], &r_field, sizeof(r_field));
		bt_gatt_notify(NULL, &vnd_attrs[6], &y_field, sizeof(y_field));

		bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
        nrf_delay_ms(5000);
	}
}
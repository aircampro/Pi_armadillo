/*
 *    bme2802.h
 *
 */
#ifndef BME280_H_
#define BME280_H_

#include "xparameters.h"
#include "xiicps.h"
#include "xil_printf.h"
#include "sleep.h"

#define IIC_DEVICE_ID  (XPAR_XIICPS_0_DEVICE_ID)                                  // i2c port can also be XPAR_XIICPS_1_DEVICE_ID

#define BME280_IIC_ADDR				0x77
#define BME280_IIC_SCLK_RATE		100000

#define IIC_SCLK_RATE BME280_IIC_SCLK_RATE
#define IIC_ADDR BME280_IIC_ADDR

// Function Prototypes
int initIicDriver(XIicPs *Iic, u16 DeviceId, u32 FsclHz);
int writeReg(XIicPs *Iic, u8 data);
int IicPsReadInt(XIicPs *Iic, u8 cmd, int *res);
int IicPsReadU8(XIicPs *Iic, u8 cmd, u8 *res);
int readTrim(XIicPs *Iic);
void setup(XIicPs *Iic);
int readData(XIicPs *Iic);
signed long int calibration_T(signed long int adc_T);
unsigned long int calibration_P(signed long int adc_P);
unsigned long int calibration_H(signed long int adc_H);
int BME280_command(XIicPs *Iic, u8 command);
int loop(XIicPs *Iic);
#endif 
/* BME280_H_ */



/*
 * st7032.h
 *
 *  Created on: 2015/01/31
 *      Author: KeitetsuWorks
 *      ref:- https://github.com/KeitetsuWorks/xiicps_st7032/blob/master/src/st7032.h
 *      ref:- https://keitetsu.blogspot.com/2015/02/zybo-zynq-psi2c.html
 */

#ifndef ST7032_H_
#define ST7032_H_

//#define _DEBUG_ST7032

#define ST7032_IIC_ADDR				0x3E
#define ST7032_IIC_SCLK_RATE		100000
#define ST7032_DEFAULT_CONTRAST		0x38

// commands
#define ST7032_CLEARDISPLAY			0x01
#define ST7032_RETURNHOME			0x02
#define ST7032_ENTRYMODESET			0x04
#define ST7032_DISPLAYCONTROL		0x08
#define ST7032_CURSORSHIFT			0x10
#define ST7032_FUNCTIONSET			0x20
#define ST7032_SETCGRAMADDR			0x40
#define ST7032_SETDDRAMADDR			0x80

#define ST7032_EX_SETBIASOSC		0x10	// Bias selection/Internal OSC frequency adjust
#define ST7032_EX_SETICONRAMADDR	0x40	// Set ICON RAM address
#define ST7032_EX_POWICONCONTRASTH	0x50	// Power/ICON control/Contrast set(high byte)
#define ST7032_EX_FOLLOWERCONTROL	0x60	// Follower control
#define ST7032_EX_CONTRASTSETL		0x70	// Contrast set(low byte)

// flags for display entry mode
#define ST7032_ENTRYRIGHT			0x00
#define ST7032_ENTRYLEFT			0x02
#define ST7032_ENTRYSHIFTINCREMENT	0x01
#define ST7032_ENTRYSHIFTDECREMENT	0x00

// flags for display on/off control
#define ST7032_DISPLAYON			0x04
#define ST7032_DISPLAYOFF			0x00
#define ST7032_CURSORON				0x02
#define ST7032_CURSOROFF			0x00
#define ST7032_BLINKON				0x01
#define ST7032_BLINKOFF				0x00

// flags for display/cursor shift
#define ST7032_DISPLAYMOVE			0x08
#define ST7032_CURSORMOVE			0x00
#define ST7032_MOVERIGHT			0x04
#define ST7032_MOVELEFT				0x00

// flags for function set
#define ST7032_8BITMODE				0x10
#define ST7032_4BITMODE				0x00
#define ST7032_2LINE				0x08
#define ST7032_1LINE				0x00
#define ST7032_5x16DOTS				0x04
#define ST7032_5x8DOTS				0x00
#define ST7032_EX_INSTRUCTION		0x01	// IS: instruction table select

// flags for Bias selection
#define ST7032_BIAS_1_4				0x08	// bias will be 1/4
#define ST7032_BIAS_1_5				0x00	// bias will be 1/5

// flags Power/ICON control/Contrast set(high byte)
#define ST7032_ICON_ON				0x08	// ICON display on
#define ST7032_ICON_OFF				0x00	// ICON display off
#define ST7032_BOOST_ON				0x04	// booster circuit is turn on
#define ST7032_BOOST_OFF			0x00	// booster circuit is turn off
#define ST7032_OSC_122HZ			0x00	// 122Hz@3.0V
#define ST7032_OSC_131HZ			0x01	// 131Hz@3.0V
#define ST7032_OSC_144HZ			0x02	// 144Hz@3.0V
#define ST7032_OSC_161HZ			0x03	// 161Hz@3.0V
#define ST7032_OSC_183HZ			0x04	// 183Hz@3.0V
#define ST7032_OSC_221HZ			0x05	// 221Hz@3.0V
#define ST7032_OSC_274HZ			0x06	// 274Hz@3.0V
#define ST7032_OSC_347HZ			0x07	// 347Hz@3.0V

// flags Follower control
#define ST7032_FOLLOWER_ON			0x08	// internal follower circuit is turn on
#define ST7032_FOLLOWER_OFF			0x00	// internal follower circuit is turn off
#define ST7032_RAB_1_00				0x00	// 1+(Rb/Ra)=1.00
#define ST7032_RAB_1_25				0x01	// 1+(Rb/Ra)=1.25
#define ST7032_RAB_1_50				0x02	// 1+(Rb/Ra)=1.50
#define ST7032_RAB_1_80				0x03	// 1+(Rb/Ra)=1.80
#define ST7032_RAB_2_00				0x04	// 1+(Rb/Ra)=2.00
#define ST7032_RAB_2_50				0x05	// 1+(Rb/Ra)=2.50
#define ST7032_RAB_3_00				0x06	// 1+(Rb/Ra)=3.00
#define ST7032_RAB_3_75				0x07	// 1+(Rb/Ra)=3.75

// Function Prototypes
int ST7032_init(XIicPs *Iic);

int ST7032_clearDisplay(XIicPs *Iic);
int ST7032_returnHome(XIicPs *Iic);
int ST7032_setCursor(XIicPs *Iic, u8 row, u8 col);

int ST7032_command(XIicPs *Iic, u8 command);
int ST7032_write(XIicPs *Iic, u8 value);

#endif /* ST7032_H_ */
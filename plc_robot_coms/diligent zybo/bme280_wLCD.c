/*
 * BME280.c Example showing BME280 on i2c diligent zybo
 *
 */

/***************************** Include Files **********************************/
#include "xparameters.h"
#include "xiicps.h"
#include "xil_printf.h"
#include "sleep.h"

#include "bme280.h"
#include "st7032.h"

unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

u16 dig_T1 = 0;
int dig_T2 = 0;
int dig_T3 = 0;
u16 dig_P1 = 0;
int dig_P2 = 0;
int dig_P3 = 0;
int dig_P4 = 0;
int dig_P5 = 0;
int dig_P6 = 0;
int dig_P7 = 0;
int dig_P8 = 0;
int dig_P9 = 0;
char  dig_H1 = 0;
int dig_H2 = 0;
char  dig_H3 = 0;
int dig_H4 = 0;
int dig_H5 = 0;
char  dig_H6 = 0;


int initIicDriver(XIicPs *Iic, u16 DeviceId, u32 FsclHz)
{
	int Status;
	XIicPs_Config *Config;	/**< configuration information for the device */

	/*
	 * Initialize the IIC driver so that it's ready to use
	 * Look up the configuration in the config table,
	 * then initialize it.
	 */
	Config = XIicPs_LookupConfig(DeviceId);
	if(Config == NULL){
		xil_printf("Error: XIicPs_LookupConfig()\r\n");
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(Iic, Config, Config->BaseAddress);
	if(Status != XST_SUCCESS){
		xil_printf("Error: XIicPs_CfgInitialize()\r\n");
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Status = XIicPs_SelfTest(Iic);
	if(Status != XST_SUCCESS){
		xil_printf("Error: XIicPs_SelfTest()\r\n");
		return XST_FAILURE;
	}

	/*
	 * Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(Iic, FsclHz);

	return XST_SUCCESS;
}

int writeReg(XIicPs *Iic, u8 data)
{
	int Status;
	u8 buffer[2];

	buffer[0] = 0x00;
	buffer[1] = data;

	Status = XIicPs_MasterSendPolled(Iic, buffer, 2, IIC_ADDR);
#ifdef _DEBUG_BME280
	xil_printf("BME280_command: 0x%02x\r\n", buffer[1]);
#endif
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while(XIicPs_BusIsBusy(Iic)){
		/* NOP */
	}

	usleep(27);	// >26.3us

	return XST_SUCCESS;
}

#define SEND_SIZE (1) 
#define RECV_SIZE (1)
#define SB_SIZE 2
int IicPsReadInt(XIicPs *Iic, u8 cmd, int *res)
{
	int Status;

	Status = XIicPs_SelfTest(Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	u8 registers[SB_SIZE] = { 0x00, 0x01 };                                                             // Register Address for Temperature MSB and LSB
	u8 SendBuffer[SB_SIZE] = { 0x00, 0x00 };
	u8 RecvBuffer[SB_SIZE] = { 0x00, 0x00 };
	registers[1] = cmd;
	int result = 0;
	for(int idx=0; idx < SB_SIZE; idx++) {
		XIicPs_SetOptions(Iic, XIICPS_REP_START_OPTION);                                           // enable repeated start condition
		SendBuffer[0] = registers[idx];
		Status = XIicPs_MasterSendPolled(Iic, SendBuffer, SEND_SIZE, IIC_ADDR);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}
		if ( !(Iic->IsRepeatedStart) ) {
			while (XIicPs_BusIsBusy(Iic)) {
				/* NOP */
			}
		}
		Status = XIicPs_MasterRecvPolled(Iic, RecvBuffer, RECV_SIZE, IIC_ADDR);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		XIicPs_ClearOptions(Iic, XIICPS_REP_START_OPTION);                                           // disable repeated start condition

		result |= RecvBuffer[0];
		if (idx == 0) { 
			result <<= 8;
		}
		usleep(100);
	}
	*res = result;
	return XST_SUCCESS;
}

int IicPsReadU8(XIicPs *Iic, u8 cmd, u8 *res)
{
	int Status;
	u8 registers[1] = { 0x00 };                                                             // Register Address for Temperature MSB and LSB
	u8 SendBuffer[1] = { 0x00 };
	u8 RecvBuffer[1] = { 0x00 };
	u8 result = 0;

	Status = XIicPs_SelfTest(Iic);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	registers[0] = cmd;
	for(int idx=0; idx < 1; idx++) {
		XIicPs_SetOptions(Iic, XIICPS_REP_START_OPTION);                                           // enable repeated start condition
		SendBuffer[0] = registers[idx];
		Status = XIicPs_MasterSendPolled(Iic, SendBuffer, SEND_SIZE, IIC_ADDR);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}
		if ( !(Iic->IsRepeatedStart) ) {
			while (XIicPs_BusIsBusy(Iic)) {
				/* NOP */
			}
		}
		Status = XIicPs_MasterRecvPolled(Iic, RecvBuffer, RECV_SIZE, IIC_ADDR);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

		XIicPs_ClearOptions(Iic, XIICPS_REP_START_OPTION);                                           // disable repeated start condition

		result = RecvBuffer[0];
		usleep(100);
	}
	*res = result;
	return XST_SUCCESS;
}

int readTrim(XIicPs *Iic)
{
	u8 data[32];
    int rcode = 0;
	
	for (int i = 0; i < 24; i++) {
		rcode = IicPsReadU8(Iic, 0x88 + i, &data[i]);
		if(rcode != XST_SUCCESS) {
		    return XST_FAILURE;
	    }
	}

	rcode = IicPsReadU8(Iic, 0xa1, &data[24] );
    if(rcode != XST_SUCCESS) {
        return XST_FAILURE;
    }
		
	for (int i = 0; i < 7; i++) {
		rcode = IicPsReadU8(Iic, 0xe1 + i, &data[i + 25]);
        if(rcode != XST_SUCCESS) {
            return XST_FAILURE;
        }
	}

	dig_T1 = (data[1] << 8) | data[0];
	dig_T2 = (data[3] << 8) | data[2];
	dig_T3 = (data[5] << 8) | data[4];
	dig_P1 = (data[7] << 8) | data[6];
	dig_P2 = (data[9] << 8) | data[8];
	dig_P3 = (data[11]<< 8) | data[10];
	dig_P4 = (data[13]<< 8) | data[12];
	dig_P5 = (data[15]<< 8) | data[14];
	dig_P6 = (data[17]<< 8) | data[16];
	dig_P7 = (data[19]<< 8) | data[18];
	dig_P8 = (data[21]<< 8) | data[20];
	dig_P9 = (data[23]<< 8) | data[22];
	dig_H1 = data[24];
	dig_H2 = (data[26]<< 8) | data[25];
	dig_H3 = data[27];
	dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
	dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
	dig_H6 = data[31];
	return XST_SUCCESS;
}

void setup(XIicPs *Iic)
{
	u8 osrs_t = 1;             // Temperature oversampling x 1
	u8 osrs_p = 1;             // Pressure oversampling x 1
	u8 osrs_h = 1;             // Humidity oversampling x 1
	u8 mode = 3;               // Normal mode
	u8 t_sb = 5;               // Tstandby 1000ms
	u8 filter = 0;             // Filter off
	u8 spi3w_en = 0;           // 3-wire SPI Disable
    int rcode = 0;

	u8 ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
	u8 config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
	u8 ctrl_hum_reg  = osrs_h;

	rcode = writeReg(Iic, 0xf2, ctrl_hum_reg);
    if(rcode != XST_SUCCESS) {
        return XST_FAILURE;
    }
	rcode = writeReg(Iic, 0xf4, ctrl_meas_reg);
    if(rcode != XST_SUCCESS) {
        return XST_FAILURE;
    }
	rcode = writeReg(Iic, 0xf5, config_reg);
    if(rcode != XST_SUCCESS) {
        return XST_FAILURE;
    }
	rcode = readTrim(Iic);    
    if(rcode != XST_SUCCESS) {
        return XST_FAILURE;
    }
	return XST_SUCCESS;	
}

int readData(XIicPs *Iic)
{
	int i = 0;
    int rcode = 0;
	uint32_t data[8];
	for (int i = 0; i < 8; i++) {
		rcode = IicPsReadU8(Iic, 0xf7 + i, &data[i]);
		if(rcode != XST_SUCCESS) {
		    return XST_FAILURE;
	    }
	}
	pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	hum_raw  = (data[6] << 8) | data[7];
	return XST_SUCCESS;	
}

signed long int calibration_T(signed long int adc_T)
{
	signed long int var1, var2, T;
	var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
	signed long int var1, var2;
	unsigned long int P;
	var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
	var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
	var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
	var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
	if (var1 == 0) {
		return 0;
	}
	P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
	if (P < 0x80000000) {
		P = (P << 1) / ((unsigned long int) var1);
	} else {
		P = (P / (unsigned long int)var1) * 2;
	}
	var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
	var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
	P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
	return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
	signed long int v_x1;

	v_x1 = (t_fine - ((signed long int)76800));
	v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
		((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
		(((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
		((signed long int) dig_H2) + 8192) >> 14));
	v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
	v_x1 = (v_x1 < 0 ? 0 : v_x1);
	v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
	return (unsigned long int)(v_x1 >> 12);
}

int BME280_command(XIicPs *Iic, u8 command)
{
	int Status;
	u8 buffer[2];

	buffer[0] = 0x00;
	buffer[1] = command;

	Status = XIicPs_MasterSendPolled(Iic, buffer, 2, IIC_ADDR);
#ifdef _DEBUG_BME280
	xil_printf("BME280_command: 0x%02x\r\n", buffer[1]);
#endif
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while(XIicPs_BusIsBusy(Iic)){
		/* NOP */
	}

	usleep(27);	// >26.3us

	return XST_SUCCESS;
}

/******************************************************************************/
/*
 * IIC LCD (ST7032) Initialization Function
 *
 * @param
 * XIicPs *Iic		Instance of the IIC Controller
 *
 * @return
 * int				XST_SUCCESS:	Success
 * 					XST_FAILURE:	Failure
 *
 * @note
 * None
 */
int ST7032_init(XIicPs *Iic)
{
	int Status;

	usleep(40000);						// Wait time >40ms after VDD stable

	Status = ST7032_command(
			Iic,
			ST7032_FUNCTIONSET |		// Function Set
			ST7032_8BITMODE |			// 8-bit bus mode with MPU
			ST7032_2LINE |				// 2-line display mode
			ST7032_5x8DOTS				// normal display font (5x8 dot)
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_FUNCTIONSET |		// Function Set
			ST7032_8BITMODE |			// 8-bit bus mode with MPU
			ST7032_2LINE |				// 2-line display mode
			ST7032_5x8DOTS |			// normal display font (5x8 dot)
			ST7032_EX_INSTRUCTION		// extension instruction
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_EX_SETBIASOSC |		// Bias selection/Internal OSC frequency adjust
			ST7032_BIAS_1_5 |			// bias will be 1/5
			ST7032_OSC_183HZ			// 183Hz@3.0V
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_EX_CONTRASTSETL |	// Contrast set(low byte)
			(ST7032_DEFAULT_CONTRAST & 0x0f)
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_EX_POWICONCONTRASTH |// Power/ICON control/Contrast set(high byte)
			ST7032_ICON_OFF |			// ICON display off
			ST7032_BOOST_ON |			// booster circuit is turn on
			((ST7032_DEFAULT_CONTRAST >> 4) & 0x03)
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_EX_FOLLOWERCONTROL |	// Follower control
			ST7032_FOLLOWER_ON |		// internal follower circuit is turn on
			ST7032_RAB_2_00				// 1+(Rb/Ra)=2.00
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}
	usleep(200000);						// Wait time >200ms (for power stable)

	Status = ST7032_command(
			Iic,
			ST7032_FUNCTIONSET |		// Function Set
			ST7032_8BITMODE |			// 8-bit bus mode with MPU
			ST7032_2LINE |				// 2-line display mode
			ST7032_5x8DOTS				// normal display font (5x8 dot)
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_command(
			Iic,
			ST7032_DISPLAYCONTROL |		// Display ON/OFF
			ST7032_DISPLAYON |			// display is turned on
			ST7032_CURSOROFF |			// cursor is disappeared in current display
			ST7032_BLINKOFF				// cursor blink is off
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}


	Status = ST7032_clearDisplay(Iic);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


int ST7032_clearDisplay(XIicPs *Iic)
{
	int Status;

	Status = ST7032_command(Iic, ST7032_CLEARDISPLAY);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}
	sleep(2);

	return XST_SUCCESS;
}


int ST7032_returnHome(XIicPs *Iic)
{
	int Status;

	Status = ST7032_command(Iic, ST7032_RETURNHOME);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}
	sleep(2);

	return XST_SUCCESS;
}


int ST7032_setCursor(XIicPs *Iic, u8 row, u8 col)
{
	int Status;
	const int row_offsets[] = {0x00, 0x40};

	Status = ST7032_command(
			Iic,
			ST7032_SETDDRAMADDR |
			(row_offsets[row] + col)
			);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


/*
 * IIC LCD (ST7032) Command Execution Function
 *
 * @param
 * XIicPs *Iic		Instance of the IIC Controller
 * u8 command		Instruction Code of IIC LCD (ST7032)
 *
 * @return
 * int				XST_SUCCESS:	Success
 * 					XST_FAILURE:	Failure
 *
 * @note
 * None
 */
int ST7032_command(XIicPs *Iic, u8 command)
{
	int Status;
	u8 buffer[2];

	buffer[0] = 0x00;
	buffer[1] = command;

	Status = XIicPs_MasterSendPolled(Iic, buffer, 2, ST7032_IIC_ADDR);
#ifdef _DEBUG_ST7032
	xil_printf("ST7032_command: 0x%02x\r\n", buffer[1]);
#endif
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while(XIicPs_BusIsBusy(Iic)){
		/* NOP */
	}

	usleep(27);	// >26.3us

	return XST_SUCCESS;
}


/*
 * IIC LCD (ST7032) Data Transfer Function
 *
 * @param
 * XIicPs *Iic		Instance of the IIC Controller
 * u8 value			Transfer Data of IIC LCD (ST7032)
 *
 * @return
 * int				XST_SUCCESS:	Success
 * 					XST_FAILURE:	Failure
 *
 * @note
 * None
 */
int ST7032_write(XIicPs *Iic, u8 value)
{
	int Status;
	u8 buffer[2];

	buffer[0] = 0x40;
	buffer[1] = value;

	Status = XIicPs_MasterSendPolled(Iic, buffer, 2, ST7032_IIC_ADDR);
#ifdef _DEBUG_ST7032
	xil_printf("ST7032_write: 0x%02x\r\n", buffer[1]);
#endif
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while(XIicPs_BusIsBusy(Iic)){
		/* NOP */
	}

	usleep(27);	// >26.3us

	return XST_SUCCESS;
}

int loop(XIicPs *Iic)
{
	double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
	signed long int temp_cal;
	unsigned long int press_cal,hum_cal;
	int Status;
	char *text0 = "Hello ZYBO!";
	char *text1 = "Aitendo IIC LCD";
	
	Status = readData(Iic);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	temp_cal = calibration_T(temp_raw);
	press_cal = calibration_P(pres_raw);
	hum_cal = calibration_H(hum_raw);
	temp_act = (double)temp_cal / 100.0;                     // degC
	press_act = (double)press_cal / 100.0;                   // hPa
	hum_act = (double)hum_cal / 1024.0;                      // %

	xil_printf("temp=%.2f;press=%.2f;humid=%.2f;\n", temp_act, press_act, hum_act);
    sprintf(text0, "temp=%.2f", temp_act);
	Status = ST7032_setCursor(&Iic, 0, 0);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}
	while(*text0){
		Status = ST7032_write(&Iic, *text0);
	    if(Status != XST_SUCCESS){
		    return XST_FAILURE;
	    }
		text0++;
	}
	Status = ST7032_setCursor(&Iic, 1, 0);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}
    sprintf(text1, "oress=%.2f humid=%.2f", press_act, hum_act);
	while(*text1){
		Status = ST7032_write(&Iic, *text1);
	    if(Status != XST_SUCCESS){
		    return XST_FAILURE;
	    }
		text1++;
	}
	sleep(1);
	return XST_SUCCESS;
}

int main(void)
{
	int Status;
	XIicPs Iic;				                                /**< Instance of the IIC Device */

	xil_printf(" === Test Program for AD ZYBO IIC BME280 Temperature Pressure Humidity Sensor === \r\n");

	Status = initIicDriver(&Iic, IIC_DEVICE_ID, IIC_SCLK_RATE);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = setup(&Iic);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	Status = ST7032_init(&Iic);
	if(Status != XST_SUCCESS){
		return XST_FAILURE;
	}

	while (1) {
	    Status = loop(&Iic);
#ifdef _DEBUG_BME280
	    if(Status != XST_SUCCESS){
	        xil_printf("BME280_data error");
	    }
#endif		
	}
	return XST_SUCCESS;

}

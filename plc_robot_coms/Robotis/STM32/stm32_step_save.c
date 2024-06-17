/* example of writing to FATFS on STM32 (STM32F103C8T6)
//
// to save the controllers step and reboot reading it from the file saved

   uses dyanmixel protocol
   
*/

#include "DKS_Common_F103xB.h"
#include "DKS_SPI_F103xB.h"
#include "DKS_Timer_F103xB.h"
#include "DKS_GPIO_F103xB.h"
#include "DKS_SD.h"
#include "DKS_F103C8T6.h"

#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"

#include <stdio>
#include <stdlib>
#include <string>

/* error from dynamixel */
typedef enum
{
    HWFail = 0x01,
    InstructionErr = 0x02,
    CRCError = 0x03,
    DataRangeErr = 0x04,
    DataLenErr = 0x05,
    DataLimitErr = 0x06,	
    DataAccessErr = 0x07
} DynamixelError_e; 
/*
    0	Current Control Mode DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
    1	Velocity Control Mode This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
    3   (Default) Position Control Mode	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
    4	Extended Position Control Mode(Multi-turn)	This mode controls position. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
    5	Current-based Position Control Mode	This mode controls both position and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers.
    16	PWM Control Mode (Voltage Control Mode)
*/
typedef enum
{
    CurrentCtl = 0,
    VelocityCtl = 1,
    PositionCtl = 3,
    ExtPositionCtl = 4,
    CurrPositionCtl = 5,	
    PWMCtl = 16
} DynamixelCtlMode_e; 
		   
#define DS_TORQ_BIT 0x01u                               /* Torque On on startup (Torque Enable(64) is set to */
#define DS_BOOT_RAM_BIT 0x02u                           /* On startup, use the backup data to restore the RAM area. */

/* shutdown conf word */
#define VOLT_SHUT_BT 1u                                  
#define TEMP_SHUT_BT 4u
#define MTRENC_SHUT_BT 8u
#define SHOCK_SHUT_BT 16u
#define OVERLD_SHUT_BT 32u

/* define status return for all msgs */
#define PINGS_ONLY 0u
#define PINGS_AND_READS 1u
#define ALL_MESSAGES 2u

/* drive mode variables */
#define IG_TORQ_DM 0x08
#define TIME_PROFILE_DM 0x04                             /* set to time profile else its velocity profile */
#define REVERSE_MODE_DM 0x01

#define COM_DLY_DS 9u                                    /* set communication delay */
#define DRV_MODE_DS 10u                                  /* control param drive mode */
#define OP_MODE_DS 11u                                   /* control param operating mode */
#define HOMING_OFFSET_DS 20u                             /* control param homing offset */
#define MOVING_THRESH_DS 24u                             /* in motion 0-1023 */
#define OVERTEMP_THRESH_DS 31u                           /* shutdown if temp exceeds this 0-100 and Overheating Error Bit(0x04) is set */
#define MIN_VOLT_THRESH_DS 32u                           /* shutdown if volts under this 95-160 */
#define MAX_VOLT_THRESH_DS 34u                           /* shutdown if volts over this 95-160 */
#define WRITE_DS 60u                                     /* control param where dyanmixel start-up config is stored */
#define SHUTD_CONF_DS 63u
#define TORQUE_ENAB_DS 64u
#define STAT_RTN_DS 68u

#define GET_HW_STATUS 70u;

/* controller set-up */
#define VEL_CON_P_DS 78u
#define VEL_CON_I_DS 76u
#define POS_CON_P_DS 84u
#define POS_CON_I_DS 82u
#define POS_CON_D_DS 80u
#define POS_FF_G1_DS 90u
#define POS_FF_G2_DS 88u

#define BUS_WDOG_DS 98u                                   /* replies with  */
#define PWM_GOAL_DS 100u                                  /* pwm goal */
#define CURRENT_GOAL_DS 102u                              /* current goal */
#define VELO_GOAL_DS 104u                                 /* velocity goal */
#define POSN_GOAL_DS 116u                                 /* position goal */

#define PROFILE_ACC 108u                                  /* profile acceleration */
#define PROFILE_VEL 112u                                  /* profile velocity */

#define SENDDELAYCOUNT 500
#define RCVDELAYCOUNT 510

DKS::STM32F103C8T6 board(DKS::BlackPill);
//DKS::DigitalIn cd;
DKS::DigitalOut cs;
DKS::Ticker ticker;
DKS::SPI::SPI spi;

char write_buffer[_MAX_SS];
uint16_t step = 0;                                                              // default step for the seqyence
uint16_t sub_state = 0;

extern "C" void TIM3_IRQHandler()
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3) == 1)
    {
        LL_TIM_ClearFlag_UPDATE(TIM3);
    }
    DKS::FatFs::disk_timerproc();
}

// set pwm output timers
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USART1 init function */
UART_HandleTypeDef huart1;

// encoder start
TIM_HandleTypeDef htim3;

/* timer interrupt every X seconds */
TIM_HandleTypeDef htim17;

int _index = 0;                                      /* receive index */
uint8_t RxData;
uint8_t RxBuff[20];                                  /* rcv buffer */
uint8_t TxDataIT[20];                                /* tx buffer */
uint8_t TxDataITMem[20];                             /* memory of tx buffer */
uint16_t Data1 = 116u;                               /* position */
uint16_t Data2 = 300u;                               /* value */
uint8_t id = 0x01u;			                         /* id ID1(XM430-W210)  */
uint8_t device_ping = 0;                             /* sets to 1 when ping is okay */
volatile uint8_t new_pos = 0;                        /* a tri-state for position == requesting a new position, read a new position, set the new position */
uint32_t position_readbk = 0;                        /* the position returned from a read back of the robotis dynamixel drive */
uint8_t re_trans1 = 0;                               /* flags to allow re-transmision for bad messages once */
uint8_t re_trans2 = 0;
uint8_t re_trans3 = 0;
uint16_t len_msg = 0u;                               /* msg len read back */
uint8_t inst = 0u;                                   /* instance */
uint8_t err = 0u;                                    /* error byte */

/* data for timed buffer flush */
uint32_t msgCnt = 0;
uint32_t lstMsgCnt = 0;
volatile uint8_t noActivityForTime = 0;	

/* -------------- function defs ---------------- */
void SystemClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void TIM17_IRQHandler(void);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base);
void HAL_UART_MspInit(UART_HandleTypeDef* huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart);
static int USART1_UART_Init(void);
void user_tim1_pwm_setvalue(float value);
void user_tim2_pwm_setvalue(float value);
static void encoder_t3_start(void);
static int get_encoder_t3_value(void);
static void encoder_t2_start(void);
static int get_encoder_t2_value(void);
uint16_t update_dyn_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, uint16_t data_blk_size);
void do_dynamixel_crc(unsigned char* TxPacket, uint16_t data_blk_size, uint8_t * crc_l, uint8_t * crc_h, uint8_t endian);
void write_dyna(uint16_t *Data1, uint16_t *Data2, uint8_t id);
void write_dyna_IT(uint16_t *Data1, uint16_t *Data2, uint8_t id);
void write_reg_dyna(uint16_t *Data1, uint16_t *Data2, uint8_t id);
void write_reg_dyna_IT(uint16_t *Data1, uint16_t *Data2, uint8_t id);
void factory_reset_dyna(uint8_t id);
void reboot_dyna(uint8_t id);
void ping_dyna(uint8_t id);
void ping_dyna_IT(uint8_t id);
void read_dyna_pos_IT(uint8_t id);
static int16_t isdigitanum(unsigned char ch);

int main(void)
{
        DKS::InitSystem();                                                  // initialise HAL and board
        board.Init();

        // Prepareing FatFs
        spi.Init(SPI1, 0, 0, 0);                                            // PA5: SCL, PA6: MISO, PA7: MOSI
        cs.Init(GPIOB, LL_GPIO_PIN_8, DKS::Push_Pull, DKS::Pull_Up);
        ticker.Init(TIM3, 1, DKS::TimeUnit_MilliSec);
        rtc.Init(DKS::Rtc_ClockSource_LSE);
        DKS::FatFs::SetEnv(&spi, &cs, &ticker, 0, 0);

        //Local variables
        FATFS FatFs;                                                          // Logical drive work area (file system object)
        FIL fil;                                                              // file object used when opening and closing the file
        FRESULT res;                                                          // result from the call
        const std::string fname = "test_the_write.txt";                       // the file used to check the write function on boot up
        const std::string fname2 = "sequence_data.txt";                       // the file we are writing and reading to save the step
        uint8_t writelen(0);
        char line[82];                                                        // read back of the line saved in the file
        std::string resMsg(line);

        // first check that we can read and write a test file correctly to check our FATFS 
        // Create File object
        res = f_mount(&FatFs, "", 0);
        if (res != FR_OK) return EXIT_FAILURE;
        res = f_open(&fil, fname.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
        if (res != FR_OK) return EXIT_FAILURE;

        // Write text
		sprintf(write_buffer, "step is = %d\r\n", step);
        writelen=f_puts(&write_buffer, &fil);
        if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
        res = f_close(&fil);
        if (res != FR_OK) return EXIT_FAILURE;

        // Read Test
        res = f_open(&fil, fname.c_str(), FA_READ);
        if (res != FR_OK) return EXIT_FAILURE;
        f_gets(line, sizeof(line) , &fil);
        res = f_close(&fil);
        if (res != FR_OK) return EXIT_FAILURE;

        // Confirm result if not ok FATFS not working
        std::string sndMsg(write_buffer);
        if (resMsg == sndMsg) nucleo.led->write(1);
        else return EXIT_FAILURE;

	    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
        HAL_Init();
        /* Configure the system clock */
        SystemClock_Config();

		GPIO_InitTypeDef GPIO_InitStruct = {0};
        __HAL_RCC_GPIOA_CLK_ENABLE();                              // GPIOA 
        GPIO_InitStruct.Pin = GPIO_PIN_5 / GPIO_PIN_4;             // Set both PIN5 and 4 at the same time
	    //GPIO_InitStruct.Pin = GPIO_PIN_ALL;                      set all 16 simulatenous
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;                        // Configure whether the GPIO is pulled up, pulled down, or not.
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;               // Set the operating frequency
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6;             
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;                        // Configure whether the GPIO is pulled up, pulled down, or not.
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;               // Set the operating frequency
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	   
        GPIO_InitStruct.Pin = GPIO_PIN_7;             
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;                        // Configure whether the GPIO is pulled up, pulled down, or not.
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;               // Set the operating frequency
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	   
        /*Configure GPIO pin : Bit1_state_Pin */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Bit1_state_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(Bit1_state_GPIO_Port, &GPIO_InitStruct);
	   
        /*Configure GPIO pin : StateLED_Pin */
        GPIO_InitStruct.Pin = StateLED_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(StateLED_GPIO_Port, &GPIO_InitStruct);
  
        MX_TIM1_Init();                                             // initialise the timers for pwm and encoder
        MX_TIM2_Init();
        MX_TIM3_Init();

        MX_TIM17_Init();                                           /* interrupt timer */
        HAL_TIM_Base_Start_IT(&htim17);
		
        __HAL_RCC_GPIOB_CLK_ENABLE();
        if (USART1_UART_Init() == -1) return EXIT_FAILURE;
        //unsigned char uart1buf[100];
		HAL_UART_MspInit(&huart1);
		// enable UART1 Rx and Tx interrupts
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);

        /* enable interrupt timer17 for buffer flush if no activity for 1 second on receive port */
        HAL_TIM_Base_MspInit(&htim17);
		
		// now read the saved step file if we have one otherwise default the step to zero
        if (f_open(&fil, fname2.c_str(), FA_READ) == FR_OK) {
            f_gets(line, sizeof(line) , &fil);
			unsigned char* token;
			uint16_t fieldNo=0u;
			unsigned char strField1[8u];                                               // field to contain the step number read from file
			token = strtok(&line, "="); 
			while (token != NULL)                                                       // read each delimetered field in the string
            {
              switch (fieldNo)
              {
                case 0u:                                                               // reads the first field
				{
					if (!strncmp(strField1,"step",4))                                  // its says step
                    {
                        token = strtok(NULL, ",");                                     // check for another comma
                        ++fieldNo;
					}
					else
					{
						token = NULL;                                                 // exit file is not containing the step
					}
				}
				break;
				
                case 1u:                                                              // reads the 2nd field
				{
                    sprintf( strField1,"%s\n", token );
                    if(isdigitanum(strField1[0u]))                                        // the 1st char was an ascii number then convert it
                    {
                       step = atoi(strField1);                                 
                    }
                    token = strtok(NULL, ",");                                         // check for another comma
                    ++fieldNo;
				}
                break;
				
				default:                                                               // got an errot with the value ignore at defaulted step
				{}
				break;
			  }
			}
			// close the file
			res = f_close(&fil);
            if (res != FR_OK) return EXIT_FAILURE;
		} 

        encoder_t3_start();	                                                                   /* encoder start */

        ping_dyna_IT(id);                                                                      // ping the dynamixel at the specified id to see if its communicating
        /* wait for ping success */
		
        uint16_t velocities[10] = { 276u, 267u, 300u, 140u, 200u, 50u, 10u, 310u, 110u, 40u };	/* list of velocities we set in the sequence */
        uint8_t v_cnt = 0;

        sub_state = 0;

        /*
		    set up for Dynamixel id 1 (XM430-W210) ref :- https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
		*/		
		while (sub_state <= 18) {                                  /* while all the config steps are not complete with status reply ok keep polling at the poll rate */
            switch (sub_state) {
                case 0:
			    {
                    Data1 = STAT_RTN_DS;                           /* define which messages return status */
                    Data2 = ALL_MESSAGES;                          /* set this to all */
                    id = 0x1u;			                           /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);             /* write to robotis dynamixel */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 1;                                                                     /* advance as we got the status back */
                        }
					}
                }
			    break;
		

		         case 1:
			    {
                    Data1 = WRITE_DS;                              /* write the start-up config */
                    Data2 = DS_BOOT_RAM_BIT | DS_TORQ_BIT;         /* torque on restore RAM from backup */
                    id = 0x1u;			                           /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);             /* write to robotis dynamixel */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 2;                                                                     /* advance as we got the status back */
                        } 
                    }						
                }
                break;
				
                case 2:
                {
                    /*
                        Operating Mode(11)
                        Value	Operating Mode	Description
                        0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
                        1   	Velocity Control Mode	This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
                        3(Default)	Position Control Mode	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
                        4	Extended Position Control Mode(Multi-turn)	This mode controls position. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
                        5	Current-based Position Control Mode	This mode controls both position and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers.
                        16	PWM Control Mode (Voltage Control Mode)
                        */
                    Data1 = OP_MODE_DS;
                    Data2 = DynamixelCtlMode_e::PositionCtl;
                    id = 0x1u;			                           /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);             /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 3;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;				

                case 3:
                {					
                    /*
                     If the DYNAMIXEL receives an Instruction Packet, it will return the Status Packet after the time of the set Return Delay Time(9).
                     Note that the range of values is 0 to 254 (0XFE) and its unit is 2 [μsec]. For instance, if the Return Delay Time(9) is set to ‘10’, the Status Packet will be returned after 20[μsec] when the Instruction Packet is received.
                     Default value ‘250’(500[μsec]) units are 2 μsec
                     */
                    Data1 = COM_DLY_DS;
                    Data2 = 50u;                                    /* 100 μsec */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 4;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 4:
                {					
                    /* set up shutdown criterea 
                    The DYNAMIXEL can protect itself by detecting dangerous situations that could occur during the operation. Each Bit is inclusively processed with the ‘OR’ logic, therefore, multiple options can be generated. For instance, when ‘0x05’ (binary : 00000101) is defined in Shutdown(63), DYNAMIXEL can detect both Input Voltage Error(binary : 00000001) 
                    and Overheating Error(binary : 00000100). If those errors are detected, Torque Enable(64) is cleared to ‘0’ and the motor’s output becomes 0 [%].
                    REBOOT is the only method to reset Torque Enable(64) to ‘1’(Torque ON) after the shutdown.
                    Check Alert Bit(0x80) in an error field of Status Packet or a present status via Hardware Error Status(70). The followings are detectable situations
                    */
                    Data1 = OVERTEMP_THRESH_DS;
                    Data2 = 75u;                                    /* if temp over 75 degrees shutdown and turn torque off */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 4;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 4:
                {					
                    Data1 = MIN_VOLT_THRESH_DS;
                    Data2 = 105u;                                    /* min vlts */
                    id = 0x1u;			                             /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);               /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 5;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 5:
                {				
                    Data1 = MAX_VOLT_THRESH_DS;
                    Data2 = 150u;                                    /* max vlts */
                    id = 0x1u;			                             /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);               /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 6;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 6:
                {					
                    Data1 = SHUTD_CONF_DS;                                                                         /* configure shutdown mode on error (trip) */
                    Data2 = VOLT_SHUT_BT | TEMP_SHUT_BT | MTRENC_SHUT_BT | SHOCK_SHUT_BT | OVERLD_SHUT_BT;         /* configure the shutdown word */
                    id = 0x1u;			                                                                           /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);                                                             /* write to robotis dynamixel */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                                 /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {                  /* check a set packet was received and hanshaked correctly */
                            sub_state = 7;                                                                         /* advance as we got the status back */
                        }
                    }
                }
                break;		

                case 7:
                {				
                    /* velocity PI controller set-up */
                    Data1 = VEL_CON_P_DS;                           /* proportional band */
                    Data2 = 7000u;                                  /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 8;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;	

                case 8:
                {					
                    Data1 = VEL_CON_I_DS;                           /* integral time */
                    Data2 = 100u;                                   /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {              /* check a set packet was received and hanshaked correctly */
                            sub_state = 9;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 9:
                {					
                    /* position PID with feedfoward gain controller set-up */
                    Data1 = POS_CON_P_DS;                           /* proportional band */
                    Data2 = 6000u;                                  /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 10;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 10:
                {				
                    Data1 = POS_CON_I_DS;                           /* integral time */
                    Data2 = 200u;                                   /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 11;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 11:
                {					
                    Data1 = POS_CON_D_DS;                           /* derivative term */
                    Data2 = 10u;                                    /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 12;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 12:
                {				
                    Data1 = POS_FF_G1_DS;                           /* feedforward gain 1 */
                    Data2 = 1000u;                                   /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 13;                                                                     /* advance as we got the status back */
                        }
                    }						
                }
                break;

                case 13:
                {					
                    Data1 = POS_FF_G2_DS;                           /* feedforward gain 2 */
                    Data2 = 890u;                                   /* 0-16383 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 14;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 14:
                {					
                    Data1 = BUS_WDOG_DS;                            /* bus wtachdog */
                    Data2 = 0u;                                     /* clear wdog */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 15;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 15:
                {					
                    Data1 = DRV_MODE_DS;                            /* set drive mode */
                    Data2 = TIME_PROFILE_DM | IG_TORQ_DM;           /* ignore torque on do action adn time profile not velocity is chosen */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 16;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 16:
                {					
                    Data1 = PROFILE_ACC;                            /* profile acceleration */
                    Data2 = 7000;                                   /* range 0 ~ 3276 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 17;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 17:
                {					
                    Data1 = PROFILE_VEL;                            /* profile acceleration */
                    Data2 = 6000;                                   /* range 0 ~ 3276 */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 18;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                case 18:
                {					
                    Data1 = TORQUE_ENAB_DS;                         /* torque enable */
                    Data2 = 1u;                                     /* on */
                    id = 0x1u;			                            /* id ID1(XM430-W210)  */	  
                    write_dyna_IT(&Data1, &Data2, id);              /* write via robotis dynamixel control protocol */	
					for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
                        if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {               /* check a set packet was received and hanshaked correctly */
                            sub_state = 19;                                                                     /* advance as we got the status back */
                        }
                    }
                }
                break;

                default:
                {
                    sub_state = 19;
                }
                break;
            }
            HAL_Delay(1000);                                         /* poll interval */			
        }
		
        // do the state engine for the robot here.......	
        while(1) {  
           /* USER CODE BEGIN 3 */    
		   switch(step) {
               case 0:
               {
                  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
				  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)                   // pin1 off set pin 3 otherwise unset pin3
                  {
                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
                  }
                  else
                  {
                     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
                  }		
                  user_tim1_pwm_setvalue(10.0f);	
                  user_tim2_pwm_setvalue(40.0f);
				  if (device_ping == 1) {
                      Data1 = POSN_GOAL_DS;                                                         /* initial goal position */
                      Data2 = 300u;                                                                 /* init value of position */
                      id = 0x1u;			                                                        /* id ID1(XM430-W210)  */	  
				      write_dyna_IT(&Data1, &Data2, id);                                            /* write to robotis dynamixel */
                  }
                  // a simple msg transmission example HAL_UART_Transmit( &huart1, uart1buf, strlen(uart1buf) + 1, 0xFFFF);	
				  
				  // save the step to the FATFs for recall on reboot
                  res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                  if (res != FR_OK) return EXIT_FAILURE;
		          sprintf(write_buffer, "step = %d\r\n", step+1);
                  writelen=f_puts(&write_buffer, &fil);
                  if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                  res = f_close(&fil);
                  if (res != FR_OK) return EXIT_FAILURE;
                  HAL_Delay(1000);
				  ++step;
               }
			   break;
			   
               case 1:
               {
                  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7); 
				  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
                  {
                      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
                  }
                  else
                  {
                     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
                  }	
		          if (get_encoder_t3_value() > 80)
                  {
                      user_tim1_pwm_setvalue(5.0f);	
			      } else {
                      user_tim1_pwm_setvalue(15.0f);						  
				  }					  
				  // save the step to the FATFs for recall on reboot
                  res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                  if (res != FR_OK) return EXIT_FAILURE;
		          sprintf(write_buffer, "step = %d\r\n", step+1);
                  writelen=f_puts(&write_buffer, &fil);
                  if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                  res = f_close(&fil);
                  if (res != FR_OK) return EXIT_FAILURE;
                  HAL_Delay(1000);
				  
				  ++step;
               }
			   break;

               case 2:
               {
				  user_tim2_pwm_setvalue(70.0f);	
                  if (device_ping == 1) {
					  new_pos = 0;                                    /* reset position read status to reading */
					  read_dyna_pos_IT(id);                           /* send out a read of position for device id 1 and wait until interrupt has set the flag to say it has been read */
                      if (new_pos == 1) {                             /* ready we read the new position */
                          Data1 = POSN_GOAL_DS;                       /* goal position */
                          Data2 = (position_readbk + 100u) % 4095u;   /* value */
                          id = 0x1u;			                      /* id ID1(XM430-W210)  */	  
				          write_dyna_IT(&Data1, &Data2, id);
					      for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
						      if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {            /* check a set packet was received and hanshaked correctly */
						          new_pos = 2;                                                                     /* advance as we got the status back */
                              }
                          }							  
                      }
                  } else {
                     new_pos = 2;                                     /* no drive then just step forwards with the sequence */
                  }					 
				  if (new_pos == 2 ) {                                /* we completed setting the position on the dynamixel or we skipped it */
				      // save the step to the FATFs for recall on reboot
                      res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                      if (res != FR_OK) return EXIT_FAILURE;
		              sprintf(write_buffer, "step = %d\r\n", step+1);
                      writelen=f_puts(&write_buffer, &fil);
                      if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                      res = f_close(&fil);
                      if (res != FR_OK) return EXIT_FAILURE;
                      HAL_Delay(1000);
                      new_pos = 0;	
					  
				      ++step;
                  }
               }
               break;

              case 3:
               {	
                  if (device_ping == 1) {
					  new_pos = 0;                                    /* reset position read status to reading */
					  read_dyna_pos_IT(id);
                      if (new_pos == 1) {                             /* ready we read the new position read from the interrupt */
                          Data1 = POSN_GOAL_DS;                       /* goal position */
                          Data2 = (position_readbk + 100u) % 4095u;   /* value */
                          id = 0x1u;			                      /* id ID1(XM430-W210)  */	  
				          write_dyna_IT(&Data1, &Data2, id);
					      for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
						      if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {
						          new_pos = 2;
                              }
                          }
                      }
                  } else {
                     new_pos = 2;                                     /* no drive then just step forwards */
                  }					 
				  if (new_pos == 2 ) {
				      // save the step to the FATFs for recall on reboot
                      res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                      if (res != FR_OK) return EXIT_FAILURE;
		              sprintf(write_buffer, "step = %d\r\n", step+1);
                      writelen=f_puts(&write_buffer, &fil);
                      if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                      res = f_close(&fil);
                      if (res != FR_OK) return EXIT_FAILURE;
                      HAL_Delay(1000);
                      new_pos = 0;	
					  
				      ++step;
                  }
               }
               break;

              case 4:
               {	
                  if (device_ping == 1) {
					  new_pos = 0;                                    /* reset position read status to reading */
					  read_dyna_pos_IT(id);                           /* send UART message to read position from duanmixel id as passed to this function */
                      if (new_pos == 1) {                             /* ready we read the new position */
                          Data1 = POSN_GOAL_DS;                       /* goal position */
                          Data2 = (position_readbk + 100u) % 4095u;   /* value */
                          id = 0x1u;			                      /* id ID1(XM430-W210)  */	  
				          write_dyna_IT(&Data1, &Data2, id);          /* send UART message to write position to duanmixel id as passed to this function */
					      for (int k=0; k < (3*SENDDELAYCOUNT)+(3*RCVDELAYCOUNT); k++) {                             /* wait for a delay of send and receive buffer will send if error msg returned */
						      if (HAL_GPIO_ReadPin(StateLED_GPIO_Port, StateLED_Pin) == GPIO_PIN_SET) {
						          new_pos = 2;
                              }
                          }
                      }
                  } else {
                     new_pos = 2;                                     /* no drive then just step forwards */
                  }					 
				  if (new_pos == 2 ) {
				      // save the step to the FATFs for recall on reboot
                      res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                      if (res != FR_OK) return EXIT_FAILURE;
		              sprintf(write_buffer, "step = %d\r\n", step+1);
                      writelen=f_puts(&write_buffer, &fil);
                      if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                      res = f_close(&fil);
                      if (res != FR_OK) return EXIT_FAILURE;
                      HAL_Delay(1000);
                      new_pos = 0;	                                    /* reset the read flag on the position as it was used */
					  
				      ++step;
                  }
               }
               break;
			   
               case 5:
               {
                  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); 
                  user_tim1_pwm_setvalue(0.503f);		
                  if (device_ping == 1) {
                      Data1 = TORQUE_ENAB_DS;                             /* torque disable to change operating mode */
                      Data2 = 0u;                                         /* off */
                      id = 0x1u;			                              /* id ID1(XM430-W210)  */	  
                      write_dyna_IT(&Data1, &Data2, id);                  /* write via robotis dynamixel control protocol */
                      Data1 = OP_MODE_DS;                                 /* change mode to velocity control  */
                      Data2 = DynamixelCtlMode_e::VelocityCtl;
                      id = 0x1u;			                              /* id ID1(XM430-W210)  */	  
                      write_dyna_IT(&Data1, &Data2, id);                  /* write via robotis dynamixel control protocol */
					  HAL_Delay(10);
                      Data1 = VELO_GOAL_DS;                               /* goal velocity */
                      Data2 = velocities[v_cnt];                          /* value */
					  v_cnt = (v_cnt + 1) % 10;                           /* choose the next value */
					  /* v_cnt = (v_cnt + 1) % (uint8_t) sizeof(velocities);    choose the next value */
                      id = 0x1u;			                              /* id ID1(XM430-W210)  */	  
				      write_reg_dyna_IT(&Data1, &Data2, id);
                      Data1 = TORQUE_ENAB_DS;                             /* torque enable */
                      Data2 = 1u;                                         /* on */
                      id = 0x1u;			                              /* id ID1(XM430-W210)  */	  
                      write_dyna_IT(&Data1, &Data2, id);                  /* write via robotis dynamixel control protocol */
                  }				  
				  /* save the step to the FATFs for recall on reboot */
                  res = f_open(&fil, fname2.c_str() , FA_CREATE_ALWAYS | FA_WRITE);
                  if (res != FR_OK) return EXIT_FAILURE;
		          sprintf(write_buffer, "step = %d\r\n", step+1);
                  writelen=f_puts(&write_buffer, &fil);
                  if (writelen != strlen(&write_buffer)) return EXIT_FAILURE;
                  res = f_close(&fil);
                  if (res != FR_OK) return EXIT_FAILURE;
                  HAL_Delay(1000);
				  
				  ++step;
               }
			   break;

               case 6:
               {
                  if (device_ping == 0) {                         /* try re-pinging dynamixel if it is offline to poll table */
                      ping_dyna_IT(id); 					  
                  }				  
                  step = 0;                                      /* set the step back to the start again */
               }
               break;
			   
			   default:
			   {
                  step = 0;
			   }
			   break;
           }
		   HAL_Delay(1000);
		}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47999;           /* (MHz divided by a pre-scalar) e.g. if 48Mhz 4800000Hz/47999Hz=1000Hz 8MHz 8000000Hz/8000Hz=1000Hz */
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;               /* timer duration period 1000/1000Hz=1s */
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler();
  }

}

/* timed interrupt -- should be to reset the buffer to a state of looking for a header 
   if it hasnt collected a new message in that time 
   compare chars received since last interrupt if no chars have came in
   and index > header ---> flush the buffer entirely and reset the index */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if (htim == &htim17){
    if ((msgCnt == lstMsgCnt) && (lastMsgCnt != 0)) {
        noActivityForTime = 1;	
    } else {
        noActivityForTime = 0;
    }
    lstMsgCnt = msgCnt;	
 }
}

/**
* @brief This function handles TIM17 global interrupt.
*/
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim)
{
  // Other timer interrupts events..
  //..
  //..
  //..
  /* TIM Update event */
  if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
      #if (USE_HAL_TIM_REGISTER_CALLBACKS == 1)
        htim->PeriodElapsedCallback(htim);
      #else
        HAL_TIM_PeriodElapsedCallback(htim);
      #endif /* USE_HAL_TIM_REGISTER_CALLBACKS */
    }
  }
  //..
  //..
  // Other timer interrupts events..
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspInit 0 */

  /* USER CODE END TIM17_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM17_CLK_ENABLE();
    /* TIM17 interrupt Init */
    HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);
  /* USER CODE BEGIN TIM17_MspInit 1 */

  /* USER CODE END TIM17_MspInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspDeInit 0 */

  /* USER CODE END TIM17_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM17_CLK_DISABLE();

    /* TIM17 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM17_IRQn);
  /* USER CODE BEGIN TIM17_MspDeInit 1 */

  /* USER CODE END TIM17_MspDeInit 1 */
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(USART1_IRQn);

  }
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */

}
// configure UART1 port
static int USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    return -1;
  }
  return 1;
}

// channel 1
void user_tim1_pwm_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_ASSYMETRIC_PWM1;
    sConfigOC.Pulse = (uint32_t)((400)*value);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
}
// channel 2
void user_tim2_pwm_setvalue(float value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_ASSYMETRIC_PWM1;
    sConfigOC.Pulse = (uint32_t)((400)*value);
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

static void encoder_t3_start(void)
{
   HAL_TIM_Encoder_Start( &htim3, TIM_CHANNEL_ALL );
}
static int get_encoder_t3_value(void)
{
	int cnt = TIM3 -> CNT;
	return cnt
}
static void encoder_t2_start(void)
{
   HAL_TIM_Encoder_Start( &htim2, TIM_CHANNEL_ALL );
}
static int get_encoder_t2_value(void)
{
	int cnt = TIM2 -> CNT;
	return cnt
}
/*
   =============== for dynamixel protocol ============================

               dynamixel protocol CRC-16 (IBM/ANSI)
*/
uint16_t update_dyn_crc(uint16_t crc_accum, unsigned char *data_blk_ptr, uint16_t data_blk_size)
{
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

void do_dynamixel_crc(unsigned char* TxPacket, uint16_t data_blk_size, uint8_t * crc_l, uint8_t * crc_h, uint8_t endian)
{
    uint16_t CRC = update_dyn_crc(0, TxPacket , data_blk_size);   // Packet Length
	if (endian == 0)
	{
        crc_l = (CRC & 0x00FF);                              //Little-endian
        crc_h = (CRC>>8) & 0x00FF;
	}
	else
	{
        crc_h = (CRC & 0x00FF);                              //big-endian
        crc_l = (CRC>>8) & 0x00FF;		
	}
}

// goal pos Data1 116 value Data2 e.g. 512
void write_dyna(uint16_t *Data1, uint16_t *Data2, uint8_t id)
{
	int i;
	volatile int j;
	uint8_t TxData[13];
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFD;
	TxData[3] = 0x00;
	TxData[4] = id;
	TxData[5] = 0x09;
	TxData[6] = 0x00;
	TxData[7] = 0x03;                                     // write
	TxData[8] = 0x00;
	TxData[9] = Data1[0];
	TxData[10] = Data1[1];
	TxData[11] = Data2[0];
	TxData[12] = Data2[1];
	TxData[13] = Data2[2];
	TxData[14] = Data2[3];
    do_dynamixel_crc(&TxData, 11u, &crcl, &crch, 0);
	TxData[15] = crcl;
	TxData[16] = crch;
	HAL_UART_Transmit(&huart1,TxData,17,1);            
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
// goal pos Data1 116 value Data2 e.g. 512 using interrupt
void write_dyna_IT(uint16_t *Data1, uint16_t *Data2, uint8_t id)
{
	int i;
	volatile int j;
	uint8_t crcl = 0;
	uint8_t crch = 0;
	HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_RESET);
	TxDataIT[0] = 0xFF;
	TxDataIT[1] = 0xFF;
	TxDataIT[2] = 0xFD;
	TxDataIT[3] = 0x00;
	TxDataIT[4] = id;
	TxDataIT[5] = 0x09;
	TxDataIT[6] = 0x00;
	TxDataIT[7] = 0x03;                                     // write
	TxDataIT[8] = 0x00;
	TxDataIT[9] = Data1[0];
	TxDataIT[10] = Data1[1];
	TxDataIT[11] = Data2[0];
	TxDataIT[12] = Data2[1];
	TxDataIT[13] = Data2[2];
	TxDataIT[14] = Data2[3];
    do_dynamixel_crc(&TxDataIT, 11u, &crcl, &crch, 0);
	TxDataIT[15] = crcl;
	TxDataIT[16] = crch;
	HAL_UART_Transmit_IT(&huart1,TxDataIT,17,1);            
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
// goal velocity Data1 104 value Data2 e,g, 200
void write_reg_dyna(uint16_t *Data1, uint16_t *Data2, uint8_t id)
{
	int i;
	volatile int j;
	uint8_t TxData[13];
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFD;
	TxData[3] = 0x00;
	TxData[4] = id;
	TxData[5] = 0x09;
	TxData[6] = 0x00;
	TxData[7] = 0x04;                                     // write reg
	TxData[8] = 0x00;
	TxData[9] = Data1[0];
	TxData[10] = Data1[1];
	TxData[11] = Data2[0];
	TxData[12] = Data2[1];
	TxData[13] = Data2[2];
	TxData[14] = Data2[3];
    do_dynamixel_crc(&TxData, 11u, &crcl, &crch, 0);
	TxData[15] = crcl;
	TxData[16] = crch;
	HAL_UART_Transmit(&huart1,TxData,17u,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
void write_reg_dyna_IT(uint16_t *Data1, uint16_t *Data2, uint8_t id)
{
	int i;
	volatile int j;
	uint8_t crcl = 0;
	uint8_t crch = 0;
	HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_RESET);
	TxDataIT[0] = 0xFF;
	TxDataIT[1] = 0xFF;
	TxDataIT[2] = 0xFD;
	TxDataIT[3] = 0x00;
	TxDataIT[4] = id;
	TxDataIT[5] = 0x09;
	TxDataIT[6] = 0x00;
	TxDataIT[7] = 0x04;                                     // write reg
	TxDataIT[8] = 0x00;
	TxDataIT[9] = Data1[0];
	TxDataIT[10] = Data1[1];
	TxDataIT[11] = Data2[0];
	TxDataIT[12] = Data2[1];
	TxDataIT[13] = Data2[2];
	TxDataIT[14] = Data2[3];
    do_dynamixel_crc(&TxDataIT, 11u, &crcl, &crch, 0);
	TxDataIT[15] = crcl;
	TxDataIT[16] = crch;
	HAL_UART_Transmit_IT(&huart1,TxDataIT,17u,1);            
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
void factory_reset_dyna(uint8_t id)
{
	int i;
	volatile int j;
	uint8_t TxData[11];
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFD;
	TxData[3] = 0x00;
	TxData[4] = id;
	TxData[5] = 0x04;
	TxData[6] = 0x00;
	TxData[7] = 0x06;                                     // fr
	TxData[8] = 0x01;
    do_dynamixel_crc(&TxData, 9u, &crcl, &crch, 0);
	TxData[9] = crcl;
	TxData[10] = crch;
	HAL_UART_Transmit(&huart1,TxData,11,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
void reboot_dyna(uint8_t id)
{
	int i;
	volatile int j;
	uint8_t TxData[10];
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFD;
	TxData[3] = 0x00;
	TxData[4] = id;
	TxData[5] = 0x04;
	TxData[6] = 0x00;
	TxData[7] = 0x08;                                     // reboot
	TxData[8] = 0x2F;
    do_dynamixel_crc(&TxData, 9u, &crcl, &crch, 0);
	TxData[10] = crcl;
	TxData[11] = crch;
	HAL_UART_Transmit(&huart1,TxData,12,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
void ping_dyna(uint8_t id)
{
	int i;
	volatile int j;
	uint8_t TxData[9];
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxData[0] = 0xFF;
	TxData[1] = 0xFF;
	TxData[2] = 0xFD;
	TxData[3] = 0x00;
	TxData[4] = id;
	TxData[5] = 0x03;
	TxData[6] = 0x00;
	TxData[7] = 0x01;                                     // reboot
    do_dynamixel_crc(&TxData, 8u, &crcl, &crch, 0);
	TxData[10] = crcl;
	TxData[11] = crch;
	HAL_UART_Transmit(&huart1,TxData,12,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
void ping_dyna_IT(uint8_t id)
{
	int i;
	volatile int j;
	uint8_t crcl = 0;
	uint8_t crch = 0;
	TxDataIT[0] = 0xFF;
	TxDataIT[1] = 0xFF;
	TxDataIT[2] = 0xFD;
	TxDataIT[3] = 0x00;
	TxDataIT[4] = id;
	TxDataIT[5] = 0x03;
	TxDataIT[6] = 0x00;
	TxDataIT[7] = 0x01;                                     // reboot
    do_dynamixel_crc(&TxDataIT, 8u, &crcl, &crch, 0);
	TxDataIT[10] = crcl;
	TxDataIT[11] = crch;
	HAL_UART_Transmit_IT(&huart1,TxDataIT,10u,1);
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
// read the current position
void read_dyna_pos_IT(uint8_t id)
{
	int i;
	volatile int j;
	uint8_t crcl = 0;
	uint8_t crch = 0;
	new_pos = 0;
	TxDataIT[0] = 0xFF;
	TxDataIT[1] = 0xFF;
	TxDataIT[2] = 0xFD;
	TxDataIT[3] = 0x00;
	TxDataIT[4] = id;
	TxDataIT[5] = 0x07;
	TxDataIT[6] = 0x00;
	TxDataIT[7] = 0x02;                                     // read
	TxDataIT[8] = 0x00;
	TxDataIT[9] = 0x84;
	TxDataIT[10] = 0x00;
	TxDataIT[11] = 0x04;
	TxDataIT[12] = 0x00;
    do_dynamixel_crc(&TxDataIT, 11u, &crcl, &crch, 0);
	TxDataIT[13] = crcl;
	TxDataIT[14] = crch;
	HAL_UART_Transmit_IT(&huart1,TxDataIT,15u,1);            
	//HAL_Delay(1);
	for(j=0;j<SENDDELAYCOUNT;j++);
}
/* check if the char is a numerical digit */
static int16_t isdigitanum(unsigned char ch) 
{
  return ch >= '0' && ch <= '9';
}

/*--- Interrupt for UART1 rcv buffer Robotis Dyanmixel protocol ---*/
#define FULL_STAT_MSG_LEN 11
#define FULL_PINGBACK_MSG_LEN 14
#define FULL_READBK_MSG_LEN 15
#define STAT_HEADER_LEN 5
#define POS_SET_MSG_LEN 17
#define POS_RD_MSG_LEN 14
#define PING_MSG_LEN 10
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle){}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    volatile int j=0;
    HAL_UART_Receive_IT(&huart1, &RxData, 1);
	msgCnt = (msgCnt+1) % UINT32_MAX;                                                                                            /* count when we are receiving chars */
	if (noActivityForTime == 1) {                                                                                                /* receive buffer timeout occurred */
        _index = 0;                                                                                                              /* reset the buffer pointer as message must be discarded no activity within timeout */
		noActivityForTime = 0;
    }		
	uint8_t Data = RxData;                                                                                                       // interrupt receive byte
			
	if(_index >= (STAT_HEADER_LEN+1)){                                                                                            // we just saw a valid header in the buffer so start collection
		RxBuff[_index-1] = Data;
		_index++;
		if(_index == STAT_HEADER_LEN+4) {                                                                                       /* we have collected the next part of the message */
            len_msg = (RxBuff[STAT_HEADER_LEN+1] << 8) | RxBuff[STAT_HEADER_LEN];                                               /* length */
            inst = RxBuff[STAT_HEADER_LEN+2];                                                                                   /* instance */
            err = RxBuff[STAT_HEADER_LEN+3];			                                                                        /* error byte */
        }
		if(_index == FULL_STAT_MSG_LEN) {                                                                                        // full message was received
			if ((len_msg == 4) && (inst == 0x55)) {                                                                              // it is a status message from a write or write reg command
			    _index = 0;                                                                                                // re-start receive collection
				uint8_t crc1, crch;
                do_dynamixel_crc(&RxBuff, 9, &crcl, &crch, 0);
                if ((crcl == RxBuff[STAT_HEADER_LEN+4]) && (crcl == RxBuff[STAT_HEADER_LEN+5])) {                                 // crc matches
					if (err != 0) {
						if (err == DynamixelError_e::CRCError) {                                                                  // error was with the transmitted crc
                            HAL_UART_Transmit_IT(&huart1, TxDataIT, POS_SET_MSG_LEN, 1);                                          // retransmit
                        } else if (err == DynamixelError_e::DataRangeErr) { 						                              // DataRangeErr is sent upon a watchdog failure
						    memcpy(&TxDataITMem,&TxDataIT,20);                                                                    // remember what we are sending
                            Data1 = BUS_WDOG_DS;                                                                                  /* set bus watchdog */
                            Data2 = 0u;                                                                                           /* clear the wdog */
                            id = 0x1u;			                                                                                  /* id ID1(XM430-W210)  */	  
                            write_dyna_IT(&Data1, &Data2, id);                                                                    /* write via robotis dynamixel control protocol */
                            HAL_UART_Transmit_IT(&huart1, TxDataITMem, POS_SET_MSG_LEN, 1);                                       // retransmit							
                        } else {                                                                                                  // malformed message transmitted
                            if (re_trans1 == 0u) {                                                                                // try the packet once more
                               HAL_UART_Transmit_IT(&huart1, TxDataIT, POS_SET_MSG_LEN, 1);
                               re_trans1 = 1;
                            } else {                                                                                               // twice same error
                               re_trans1 = 0;
                        	}
						}
                        HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_RESET);	                                   // led off means we have an error with the transmit packet					
					} else {
                        HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_SET);                                         // ack that we wrote correctly to the drive
                    }						
                } else {
                    HAL_UART_Transmit_IT(&huart1, TxDataIT, POS_SET_MSG_LEN, 1);                                                    /* retransmit */	
                    HAL_GPIO_WritePin(StateLED_GPIO_Port, StateLED_Pin, GPIO_PIN_RESET);					
                }				
			} else {
                /* search the buffer for a new start transmission and move the message if found becasue it was a partial message overlapped  */
				for (int z=STAT_HEADER_LEN+1; z < (FULL_STAT_MSG_LEN-STAT_HEADER_LEN); z++) {                                          /* parse the buffer without the header */
			        if (((RxBuff[z]==0xFF && RxBuff[z+1]==0xFF) && (RxBuff[z+2]==0xFD && RxBuff[z+3]==0x00)) && (RxBuff[z+4]==id)) {   /* check for another header (message partial) */
                        for (int x=z; x <= (FULL_STAT_MSG_LEN-1); x++) {                                                               /* for the remaining message */
                            RxBuff[x-z] = RxBuff[x];                                                                                   /* move the buffer up */                                  
                        }
						_index = (x-z)+1;                                                                                              /* set the index to the end of this new message */
						break;                                                                                                         /* exit the parse loop */
                    }						
                }
            }
		}
		if(_index == FULL_PINGBACK_MSG_LEN) {                                                                                      // full message was received
			if ((len_msg == 7) && (inst == 0x55)) {                                                                                // it is a ping reply message
                _index = 0;                                                                                               /* reset the collector as we got a good msg */
				uint8_t crc1, crch;
                do_dynamixel_crc(&RxBuff, 12, &crcl, &crch, 0);
                if ((crcl == RxBuff[STAT_HEADER_LEN+8]) && (crcl == RxBuff[STAT_HEADER_LEN+9])) {                                  // crc matches
					if (err != 0) {
                        device_ping = 0;	
						if (err == DynamixelError_e::CRCError) {                                                                  // error was with the transmitted crc
                            ping_dyna_IT(id);                                                                                     // retransmit					
                        } else {                                                                                                  // malformed message transmitted
                            if (re_trans2 == 0u) {                                                                                 // try the packet once more
                               ping_dyna_IT(id);
                               re_trans2 = 1;
                            } else {                                                                                               // twice same error
                               re_trans2 = 0;
                        	}						   
                        }							
					} else {
                        device_ping = 1;
                    }						
                } else {
                    //HAL_UART_Transmit_IT(&huart1, TxDataIT, 12u, 1);                                                            // retransmit	
                    ping_dyna_IT(id);
                    device_ping = 0;					
                }				
			} else {
                /* search the buffer for a new start transmission and move the message if found */
				for (int z=STAT_HEADER_LEN+1; z < (FULL_PINGBACK_MSG_LEN-STAT_HEADER_LEN); z++) {                                          /* parse the buffer without the header */
			        if (((RxBuff[z]==0xFF && RxBuff[z+1]==0xFF) && (RxBuff[z+2]==0xFD && RxBuff[z+3]==0x00)) && (RxBuff[z+4]==id)) {       /* check for another header (message partial) */
                        for (int x=z; x <= (FULL_PINGBACK_MSG_LEN-1); x++) {                                                               /* for the remaining message */
                            RxBuff[x-z] = RxBuff[x];                                                                                       /* move the buffer up */                                  
                        }
						_index = (x-z)+1;                                                                                                  /* set the index to the end of this new message */
						break;                                                                                                             /* exit the parse loop */
                    }						
                }
            }
		}
		if(_index == FULL_READBK_MSG_LEN) {                                                                                        // full message was received (longest it resets collection if invalid)
			if ((len_msg == 8) && (inst == 0x55)) {                                                                                // it is a position read reply message
			    _index = 0;                                                                                                            /* reset collector on longest possible message receive */
				uint8_t crc1, crch;
                do_dynamixel_crc(&RxBuff, 13, &crcl, &crch, 0);
                if ((crcl == RxBuff[STAT_HEADER_LEN+9]) && (crcl == RxBuff[STAT_HEADER_LEN+10])) {                                  // crc matches
					if (err != 0) {
                        new_pos = 0;	
						if (err == DynamixelError_e::CRCError) {                                                                  // error was with the transmitted crc
                            read_dyna_pos_IT(id);                                                                                 // retransmit
                        } else {                                                                                                  // malformed message transmitted
                            if (re_trans3 == 0u) {                                                                                 // try the packet once more
                               read_dyna_pos_IT(id);
                               re_trans3 = 1;
                            } else {                                                                                               // twice same error
                               position_readbk = 0;                                                                                // default the readback
                               new_pos = 1;                                                                                        // advance sequence without position as message was wrong
                               re_trans3 = 0;
                        	}						   
                        }						
					} else {
						position_readbk = (RxBuff[STAT_HEADER_LEN+8] << 24) | (RxBuff[STAT_HEADER_LEN+7] << 16) | (RxBuff[STAT_HEADER_LEN+6] << 8) | (RxBuff[STAT_HEADER_LEN+5]);
                        new_pos = 1;                                                                                               // request complete
                    }						
                } else {
                    //HAL_UART_Transmit_IT(&huart1, TxDataIT, 15u, 1);                                                   
                    read_dyna_pos_IT(id);					
                    new_pos = 0;					
                }				
			} else {                                                                                                                /* no match was found */
                uint8_t new_heder_found = 0;
                /* search the buffer for a new start transmission and move the message if found */
				for (int z=STAT_HEADER_LEN+1; z < (FULL_READBK_MSG_LEN-STAT_HEADER_LEN); z++) {                                          /* parse the buffer without the header */
			        if (((RxBuff[z]==0xFF && RxBuff[z+1]==0xFF) && (RxBuff[z+2]==0xFD && RxBuff[z+3]==0x00)) && (RxBuff[z+4]==id)) {     /* check for another header (message partial) */
                        for (int x=z; x <= (FULL_READBK_MSG_LEN-1); x++) {                                                               /* for the remaining message */
                            RxBuff[x-z] = RxBuff[x];                                                                                     /* move the buffer up */                                  
                        }
						_index = (x-z)+1;                                                                                                /* set the index to the end of this new message */
                        new_heder_found = 1;
						break;                                                                                                           /* exit the parse loop */
                    }						
                }
                if (new_heder_found == 0) {                                                                                              /* no header found leave the last 5 bytes in the buffer */
			        _index = STAT_HEADER_LEN;                                                                                          // prepare another 5 byte sequence from the last 5 bytes
				    for (int z=0; z <= (STAT_HEADER_LEN-1); z++) {
                        RxBuff[z] = RxBuff[z+(FULL_READBK_MSG_LEN-5)];                                                                 // shuffle rcv buffer to last 5 chars
                    }
                }
                HAL_UART_Transmit_IT(&huart1, TxDataIT, 15u, 1);	                                                               // retransmit as we got some garbled info			
            }				
		}
	} else {                                                                                                                       // detect the start of a valid message
		RxBuff[_index] = Data;
		_index++;
		if(_index == STAT_HEADER_LEN){                                                                                             // 5 byte header received		
			if (((RxBuff[0]==0xFF && RxBuff[1]==0xFF) && (RxBuff[2]==0xFD && RxBuff[3]==0x00)) && (RxBuff[4]==id)) {              // dynamixel message from the address 0x1 
			    _index = STAT_HEADER_LEN + 1;
			} else {
			    _index = STAT_HEADER_LEN - 1;                                                                                      // prepare another 5 byte sequence if not at start of message
				for (int z=0; z <= (STAT_HEADER_LEN-1); z++) {
                    RxBuff[z] = RxBuff[z+1];                                                                                       // shuffle rcv buffer
                }				
			}
		}	
	}
}
void waitRxInterrupt(){
	//while(1){
	//	if(RxBuff[0] == 0xFF && RxBuff[1]==0xFF)break;
	//}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
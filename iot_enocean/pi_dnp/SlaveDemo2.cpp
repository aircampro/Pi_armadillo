// ref:- https://github.com/gec/dnp3/tree/master
//
// Licensed to Green Energy Corp (www.greenenergycorp.com) under one
// or more contributor license agreements. See the NOTICE file
// distributed with this work for additional information
// regarding copyright ownership.  Green Enery Corp licenses this file
// to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// software distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.
//
#include "SlaveDemo.h"

// define your raspi i/o
#define MY_OUT 24                                // drive the output
#define MY_OUT_FB 23                             // this output feedback
#define MY_PU_IN 25                              // pull-up DIN feedback
#define MY_PD_IN 18                              // pull-dwn DIN feedback

#define MY_ANI_UR 100.0                          // analog inputs upper range in our case speed

#if defined(LCD_ATTACHED)
    // LCD Display attached 
    CLCD display;
#elif defined(OLED_ATTACHED)
    // OLED Display attached 
    OLED display;
#endif

// PID loop
double mPidSpt;             // PID Loop setpoint
PID mPid;                   // PID object
double mPidIn;              // measured input
double mPidOut;             // PID output
ADC mAdc;                   // analog to digital conv object
int mAdcRaw[8];             // 8 channel raw analog read over SPI bus
int mRawPidOut;             // PID output representeed as RAW counts 0-4095
#if defined(PULSE_OUT)
    bool m_timer_act = false;   // global to record whether internal timer is running for pulse outputs
    boost::timer m_t;           // boost timer for pulse output
#endif

#define PID_LOOP_TIME 2.0            // loop time for the PID loop s
boost::timer m_pid_loop_t;           // boost timer for reading AIN doing PID and writing AOT
	
// global run flag for the PID processing
int mRunPid = 1;

// define the i2c bus which the DAC is attached e.g. /dev/i2c-1
const U08 m_i2c_dac_bus 1

using namespace std;

namespace apl
{
namespace dnp
{

SlaveDemoBase::SlaveDemoBase(Logger* apLogger) :
	IOService(),
	IOServiceThread(apLogger, this->Get()),
	mTimerSource(this->Get()),
	mpInfiniteTimer(mTimerSource.StartInfinite())
{
	// Create a notifier that when called will post a call to OnCommandNotify
	INotifier* pNotifier = mPostSource.Get(boost::bind(&SlaveDemoBase::OnCommandNotify, this), &mTimerSource);

	// Hand the notifier to the command source, so this happens whenever a new command is present
	mCommandQueue.SetNotifier(pNotifier);
}

void SlaveDemoBase::Shutdown()
{
	// this is the only outstanding event, so this will cause the
	// io_service thread to exit cleanly
    mpInfiniteTimer->Cancel();
	// close the open i2c device and then spi device
    mRunPid = 0;
    usleep(100);
    I2cCtl_Close();
    mAdc.quit();
}

// initialise the raspi gpio
void SlaveDemoBase::RaspiGpioInit()
{
    gpio_init();                                  //  initialise the gpio
    gpio_configure(MY_OUT, GPIO_OUTPUT);          //  GPIO_24 is output
    gpio_configure(MY_PU_IN, GPIO_INPUT);         //  GPIO_25 input pull-up
    gpio_configure_pull(MY_PU_IN, GPIO_PULLUP);   //  define pull-up
    gpio_configure(MY_PD_IN, GPIO_INPUT);         //  GPIO_18 pull down
    gpio_configure_pull(MY_PD_IN, GPIO_PULLDOWN); //  define pull-down	
    gpio_configure(MY_OUT_FB, GPIO_INPUT);        //  GPIO_23 input pull-up 
    gpio_configure_pull(MY_OUT_FB, GPIO_PULLUP);   
#if defined(LCD_ATTACHED)
    display.init();
#elif defined(OLED_ATTACHED)
    OLED display;
    display.init(OLED_SPI);
#endif
}

void SlaveDemoBase::PidLoopInit(double p, double i, double d)
{
    mPid.Init(p,i,d);                   // make PID loop object with p i d as paramters
    mAdc.init(ADC_SPI, ADC_3208);       // SPI_DEVICE with ADC attached
	I2cCtl_Init(m_i2c_dac_bus);         // I2C bus with DAC attached
    for (int i = 0; i < 8; i++)
        mAdcRaw[i] = 0.0;	
}

void SlaveDemoBase::ReadAllMeasInput()
{
    for (int i = 0; i < 8; i++)
        mAdcRaw[i] = mAdc.get(i);
}

int SlaveDemoBase::ReadMeasInput( int chan_no )
{
    mAdcRaw[chan_no] = mAdc.get(chan_no);
    if (mAdcRaw[chan_no] == -1) {                  // on error retry
        mAdc.init(ADC_SPI, ADC_3208);              // re-init the ADC with the specified parameters 
        usleep(10000);
        mAdcRaw[chan_no] = mAdc.get(chan_no);      // get the data return -1 if there is still and error		
    }
	return mAdcRaw[chan_no];
}

double SlaveDemoBase::ScaleInput(int inp, double range)
{
    return (range * (static_cast<double>(inp) / 4095.0));
}

int SlaveDemoBase::ScaleOutput(double inp, double range)
{
    return static_cast<int>(4095.0 * (static_cast<double>(inp) / range));
} 	

void SlaveDemoBase::doPidLoop()
{
    if (ReadMeasInput(0) != -1) {
        mPidIn = ScaleInput(mAdcRaw[0], MY_ANI_UR);
        mPid.UpdateSpeedError(mPidIn, mPidSpt);
        mPidOut = mPid.TotalError(MY_ANI_UR, 0.0);
	    mRawPidOut = ScaleOutput(mPidOut, MY_ANI_UR);
	    if (setRawDACValue(static_cast<uint16_t>(mRawPidOut), MCP4725_FastMode, MCP4725_PowerDown_Off, m_i2c_dac_bus) == false) {
            I2cCtl_Close();  
            I2cCtl_Init(m_i2c_dac_bus);
	        if (setRawDACValue(static_cast<uint16_t>(mRawPidOut), MCP4725_FastMode, MCP4725_PowerDown_Off, m_i2c_dac_bus) == false) {
                printf("Error writing to the DAC");
            }
        }
    } else {
        printf("Error reading the ADC");		
    } 		
}

void SlaveDemoBase::RunPidLoop()
{
	while(mRunPid) {
		if (m_pid_loop_t.elapsed() >= PID_LOOP_TIME) {
            doPidLoop();
            m_pid_loop_t.restart();
        }
        usleep(100);
	}
}
		  
void SlaveDemoBase::OnCommandNotify()
{
	// execute a single command
	mCommandQueue.ExecuteCommand(this);
	
	// run the pid loop (seems like only above can run so i will do this via a thread in the Main)
    // RunPidLoop();
}

SlaveDemoApp::SlaveDemoApp(Logger* apLogger) :
	SlaveDemoBase(apLogger),
	mCountSetPoints(0),
	mCountBinaryOutput(0),
	mpObserver(NULL)
{}

void SlaveDemoApp::SetDataObserver(IDataObserver* apObserver)
{
	mpObserver = apObserver;
}

CommandStatus SlaveDemoApp::HandleControl(Setpoint& aControl, size_t aIndex)
{
	LOG_BLOCK(LEV_INFO, "Received " << aControl.ToString() << " on index: " << aIndex);

	// set the PID setpoint to the same value as the setpoint we were
	// given from the master. reply with DIN and Configure it with the current time and good quality
	std:: cerr << " master sent analog value @ " << aControl.GetValue() << std::endl;
	mPidSpt = aControl.GetValue();
#if defined(LCD_ATTACHED)
    display.init();
	char line[LINE_LEN + 1];
    for (int i = 0; i < LINE_LEN; i++)
        line[i] = ' ';
    display.locate(2, 0);
	int result = snprintf(&line,  sizeof(line), "%.2f", aControl.GetValue());
    if (result >= sizeof(line)) {
        std::cerr << "Warning: Truncated output!" << std::endl;
    }
    display.print(line);
    display.locate(0, 1);
    display.print("= Raspberry Pi =");
#elif defined(OLED_ATTACHED)
    display.clear(display.color(255, 255, 255));
    display.text( 0, 12, (char *) "Setpoint from DNP", display.color(128, 255, 128) );
	char line[LINE_LEN + 1];
    for (int i = 0; i < LINE_LEN; i++)
        line[i] = ' ';
	int result = snprintf(&line,  sizeof(line), "%.2f", aControl.GetValue());
    if (result >= sizeof(line)) {
        std::cerr << "Warning: Truncated output!" << std::endl;
    }
    display.text( 9, 40, &line, display.color(128, 255, 128));	
#endif	

    int val = gpio_read(MY_PU_IN);            
    printf("input(PU_IN): %d\n", val); 
    int val1 = gpio_read(MY_PD_IN);            
    printf("input(PD_IN): %d\n", val1); 
	double val2 = (static_cast<double>(val) + (static_cast<double>(val1)*2.0f);
	Analog a(val2, AQ_ONLINE);
	a.SetToNow();

	// Create an additional counter to let the master know how many setpoints
	// we've receieved
	Counter c(++mCountSetPoints, CQ_ONLINE);
	c.SetToNow();

	// We would like all updates to be sent at one time.When the Transaction object
	// goes out of scope, all updates will be sent in one block to do the slave database.
	Transaction t(mpObserver);

	// Push the prepared datapoints to the database of this slave. The slave
	// can now transmit the changes to the master (polling or unsol)
	mpObserver->Update(a, aIndex);
	mpObserver->Update(c, 0);

	// This is the control code returned to the slave stack, and forwared
	// on to the master. These are DNP3 control codes.
	return CS_SUCCESS;
}

// same as for the setpoint
// read the command code and set the corresponding pi output
// read the pi input and send it back as the binary
CommandStatus SlaveDemoApp::HandleControl(BinaryOutput& aControl, size_t aIndex)
{
	LOG_BLOCK(LEV_INFO, "Received " << aControl.ToString() << " on index: " << aIndex);

	// set the binary to ON if the command  code was LATCH_ON, otherwise set it off (LATCH_OFF)
	// PULSE_OUT mode do a one shot rising edge pulse for period of PLS_DURATION (s)
#if defined(PULSE_OUT)
    if ((aControl.GetCode() == CC_LATCH_ON) && (m_timer_act == false)) {
        gpio_set(MY_OUT);                       //  1 =（3.3V）
        m_t.restart();
		m_timer_act = true;
    } 
    while (m_t.elapsed() <= PLS_DURATION) {
        usleep(50);  
        if (aControl.GetCode() != CC_LATCH_ON) {
            break;
        }			
    }
    if (m_t.elapsed() >= PLS_DURATION) {
	    gpio_clear(MY_OUT);     		         //  0 =（0V） pulse duration is expired	
    }
    if (aControl.GetCode() != CC_LATCH_ON) {
        gpio_clear(MY_OUT);     		         //  0 =（0V） command from other side to reset			
        m_timer_act = false;
    } 		
#else
    if (aControl.GetCode() == CC_LATCH_ON) {
        gpio_set(MY_OUT);                       //  1 =（3.3V）
    } else {
		gpio_clear(MY_OUT);                     //  0 =（0V）
    }
#endif
    usleep(10000);
    int val = gpio_read(MY_OUT_FB); 
	apl::Binary b((val & 0x1), BQ_ONLINE);
	b.SetToNow();

	// count how many BinaryOutput commands we recieve
	apl::Counter c(++mCountBinaryOutput, CQ_ONLINE);
	c.SetToNow();

	Transaction t(mpObserver);
	mpObserver->Update(b, aIndex);
	mpObserver->Update(c, 1);

	return CS_SUCCESS;
}

}
}
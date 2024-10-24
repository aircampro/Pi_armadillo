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
#include "GpioPi.h"

#if defined(LCD_ATTACHED)
// LCD Display attached 
#include "raspCLCD.h"
#include <cstdio>
#define LINE_LEN 16 
CLCD display;
#endif

// define your raspi i/o
#define MY_OUT 24                                // drive the output
#define MY_OUT_FB 23                             // this output feedback
#define MY_PU_IN 25                              // pull-up DIN feedback
#define MY_PD_IN 18                              // pull-dwn DIN feedback

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
#endif
}

void SlaveDemoBase::OnCommandNotify()
{
	// execute a single command
	mCommandQueue.ExecuteCommand(this);
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

	// Update a  feedback point that has the same value as the setpoint we were
	// given from the master. Configure it with the current time and good quality
	std:: cerr << " master sent analog value @ " << aControl.GetValue() << std::endl;
#if defined(LCD_ATTACHED)
    display.init();
	char line[LINE_LEN + 1];
    for (int i = 0; i < LINE_LEN; i++)
        line[i] = ' ';
    display.locate(2, 0);
	int result = snprintf(&line,  sizeof(line), "%.2f", aControl.GetValue());
    if (result >= sizeof(buffer)) {
        std::cerr << "Warning: Truncated output!" << std::endl;
    }
    display.print(line);
    display.locate(0, 1);
    display.print("= Raspberry Pi =");
#endif	
    int val = gpio_read(MY_PU_IN);            
    printf("input(PU_IN): %d\n", val); 
    int val1 = gpio_read(MY_PD_IN);            
    printf("input(PD_IN): %d\n", val1); 
	float val2 = static_cast<float>(val) + (static_cast<float>(val1)*2.0f)
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
    if (aControl.GetCode() == CC_LATCH_ON) {
        gpio_set(MY_OUT);                   //  1 を出力（3.3V）
    } else {
		gpio_clear(MY_OUT);                 //  0 を出力（0V）
    }
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
//
// Sample poco library calls to run an interpretaed script program
// We run python to reset the usb power line
//
// Compiles as :- g++-8 -I/usr/local/include -L/usr/local/lib -lPocoFoundation poco_script_runner.cpp
// Runs as :-     env LD_LIBRARY_PATH=/usr/local/lib ./a.out 
//
#ifndef __poco_script_runner
#define __poco_script_runner

#include <iostream>
#include "Poco/Process.h"
#include "Poco/PipeStream.h"

void run_script(char const* interpreter, char const* filename, int times_to_run)
{
    std::vector<std::string> args{ filename };
    Poco::Pipe outPipe;
    Poco::Pipe inPipe;
    Poco::ProcessHandle process_handle = Poco::Process::launch(interpreter, args, &inPipe, &outPipe, nullptr/*errPipe*/);
    Poco::PipeInputStream output_reader(outPipe);
    Poco::PipeOutputStream input_writer(inPipe);
    for (int repeat_counter = 0; repeat_counter < times_to_run; ++repeat_counter)
    {
        auto send_str("running the interpreted script....");
        input_writer << send_str << std::endl;
        std::cout << interpreter << " " << send_str << " " << std::endl;
        std::cout.flush();
        std::string receiv_str;
        output_reader >> receiv_str;
        std::cout << interpreter << " " << receiv_str << " " << std::endl;
    }
}

int reset_usb_sony(void)
{
    run_script("python3", "/home/anthony/latest/Airobot-Dynamics-UK/DronePayloadManager/reset_usb.py", 1);
    return 0;
}

#endif
// Compiles with : g++-8 -std=c++17 -o m m.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK 
// -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer 
// -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I/home/mark/boost/boost_1_79_0
//
// This shows an example of a thread sheduler using boost each daemon is started and runs via commands to the global
// variables as they are set.
//
#include <boost/thread/thread.hpp>//Include boost header file
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <cstdlib>
boost::mutex io_mutex;

using namespace std;

volatile bool isRuning = true;
volatile bool isRuning2 = true;
volatile int picTake = 0;
volatile int globalNumber = 23;

// in reality this will be the sony API thread making usb connection and then reading a global to do the action
// opt is the start-up arguments that are taken at initialisation of the thread
//
void sony_camera( int opt )
{
   while(isRuning)
   {
      boost::mutex::scoped_lock lock(io_mutex);
      if (globalNumber % 2)
          std::cout << "\033[33m set camera option to : \033[0m" << opt << " " << globalNumber << endl;
      sleep(5);
   }
   sleep(2);
}

// this is the example of doing something with the micasense camera
// using the http API
//
void micasense_camera()
{
    static int cnt1 = 0;
    while(isRuning)
    {
       cout << "pic take :: " << picTake << endl;
       if ((picTake % 3) == 0)
           cout << "\033[31m picture being taken: \033[0m" << ++cnt1  << " " << picTake << endl;
       sleep(2);
       ++picTake;
    }
    sleep(2);
}

// htis is an example of sending a timed heartbeat
//
void send_heartbeat()
{
    static int cnt2 = 0; 
    while(isRuning2)
    {
        cout << "\033[32m send heartbeat: \033[0m" << ++cnt2 << endl;
        sleep(1);
	++globalNumber;
    }
}

int initialise_mavlink()
{
	cout << "initialised conenction to mavlink" << endl;
	return 30;
}

// mavlink reader
void read_mavlink( int handle )
{
    static int cnt1 = 0;
    while(isRuning)
    {
       if ((cnt1 % 10) == 0)
           cout << "\033[34;43m Reading Mavlink Data: \033[0m" << ++cnt1  << " " << handle << endl;
           sleep(1);
           ++cnt1;
    }
    sleep(2);
}

// ====================================== main thread ============================================================
//
int main(void)
{
	int mavHandle = initialise_mavlink();
    while (isRuning == true)
    {
        ++picTake;
	    ++globalNumber;
	    isRuning = true;
        boost::thread thread0(boost::bind(&read_mavlink, mavHandle));
        boost::thread thread1(&micasense_camera);
        boost::thread thread2(&send_heartbeat);
	    //arg = std::to_string(picTake);
        boost::thread thread3(boost::bind(&sony_camera, globalNumber));
        system("read");
	    cout << "\033[35m started all the threads \033[0m" << endl;
	    thread3.join();
	    thread2.join();
	    thread1.join();
	    thread0.join();
    }

    return 0;
}

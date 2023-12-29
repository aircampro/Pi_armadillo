#include <iostream>
#include <thread>
#include <chrono>
#include "timer_wheel.h"
#include "min_heap.h"
#include <chrono>

void TimerHandler()
{

    std::chrono::steady_clock::duration d =
            std::chrono::steady_clock::now().time_since_epoch();

    std::chrono::microseconds mic = std::chrono::duration_cast<std::chrono::microseconds>(d);

    std::cout << "Timer:"<<mic.count() << std::endl;
}

typedef void (*Function)();

int main()
{
//    MinHeap::TimerManager tm;
//    MinHeap::Timer t(tm);
    TimerWheel::TimerManager tm;
    TimerWheel::Timer t(tm);
    t.Start(&TimerHandler, 1000);
    while (true)
    {
        tm.DetectTimers();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cin.get();
    return 0;
}

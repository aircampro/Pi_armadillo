#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/chrono/duration.hpp>
#include <boost/chrono/chrono_io.hpp>
#include <boost/chrono/floor.hpp>
#include <boost/chrono/round.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/asio/basic_waitable_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono/chrono.hpp>
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/chrono/floor.hpp>
#include <boost/chrono/round.hpp>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/system_timer.hpp>

namespace asio = boost::asio;
namespace chrono1 = boost::chrono;

void on_timer(const boost::system::error_code& ec)
{
    if (!ec) {
        std::cout << "on timer" << std::endl;
    }
}

int main()
{
    asio::io_service io_service;
    asio::steady_timer timer(io_service); // chronoベースのタイマー

    //timer.expires_from_now(boost::posix_time::seconds(2));
    timer.expires_from_now(std::chrono::milliseconds(200));
//    timer.expires_from_now(boost::chrono::milliseconds(100)); // chronoのdurationを渡す
    timer.async_wait(on_timer);

    io_service.run();
}

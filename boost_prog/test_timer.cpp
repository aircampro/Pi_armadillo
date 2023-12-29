#include <iostream>
#include <string>
#include <ctime>
#include <boost/asio.hpp>
#include <boost/format.hpp>
#include <boost/current_function.hpp>

namespace asio = boost::asio;

std::string now()
{
    std::time_t t;
    std::time(&t);
    std::tm* st = std::localtime(&t);

    return (boost::format(" %1%/%2%/%3% %4%:%5%:%6% : ")
                % (1900 + st->tm_year)
                % (1 + st->tm_mon)
                % st->tm_mday
                % st->tm_hour
                % st->tm_min
                % st->tm_sec
            ).str();
}

void on_timer1(const boost::system::error_code& error)
{
    if (error == asio::error::operation_aborted)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "cancelled" << std::endl;
    else if (error)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "other error" << std::endl;
    else
        std::cout << BOOST_CURRENT_FUNCTION << now() << "correct" << std::endl;
}

void on_timer2(const boost::system::error_code& error)
{
    if (error == asio::error::operation_aborted)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "cancelled" << std::endl;
    else if (error)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "other error" << std::endl;
    else
        std::cout << BOOST_CURRENT_FUNCTION << now() << "correct" << std::endl;
}

void on_timer3(const boost::system::error_code& error)
{
    if (error == asio::error::operation_aborted)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "cancelled!" << std::endl;
    else if (error)
        std::cout << BOOST_CURRENT_FUNCTION << now() << "other error" << std::endl;
    else
        std::cout << BOOST_CURRENT_FUNCTION << now() << "correct!!!!" << std::endl;
}

int main()
{
    asio::io_service io_service;
   
    asio::deadline_timer timer(io_service);
    std::cout << now() << "start" << std::endl;

    timer.expires_from_now(boost::posix_time::seconds(5));
    timer.async_wait(on_timer1);
    timer.async_wait(on_timer2);

//  nothing here they are all at correct
    timer.cancel_one();
//    int timersoff = timer.cancel();
//    std::cout << timersoff << " timers at cancek" << std::endl;
 
    timer.expires_from_now(boost::posix_time::seconds(10));
    timer.async_wait(on_timer1);
    timer.async_wait(on_timer2);
    timer.async_wait(on_timer3);
    
    io_service.run();
}

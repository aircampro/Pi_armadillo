#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

namespace asio = boost::asio;

void f()
{
    long long cc = 0;
    while (cc < 10000) {
    if (cc == 1000) {
        std::cout << "f" << std::endl;
    }
    ++cc;
    }
}

void g(const boost::system::error_code& error)
{
    if (error) {
        std::cout << "error g" << std::endl;
    }
    else {
        std::cout << "correct g" << std::endl;
    }
}

int main()
{
    asio::io_service io_service;
    asio::deadline_timer timer(io_service);

    io_service.post(f);

    timer.expires_from_now(boost::posix_time::seconds(10));
    timer.async_wait(g);

    io_service.poll();

    io_service.run_one();
}

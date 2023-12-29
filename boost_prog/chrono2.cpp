#include <iostream>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <boost/asio/high_resolution_timer.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/system_timer.hpp>


class printer {
    private:
        boost::asio::io_context io_;
        boost::asio::steady_timer timer_;
        int count_;
        void print() {
            if (count_ < 500) {
                std::cout << count_ << "\n";
                ++count_;

                timer_.expires_from_now(std::chrono::milliseconds (50));
                timer_.async_wait(std::bind(&printer::print, this));
            }
            else
            {
                std::cout << "Final count is " << count_ << "\n";
                delete this;
            }
        }
        void run() {
            timer_.expires_from_now(std::chrono::milliseconds (50));
            timer_.async_wait(std::bind(&printer::print, this));
            io_.run();
        }
        printer()
            : timer_(io_),
              count_(0) {

        }
        ~printer() {

        }

    public:

        static printer* Create(){
            return new printer;
        }

        void start() {
            std::thread t;
            t = std::thread(std::mem_fn(&printer::run), this);
            t.detach();
        }
};
void foo()
{
    printer *p = printer::Create();
    p->start();
}
int main() {
    foo();
    std::cin.get();
    return 0;
}

//boost::posix_time::to_simple_string函数需要这两个头文件
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
//使用boost.chrono代替std.chrono,
#define BOOST_ASIO_DISABLE_STD_CHRONO
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/thread.hpp>

class Timer
{
public:
    Timer() :work_(io_), timer_(io_){}
public:
    boost::thread_group thgp_;
    boost::asio::io_context io_;
    boost::asio::io_context::work work_;
    boost::asio::steady_timer timer_;
public:
    void init()
    {
        boost::system::error_code errCode;
        thgp_.create_thread(boost::bind(&boost::asio::io_service::run, boost::ref(io_), errCode));
        timer_.expires_from_now(boost::chrono::milliseconds(1000)); //设置过期时间长度
        timer_.async_wait(boost::bind(&Timer::excuteMission, this, boost::asio::placeholders::error));//异步等待
        std::cout << "initialize:" << localTime() << std::endl;
        //由Console可知, 函数立即返回了, 定时器的expires_from_now是由完成端口处理的
    }
    void stop()
    {
        timer_.cancel();  // 取消所有handler
        work_.~work();
        thgp_.join_all();
        std::cout << "Stop:" << localTime() << std::endl;
    }
    static std::string localTime()
    {
        return boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::local_time());
    }

    void excuteMission(const boost::system::error_code& ec)
    {
        std::cout<<"mission to print time:"<<localTime().c_str()<<" ErrorValue:"<<ec.value()<<" ErrorCode:"<<ec.message().c_str()<<std::endl;
        timer_.expires_from_now(boost::chrono::milliseconds(1000));
        timer_.async_wait(boost::bind(&Timer::excuteMission, boost::ref(*this), _1));
#if 0
        timer_.async_wait(boost::bind(&Timer::excuteMission, this, _1));
        timer_.async_wait(boost::bind(&Timer::excuteMission, this, boost::asio::placeholders::error));
#endif
    }
};

int main(int argc, char** argv)
{
    Timer t;
    t.init();
    while(true)
    {
        std::cout<<"execute other mission"<<std::endl;
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    }
    t.stop();
    std::cout << "press ENTER to exit..." << std::endl;
    //    std::cin.sync();
    return 0;
}

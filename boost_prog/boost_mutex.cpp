#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

boost::mutex mutex;

template <class T>
void write_console(const T& x)
{
    boost::mutex::scoped_lock lock(mutex);
    std::cout << x << std::endl;
}

class event_manager {
    boost::asio::io_service io_service_;
    boost::shared_ptr<boost::asio::io_service::work> work_;
    boost::thread_group group_;

    boost::asio::io_service::strand ok_button_strand_;
    boost::asio::io_service::strand cancel_button_strand_;

    boost::function<void()> ok_button_clicked_;
    boost::function<void()> cancel_button_clicked_;
public:
    event_manager()
        : ok_button_strand_(io_service_),
          cancel_button_strand_(io_service_)
    {
        work_.reset(new boost::asio::io_service::work(io_service_));

        for (std::size_t i = 0; i < 3; ++i) { // 3スレッドで動かす
            group_.create_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        }
    }

    ~event_manager()
    {
        work_.reset();
        group_.join_all();
    }

    template <class F>
    void set_ok_button_event(F f) { ok_button_clicked_ = f; }

    template <class F>
    void set_cancel_button_event(F f) { cancel_button_clicked_ = f; }

    void ok_button_click()
    {
        // 排他可能にしてpost
        io_service_.post(ok_button_strand_.wrap(ok_button_clicked_));
    }

    void cancel_button_click()
    {
        io_service_.post(cancel_button_strand_.wrap(cancel_button_clicked_));
    }
};


class Button {
    std::string name;
public:
    explicit Button(const std::string& name)
        : name(name) {}

    void clicked()
    {
        write_console("start : " + name + " clicked");
        boost::this_thread::sleep(boost::posix_time::seconds(2));
        write_console("end : " + name + " clicked");
    }
};

int main()
{
    event_manager manager;

    Button ok_button("ok");
    Button cancel_button("cancel");

    manager.set_ok_button_event(boost::bind(&Button::clicked, &ok_button));
    manager.set_cancel_button_event(boost::bind(&Button::clicked, &cancel_button));

    // ボタンのクリックイベント中に同じイベントを発生させようとする
    manager.ok_button_click();
    manager.ok_button_click();
    manager.cancel_button_click();
    manager.cancel_button_click();
    manager.ok_button_click();

    for (;;) {}
}

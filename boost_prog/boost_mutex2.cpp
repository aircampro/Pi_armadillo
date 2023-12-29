//
// Shows example of event handling for camera actions using boost library
//
//
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

boost::mutex mutex;
int g_val = 100;

// use the standard mutex to only allow these to occur sequentially
//
std::mutex m_mutex;

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

    // these are typical actions which would occur syncronsly
	//
    boost::asio::io_service::strand ok_button_strand_;
    boost::asio::io_service::strand cancel_button_strand_;

    // these actions are queued by the mutex sephaphore
    //	
    boost::asio::io_service::strand set_sony_aper_strand_;
    boost::asio::io_service::strand set_sony_wb_strand_;
    boost::asio::io_service::strand set_sony_fm_strand_;
    boost::asio::io_service::strand set_sony_fa_strand_;
    boost::asio::io_service::strand set_sony_iso_strand_;
    boost::asio::io_service::strand set_sony_ss_strand_;
    boost::asio::io_service::strand set_sony_sc_strand_;
    boost::asio::io_service::strand set_sony_expro_strand_;
	
    boost::function<void()> ok_button_clicked_;
    boost::function<void()> cancel_button_clicked_;
    boost::function<void()> do_sony_aper_;
    boost::function<void()> do_sony_wb_;
    boost::function<void()> do_sony_fm_;	
    boost::function<void()> do_sony_fa_;
    boost::function<void()> do_sony_iso_;
    boost::function<void()> do_sony_ss_;
    boost::function<void()> do_sony_sc_;	
    boost::function<void()> do_sony_expro_;	
	
public:
    event_manager()
        : ok_button_strand_(io_service_),
          cancel_button_strand_(io_service_),
          set_sony_aper_strand_(io_service_),
          set_sony_wb_strand_(io_service_),
          set_sony_fm_strand_(io_service_),
          set_sony_fa_strand_(io_service_),
          set_sony_iso_strand_(io_service_),
          set_sony_ss_strand_(io_service_),
          set_sony_sc_strand_(io_service_),
          set_sony_expro_strand_(io_service_)
    {
        work_.reset(new boost::asio::io_service::work(io_service_));

        for (std::size_t i = 0; i < 3; ++i) { // 3 Move by thread
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

    template <class F>
    void set_sony_aper_event(F f) { do_sony_aper_ = f; }

    template <class F>
    void set_sony_wb_event(F f) { do_sony_wb_ = f; }

    template <class F>
    void set_sony_fm_event(F f) { do_sony_fm_ = f; }

    template <class F>
    void set_sony_fa_event(F f) { do_sony_fa_ = f; }

    template <class F>
    void set_sony_iso_event(F f) { do_sony_iso_ = f; }

    template <class F>
    void set_sony_ss_event(F f) { do_sony_ss_ = f; }

    template <class F>
    void set_sony_sc_event(F f) { do_sony_sc_ = f; }

    template <class F>
    void set_sony_expro_event(F f) { do_sony_expro_ = f; }
	
    void ok_button_click()
    {
        // Make it exclusive and post
        io_service_.post(ok_button_strand_.wrap(ok_button_clicked_));
    }

    void cancel_button_click()
    {
        io_service_.post(cancel_button_strand_.wrap(cancel_button_clicked_));
    }
	
    void do_sony_aper()
    {
        io_service_.post(set_sony_aper_strand_.wrap(do_sony_aper_));
    }
	
    void do_sony_wb()
    {
        io_service_.post(set_sony_wb_strand_.wrap(do_sony_wb_));
    }

    void do_sony_fm()
    {
        io_service_.post(set_sony_fm_strand_.wrap(do_sony_fm_));
    }

    void do_sony_fa()
    {
        io_service_.post(set_sony_fa_strand_.wrap(do_sony_fa_));
    }
	
    void do_sony_iso()
    {
        io_service_.post(set_sony_iso_strand_.wrap(do_sony_iso_));
    }

    void do_sony_ss()
    {
        io_service_.post(set_sony_ss_strand_.wrap(do_sony_ss_));
    }

    void do_sony_sc()
    {
        io_service_.post(set_sony_sc_strand_.wrap(do_sony_sc_));
    }

    void do_sony_expro()
    {
        io_service_.post(set_sony_expro_strand_.wrap(do_sony_expro_));
    }
	
};

// The button class denotes the action request - typically HMI button but can be anything
//
class Button {
    std::string name;
public:
    explicit Button(const std::string& name)
        : name(name) {}

    // these operations can run concurrently
	//
    void clicked()
    {
        write_console("start : " + name + " clicked");
        boost::this_thread::sleep(boost::posix_time::seconds(2));
        write_console("end : " + name + " clicked");
    }

    // setting options are mutexed you can only set one of them at any one time
    //	
    void set_option()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_val << std::endl;
        boost::this_thread::sleep(boost::posix_time::seconds(2));
        write_console("ends the action : " + name + "  ");
    }
};

int main(void)
{
    event_manager manager;

    Button ok_button("ok");
    Button cancel_button("cancel");
    Button set_aperture("aperture");
    Button set_wb("white_balance");
    Button set_fm("focus_mode");
    Button set_fa("focus_area");
    Button set_iso("iso");
    Button set_ss("shutter_speed");
    Button set_sc("still_capture");
    Button set_expro("exposure_program");
	
    manager.set_ok_button_event(boost::bind(&Button::clicked, &ok_button));
    manager.set_cancel_button_event(boost::bind(&Button::clicked, &cancel_button));
    manager.set_sony_aper_event(boost::bind(&Button::set_option, &set_aperture));
    manager.set_sony_wb_event(boost::bind(&Button::set_option, &set_wb));
    manager.set_sony_fm_event(boost::bind(&Button::set_option, &set_fm));
    manager.set_sony_fa_event(boost::bind(&Button::set_option, &set_fa));
    manager.set_sony_iso_event(boost::bind(&Button::set_option, &set_iso));
    manager.set_sony_ss_event(boost::bind(&Button::set_option, &set_ss));
    manager.set_sony_sc_event(boost::bind(&Button::set_option, &set_sc));
    manager.set_sony_expro_event(boost::bind(&Button::set_option, &set_expro));
	
    // Try to fire the same event during a button click event
    manager.ok_button_click();
    manager.ok_button_click();
	g_val = 20;
	manager.do_sony_aper();
	manager.do_sony_wb();
	manager.do_sony_fm();
	manager.do_sony_iso();
	manager.do_sony_expro();
    manager.cancel_button_click();
	manager.do_sony_fa();
    manager.cancel_button_click();
    manager.ok_button_click();
	g_val = 11;
	manager.do_sony_aper();
	manager.do_sony_sc();
	manager.do_sony_ss();
    manager.ok_button_click();
	
    for (;;) {}
}

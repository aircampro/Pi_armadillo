#ifndef __boost_strand_lib_
#define __boost_strand_lib_

//
// Shows example of event handling for camera actions and other non-depandant tasks using boost library
//
// compile :: 
//  g++-8 -std=c++17 -o  boost_mutex2 boost_mutex2.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread 
//  -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lboost_system -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I/usr/lib/arm-linux-gnueabihf -lboost_filesystem
//
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

// global mutex prevents multiple runs of the camera methods clashing and requiring usb reset
//
boost::mutex mutex;

// use this global structure to share values with this library if you wish
//
typedef struct {
 int set_ap;                                      /*<  aperture */
 int set_wb;                                      /*<  white balance */
 int set_fm;                                      /*<  focus mode */
 int set_fa;                                      /*<  focus area */
 int set_iso;                                     /*<  iso */
 int set_sc;                                      /*<  still capture */
 int set_ss;                                      /*<  shutter speed */
 int set_ex;                                      /*<  exposure program mode */
} sony_cam_option_vals_t;

sony_cam_option_vals_t g_settings;

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

    boost::asio::io_service::strand set_sony_aper1_strand_;
    boost::asio::io_service::strand set_sony_wb1_strand_;
    boost::asio::io_service::strand set_sony_fm1_strand_;
    boost::asio::io_service::strand set_sony_fa1_strand_;
    boost::asio::io_service::strand set_sony_iso1_strand_;
    boost::asio::io_service::strand set_sony_ss1_strand_;
    boost::asio::io_service::strand set_sony_sc1_strand_;
    boost::asio::io_service::strand set_sony_expro1_strand_;
	
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
	
    boost::function<void()> do_sony_aper1_;
    boost::function<void()> do_sony_wb1_;
    boost::function<void()> do_sony_fm1_;	
    boost::function<void()> do_sony_fa1_;
    boost::function<void()> do_sony_iso1_;
    boost::function<void()> do_sony_ss1_;
    boost::function<void()> do_sony_sc1_;	
    boost::function<void()> do_sony_expro1_;		
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
          set_sony_expro_strand_(io_service_),
          set_sony_aper1_strand_(io_service_),
          set_sony_wb1_strand_(io_service_),
          set_sony_fm1_strand_(io_service_),
          set_sony_fa1_strand_(io_service_),
          set_sony_iso1_strand_(io_service_),
          set_sony_ss1_strand_(io_service_),
          set_sony_sc1_strand_(io_service_),
          set_sony_expro1_strand_(io_service_)
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

    template <class F>
    void set_sony_aper_event1(F f) { do_sony_aper1_ = f; }

    template <class F>
    void set_sony_wb_event1(F f) { do_sony_wb1_ = f; }

    template <class F>
    void set_sony_fm_event1(F f) { do_sony_fm1_ = f; }

    template <class F>
    void set_sony_fa_event1(F f) { do_sony_fa1_ = f; }

    template <class F>
    void set_sony_iso_event1(F f) { do_sony_iso1_ = f; }

    template <class F>
    void set_sony_ss_event1(F f) { do_sony_ss1_ = f; }

    template <class F>
    void set_sony_sc_event1(F f) { do_sony_sc1_ = f; }

    template <class F>
    void set_sony_expro_event1(F f) { do_sony_expro1_ = f; }
	
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

    void do_sony_aper1()
    {
        io_service_.post(set_sony_aper1_strand_.wrap(do_sony_aper1_));
    }
	
    void do_sony_wb1()
    {
        io_service_.post(set_sony_wb1_strand_.wrap(do_sony_wb1_));
    }

    void do_sony_fm1()
    {
        io_service_.post(set_sony_fm1_strand_.wrap(do_sony_fm1_));
    }

    void do_sony_fa1()
    {
        io_service_.post(set_sony_fa1_strand_.wrap(do_sony_fa1_));
    }
	
    void do_sony_iso1()
    {
        io_service_.post(set_sony_iso1_strand_.wrap(do_sony_iso1_));
    }

    void do_sony_ss1()
    {
        io_service_.post(set_sony_ss1_strand_.wrap(do_sony_ss1_));
    }

    void do_sony_sc1()
    {
        io_service_.post(set_sony_sc1_strand_.wrap(do_sony_sc1_));
    }

    void do_sony_expro1()
    {
        io_service_.post(set_sony_expro1_strand_.wrap(do_sony_expro1_));
    }	
};

// The button class denotes the action request - typically HMI button but can be anything
//
class Button {
    std::string name;
	int value;
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

    void set_value(int v)
	{
        std::lock_guard<std::mutex> lock(m_mutex);	
        this->value = v;		
	}

    void push(int dat)
    {
        boost::lock_guard<boost::mutex> lock(bv_mutex);              // Защитим данные
        this->value = dat;                                                // запишем данные
        cond.notify_all();                                           // уведомим, что есть новые данные
    }

    int pop()
    {
        boost::unique_lock<boost::mutex> lock(bv_mutex);             // Защитим данные
        cond.wait(lock);                                             // ждём новые данные, при этом мутекс разблокируется, по выходу
                                                                     // он будет снова заблакирован
        return this->value;
    }
	
    // setting options are mutexed you can only set one of them at any one time
    //	
    void set_option()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("OPTION begins the action : " + name + " with ");
		std::cout << "\033[33m" << this->value << "\033[0m" << std::endl;
        boost::this_thread::sleep(boost::posix_time::seconds(10));
        write_console("OPTION ends the action : " + name + "  ");
    }
	
	void set_cam_aper()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_ap << std::endl;
		if (camera->SetApertureArgsInt(g_settings.set_ap)==1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("AP ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_wb()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_wb << std::endl;
		if (camera->SetWhiteBalanceArgsInt(g_settings.set_wb)==1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("WB ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}		
    }
	void set_cam_fm()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_fm << std::endl;
		if (camera->SetFocusModeArgsInt(g_settings.set_fm) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_fa()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_fa << std::endl;
		if (camera->SetFocusAreaArgsInt(g_settings.set_fa) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }	
	void set_cam_iso()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_iso << std::endl;
		if (camera->SetIsoArgsInt(g_settings.set_iso) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_ss()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_ss << std::endl;
		if (camera->SetShutterSpeedArgsInt(g_settings.set_ss) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_sc()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_sc << std::endl;
		if (camera->SetStillCaptureModeArgsInt(g_settings.set_sc) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }	
	void set_cam_expro()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << g_settings.set_ex << std::endl;
		if (camera->SetExposureProgramModeArgsInt(g_settings.set_ex) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	
	void set_cam_aper1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetApertureArgsInt(this->value)==1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("AP ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_wb1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetWhiteBalanceArgsInt(this->value)==1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("WB ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}		
    }
	void set_cam_fm1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetFocusModeArgsInt(this->value) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_fa1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetFocusAreaArgsInt(this->value) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }	
	void set_cam_iso1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetIsoArgsInt(this->value) == 1 ) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_ss1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetShutterSpeedArgsInt(this->value) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
	void set_cam_sc1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetStillCaptureModeArgsInt(this->value) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }	
	void set_cam_expro1()
    {
        std::lock_guard<std::mutex> lock(m_mutex);	
        write_console("begins the action : " + name + " with ");
		std::cout << this->value << std::endl;
		if (camera->SetExposureProgramModeArgsInt(this->value) == 1) {
            boost::this_thread::sleep(boost::posix_time::seconds(2));
            write_console("ends the action : " + name + "  ");
		} else {
            write_console("set failed : " + name + "  ");		    
		}
    }
private:
    boost::mutex                bv_mutex;
    boost::conditional_variable cond;
};

#define __boost_strand_test_
#if defined(__boost_strand_test_)
int main(void)
{
    event_manager manager;

    Button ok_button("ok");
    Button cancel_button("cancel");
	
	// make each operation and call them with a separate method
    Button set_aperture("aperture");
    Button set_wb("white_balance");
    Button set_fm("focus_mode");
    Button set_fa("focus_area");
    Button set_iso("iso");
    Button set_ss("shutter_speed");
    Button set_sc("still_capture");
    Button set_expro("exposure_program");
    // you can use this global structure to communicate the value as if it overwrites its okay
	// this is used in each separate events methods
	//
    g_settings.set_ap = 1;
    g_settings.set_fa = 1;
    g_settings.set_fm = 2;
    g_settings.set_ss = 2;
    g_settings.set_sc = 2;
    g_settings.set_ex = 2;
    g_settings.set_iso = 2;
    g_settings.set_wb = 3;
	
	// now alternativly make a second set of objects calling one method using the class variable
	//
    Button set_aperture1("aperture");
    Button set_wb1("white_balance");
    Button set_fm1("focus_mode");
    Button set_fa1("focus_area");
    Button set_iso1("iso");
    Button set_ss1("shutter_speed");
    Button set_sc1("still_capture");
    Button set_expro1("exposure_program");
	
	// set the class variables instead of using the global structure
	//
    set_aperture1.set_value(1);
    set_wb1.set_value(1);
    set_fm1.set_value(1);
    set_fa1.set_value(2);
    set_iso1.set_value(2);
    set_ss1.set_value(2);
    set_sc1.set_value(2);
    set_expro1.set_value(3);	
	
    manager.set_ok_button_event(boost::bind(&Button::clicked, &ok_button));
    manager.set_cancel_button_event(boost::bind(&Button::clicked, &cancel_button));

    manager.set_sony_aper_event(boost::bind(&Button::set_cam_aper, &set_aperture));
    manager.set_sony_wb_event(boost::bind(&Button::set_cam_wb, &set_wb));
    manager.set_sony_fm_event(boost::bind(&Button::set_cam_fm, &set_fm));
    manager.set_sony_fa_event(boost::bind(&Button::set_cam_fa, &set_fa));
    manager.set_sony_iso_event(boost::bind(&Button::set_cam_iso, &set_iso));
    manager.set_sony_ss_event(boost::bind(&Button::set_cam_ss, &set_ss));
    manager.set_sony_sc_event(boost::bind(&Button::set_cam_sc, &set_sc));
    manager.set_sony_expro_event(boost::bind(&Button::set_cam_expro, &set_expro));

    manager.set_sony_aper_event(boost::bind(&Button::set_cam_aper1, &set_aperture1));
    manager.set_sony_wb_event(boost::bind(&Button::set_cam_wb1, &set_wb1));
    manager.set_sony_fm_event(boost::bind(&Button::set_cam_fm1, &set_fm1));
    manager.set_sony_fa_event(boost::bind(&Button::set_cam_fa1, &set_fa1));
    manager.set_sony_iso_event(boost::bind(&Button::set_cam_iso1, &set_iso1));
    manager.set_sony_ss_event(boost::bind(&Button::set_cam_ss1, &set_ss1));
    manager.set_sony_sc_event(boost::bind(&Button::set_cam_sc1, &set_sc1));
    manager.set_sony_expro_event(boost::bind(&Button::set_cam_expro1, &set_expro1));

    // this was calling a single function to handle each indivual event if you like
    //	
    //manager.set_sony_aper_event1(boost::bind(&Button::set_option, &set_aperture1));
    //manager.set_sony_wb_event1(boost::bind(&Button::set_option, &set_wb1));
    //manager.set_sony_fm_event1(boost::bind(&Button::set_option, &set_fm1));
    //manager.set_sony_fa_event1(boost::bind(&Button::set_option, &set_fa1));
    //manager.set_sony_iso_event1(boost::bind(&Button::set_option, &set_iso1));
    //manager.set_sony_ss_event1(boost::bind(&Button::set_option, &set_ss1));
    //manager.set_sony_sc_event1(boost::bind(&Button::set_option, &set_sc1));
    //manager.set_sony_expro_event1(boost::bind(&Button::set_option, &set_expro1));
	
    // Try to fire the same event during a button click event
    manager.ok_button_click();
    manager.ok_button_click();

	// this first method is using the global structure
	manager.do_sony_aper();
	manager.do_sony_wb();
	manager.do_sony_fa();
	manager.do_sony_fm();
	manager.do_sony_iso();
	manager.do_sony_expro();
    manager.cancel_button_click();
	manager.do_sony_fa();
    manager.cancel_button_click();
    manager.ok_button_click();
	manager.do_sony_aper();
	manager.do_sony_sc();
	manager.do_sony_ss();
    manager.ok_button_click();
	// the second methods use the class variable
	manager.do_sony_aper1();
	manager.do_sony_wb1();
	manager.do_sony_fa1();
	manager.do_sony_fm1();
	manager.do_sony_iso1();
	manager.do_sony_expro1();
	manager.do_sony_sc1();
	manager.do_sony_ss1();
    set_aperture1.set_value(3);
	manager.do_sony_aper1();
    set_wb1.push(5);
	int wb_pop = set_wb1.pop();
    std::cout << " push pop " << wb_pop << std::endl;   	
    for (;;) {}
}
#endif

#endif // end_library

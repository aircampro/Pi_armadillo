#include <ctime>
#include <cstdlib>
#if defined(USE_EXPERIMENTAL_FS)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace fs = std::filesystem;
#if defined(__APPLE__)
#include <unistd.h>
#endif
#endif
#include <cstdint>
#include <iomanip>
#include "CRSDK/CameraRemote_SDK.h"
#include "CameraDevice.h"
#include "Text.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>

// used as a global to trap a segfault when trying to take a photo too quickly
//
static int g_hoge = 0;

#include <time.h>
#include <sys/time.h>

//#define __focus_adjustment                  /* set this for focus adjustment to be active */
#include "focus_adjustment.h"

//#define LIVEVIEW_ENB

namespace SDK = SCRSDK;

// MAVLink
#include "../../mavlink2C/common/mavlink.h"
#include "../../mavlink2C/mavlink_types.h"
#include "../../mavlink2C/common/mavlink_msg_statustext.h"

// Global dll object
// cli::CRLibInterface* cr_lib = nullptr;

//auto ncams;
//auto camera_info;
//bool connect_status;
SDK::ICrEnumCameraObjectInfo* camera_list = nullptr;
typedef std::shared_ptr<cli::CameraDevice> CameraDevicePtr;
CameraDevicePtr camera;
typedef std::vector<CameraDevicePtr> CameraDeviceList;
CameraDeviceList cameraList;                                                     // all
std::int32_t cameraNumUniq = 1;
std::int32_t selectCamera = 1;

//#include <wiringPi.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include <boost/version.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/algorithm/clamp.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

// for arg parsing using boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#if 0

void init()
{
    logging::add_file_log("/home/anthony/sample.log");

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
}

void init()
{
    logging::add_file_log
    (
        keywords::file_name = "/home/anthony/sample_%N.log",
        keywords::rotation_size = 10 * 1024 * 1024,
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
        keywords::format = "[%TimeStamp%]: %Message%"
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
	
	// added from https://github.com/termoshtt/boost_log_sample/blob/master/src/set_filter.cpp
	logging::add_common_attributes();
}

#else

void init()
{
    logging::add_file_log
    (
        keywords::file_name = "/home/anthony/sample_%N.log",                          /*< file name pattern >*/
        keywords::rotation_size = 10 * 1024 * 1024,                                   /*< rotate files every 10 MiB... >*/
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), /*< ...or at midnight >*/
        keywords::format = "[%TimeStamp%]: %Message%"                                 /*< log record format >*/
    );

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );
	
	// added from https://github.com/termoshtt/boost_log_sample/blob/master/src/set_filter.cpp
	logging::add_common_attributes();
}

#endif

// ==================== multi threading ====================================================================
#include <boost/thread/thread.hpp>//Include boost header file
#include <iostream>
#include <cstdlib>
using namespace std;
volatile bool isRuning = true;

// ==================== include object library ==============================================================
#include "object_store_class.hpp"
// ==================== include enumeration library =========================================================
#include "sony_a7_enumerate.cpp"
// ==================== random sequence generator for testing ===============================================
#include "random_seq_gen.cpp"
// ==================== run python script which resets usb via the portable code library ====================
#include "poco_script_runner.cpp"

// In this single tasking mode we will use this state Engine to determine the camera actions
//
enum CrCameraAction : std::uint32_t
{
    Sony_Camera_Idle = 0,
    Sony_Camera_Get = 1,
    Sony_Camera_Action_Init = 2,
    Sony_Camera_Set_Begin = 3,
    Sony_Camera_Set = 4,
    Sony_Camera_Set_Wait = 5,
    Sony_Camera_Check = 6,
    Sony_Camera_Set_Wait2 = 7,
    Sony_Camera_Photo = 100,
};
std::uint32_t actionState = Sony_Camera_Idle;

// this is driving an output on the input circuit interrupt
// in our main code with shall add camera->capture_image();
// to take a photo on the input being set to low
//
// set-up i/o
//
const int pin = 21;
const int ip = 7;   // GPIO No. 4 is the input from the camera

// create a global counter for number of pictures taken
//
std::uint64_t g_num_of_photo = 0;

void gpio4_interrupt(void)
{
    if (actionState == Sony_Camera_Idle) {
        actionState = Sony_Camera_Photo;
        //digitalWrite(pin, 1);    // High 3.3V
        camera->capture_image(); // capture image
        cli::tout << "taken photo.... " << "\n";
        //delay(1000);              block and wait
        //digitalWrite(pin, 0);    // High 0.0V
        actionState = Sony_Camera_Idle;
        // create a global counter for number of pictures
        ++g_num_of_photo;
    }
}

// routine which can clean up when program closed unexpectably
//
void clean_and_destroy() {
    // now disconnect the device
    //
    CameraDeviceList::const_iterator it = cameraList.begin();
    for (std::int32_t j = 0; it != cameraList.end(); ++j, ++it) {
        if ((*it)->is_connected()) {
            //cli::tout << "Initiate disconnect sequence.\n";
            auto disconnect_status = (*it)->disconnect();
            if (!disconnect_status) {
                // try again
                disconnect_status = (*it)->disconnect();
            }
            if (!disconnect_status)
                cli::tout << "Disconnect failed to initiate.\n";
            //else
            //  cli::tout << "Disconnect successfully initiated!\n\n";
        }
        (*it)->release();
    }

    //cli::tout << "Release SDK resources.\n";
    // cr_lib->Release();
    SDK::Release();
}

bool g_pause = false;

void sig_handler(int signo) {

    switch (signo)
    {
        case SIGUSR1:
        cli::tout << " --------- GOt the SIGUSR1 e.g. kill -10 Do your exception here ------------------- " << "\n";
		camera->capture_image();                    // capture image
        break;

        case SIGUSR2:
        cli::tout << " --------- GOt the SIGUSR2 e.g. kill -12 Do your exception here ------------------- " << "\n";
		camera->capture_image();                    // capture image
        break;

        case SIGXCPU:
        cli::tout << " --------- GOt the SIGXCPU - CPU Time Limit Exceeded (4.2BSD)  ------------------- " << "\n";
        break;
		
        case SIGHUP:                                // disconnecting (hanging) the controlling terminal, terminating the virtual terminal
        clean_and_destroy();
        cli::tout << " --------- Got the SIGHUP clean-up & exit ------------------- " << "\n";
        exit(-SIGHUP);
        break;

        case SIGBUS:                                // disconnecting (hanging) the controlling terminal, terminating the virtual terminal
        clean_and_destroy();
        cli::tout << " --------- Got the SIGBUS clean-up & exit ------------------- " << "\n";
        exit(-SIGBUS);
        break;
		
        case SIGINT:                               // INTERRUPT SIGNAL FROM KEYBOARD (USUALLY [CTRL]+[C])
        clean_and_destroy();
        cli::tout << " --------- Got the SIGINT clean-up & exit ------------------- " << "\n";
        exit(-SIGINT);
        break;

        case SIGQUIT:                              // KEYBOARD ABORT SIGNAL (USUALLY [CTRL]+[\])
        clean_and_destroy();
        cli::tout << " --------- Got the SIGQUIT clean-up & exit ------------------- " << "\n";
        exit(-SIGQUIT);
        break;

        case SIGFPE:	                          // occurrence of illegal floating-point operations (such as division by zero or overflow)	
        clean_and_destroy();
        cli::tout << " --------- Got the SIGFPE clean-up & exit ------------------- " << "\n";
        exit(-SIGFPE);
        break;

        case SIGKILL:	                         // FORCE EXIT SIGNAL (KILL SIGNAL)	
        clean_and_destroy();
        cli::tout << " --------- Got the SIGKILL clean-up & exit ------------------- " << "\n";
        exit(-SIGKILL);
        break;

        case SIGSEGV:	                         // incorrect memory reference occurrence	
		if (g_hoge != 1) {
            clean_and_destroy();
            cli::tout << " --------- Got the SIGSEGV clean-up & exit ------------------- " << "\n";
            exit(-SIGSEGV);
		} else {
			cli::tout << " -------- got a SIGSEGV cos i tried a picture too quickly --------- " << "\n";
			reset_usb_sony();                      // force a power reset of the usb port
			sleep(2);                              // let settle
			camera->SetShutterSpeedArgsInt(35);    // hard code to 1/250
		}
        break;

        case SIGPIPE:	                         // 	writing to a pipe without a reader (usually terminating immediately upon receiving this signal)	
        clean_and_destroy();
        cli::tout << " --------- Got the SIGPIPE clean-up & exit ------------------- " << "\n";
        exit(-SIGPIPE);
        break;

        case SIGTERM:	                        // 	exit signal (default signal for the "kill" command)
        clean_and_destroy();
        cli::tout << " --------- Got the SIGTERM clean-up & exit ------------------- " << "\n";
        exit(-SIGTERM);
        break;

        case SIGCONT:	                        // 	resume signal to a paused job
        g_pause = false;
        break;

        case SIGSTOP:	                        // 	start a paused job
        g_pause = true;
        break;

        case SIGTSTP:	                        // 	PAUSE SIGNAL FROM THE TERMINAL (USUALLY [CTRL] + [Z])
        clean_and_destroy();
        cli::tout << " --------- Got the SIGTSTP clean-up & exit ------------------- " << "\n";
        exit(-SIGTSTP);
        break;
    }
}

// http://www.coins.tsukuba.ac.jp/~syspro/2005/No5.html
//
// this is a timed interrupt routine if it is required
//
void alrm(int signo)
{
	g_hoge = 1;
    // take a photo
    int st = clock();
    camera->capture_image();
    int en = clock();
    cli::tout << "time from alarm handler : " << (en - st) / double(CLOCKS_PER_SEC) * 1000.0f << "\n";
    cli::tout << " timer action took a picture" << "\n";
	g_hoge = 0;
}

// reset the usb link via uhubctl 
//
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/wait.h>
extern char** environ;

int reset_usb_link() {
    int pid;
    int code;
    int status;
    pid_t result;
    int i = 0;

    // performs a fork to usb control to turn off then on the usb
    //
    while (i < 2) {
        pid = fork();

        // fork
        if (pid == -1) {
            fprintf(stderr, "Error\n\n");
        }

        // this is the program that resets the usb power
        //
        char* argv[6];
        std::string s1 = "/usr/sbin/uhubctl/uhubctl";
        std::string s2 = "-l";
        std::string s3 = "1-1";
        std::string s4 = "-a";
        argv[0] = &*s1.begin();
        argv[1] = &*s2.begin();
        argv[2] = &*s3.begin();
        argv[3] = &*s4.begin();


        // first time disable then enable the usb power
        if (i == 0) {
            std::string s5 = "0";
            argv[4] = &*s5.begin();
        }
        else {
            std::string s5 = "1";
            argv[4] = &*s5.begin();
        }
        argv[5] = NULL;

        // Processing Child Processes
        if (pid == 0) {
            //execl("/home/pi/cams/SonyTEST32/uhubctl/uhubctl", "/home/pi/cams/SonyTEST32/uhubctl/uhubctl", NULL);
            execve(argv[0], argv, environ);
        }
        else {
            result = wait(&status);

            if (result < 0) {
                fprintf(stderr, "Wait Error.\n\n");
                exit(-1);
            }

            //Check the Exit status
            if (WIFEXITED(status)) {
                printf("Child process Termination");
                code = WEXITSTATUS(status);
                printf("the code %d\n", code);
            }
            else {
                printf("wait failuer");
                printf("exit code : %d\n", status);
            }

            i++;

        }
        sleep(10);
    }
    printf("Parent process Termination\n");
    return 0;
}

// to read the initialisation file
//
#include "ini_file_reader.cpp"
//#include "DronePayloadMan.cpp"
// to read the xml configuration defaults
//
#define SONY_XML_FILE "/home/anthony/sony_new_xml.xml"
#include "xml_reader.cpp"
// to read the last known set-up if required
//
#include "json_file_reader.cpp"
enum CrCameraDefaultSource : std::uint16_t
{
    No_Settings = 0,
    Settings_From_Ini = 1,
    Settings_From_Xml = 2,
    Settings_From_Last = 3,
    Get_starting_Settings = 4,
};

// Create a global object for each item that is controlled via mavlink
// objects are given the default name and updated from xml if requested
//
storage_object sony_iso("S_ISO");
storage_object sony_aper("S_APERTURE");
storage_object sony_ss("S_SHUT_SPD");
storage_object sony_expro("S_EX_PRO_MODE");
storage_object sony_foc_mode("S_FOCUS_MODE");
storage_object sony_foc_area("S_FOCUS_AREA");
storage_object sony_white_bal("S_WHITE_BAL");
storage_object sony_sc("S_STILL_CAP");
bool g_new_value = false;
sony_focus_settings_t focus_obj;
std::vector<std::tuple<std::uint32_t, std::uint32_t, std::string>> g_vt;         // global vector for step action movement.

#include "boost_free_space.cpp"
#include "boost_cam_actions.cpp"

// for focus adjustment add this library
//#include "focus_adjust.h"
#include "boost_serial_class.cpp"
#include "shutter_speed_adjust.cpp"

// this is used to determine no action for a timer pewriod therefore write set-up after a change to disk
#define NO_ACTIVITY_DURATION 500000.0f
auto g_write_start = std::chrono::high_resolution_clock::now();
auto g_write_end = std::chrono::high_resolution_clock::now();
auto g_write_tdiff = std::chrono::duration<double, std::milli>(g_write_end - g_write_start).count();
int g_gimbal_connected = 0;

#include "xml_adjust.cpp"                                  // permits adjustment for index values controlled over radio advance signal

//int main(void)
//int main(const int ac, const char* av)
//int main(int argc, char* argv[])
int cam_main()
{
    using boost::algorithm::clamp;

    /*
    // ===== Define command line options =====
	//
    namespace po = boost::program_options;
    po::options_description description("option");
    description.add_options()
    // ("Option name", "Argument (optional)", "Option description")
    ("action_delay_micros,adm", po::value<int>()->default_value(2500000), "")
    //("photo_delay_micros,pdm", po::value<int>()->default_value(500000), "")
    //("photo_delay_seconds,pds", po::value<int>()->default_value(0), "")
    ("photo_delay_micros,pdm", po::value<int>()->default_value(0), "")
    ("photo_delay_seconds,pds", po::value<int>()->default_value(10), "")
    ("help,h", "show this help message");

    po::variables_map vm;
    try {
        //po::store(parse_command_line(ac, av, description), vm);
		po::store(boost::program_options::parse_command_line(argc, argv, description), vm);
    } catch (po::error &e) {
       // Nonexistent option ・ throw an exception if the wrong type is specified
       cli::tout << e.what() << "\n";
       return -1;
    }

    po::notify(vm);
	
    //help, output help (description content) when h is entered
    if (vm.count("help")) {
       cli::tout << description << std::endl;
       return 0;
    }

    // Option number output
	try {
        int ad_microseconds = vm["action_delay_micros"].as<int>();
        cli::tout << ad_microseconds << std::endl;
    } catch (boost::bad_any_cast &e) {
        // default Throw an exception if an option with no value is not specified
        cli::tout << e.what() << std::endl;
        return -1;
    }

	try {
        int pd_microseconds = vm["photo_delay_micros"].as<int>();
        cli::tout << pd_microseconds << std::endl;
    } catch (boost::bad_any_cast &e) {
        // default Throw an exception if an option with no value is not specified
        cli::tout << e.what() << std::endl;
        return -1;
    }

	try {
        int pd_seconds = vm["photo_delay_seconds"].as<int>();
        cli::tout << pd_seconds << std::endl;
    } catch (boost::bad_any_cast &e) {
        // default Throw an exception if an option with no value is not specified
        cli::tout << e.what() << std::endl;
        return -1;
    }	
	//
    // ===== end options parser =====
	*/
	
    //cli::tout << BOOST_LIB_VERSION << '\n';
    std::tuple< std::uintmax_t, std::uintmax_t, std::uintmax_t > pi_disk_data = get_pi_disk_usage();
    std::uintmax_t global_pic_counter = free_space_main();
	
	reset_usb_sony();                      // force a power reset of the usb port
	sleep(2);                              // let settle

    // read the ini file to see if we defined if we had the focus adjustment gimbal connected
	try {
            g_gimbal_connected = clamp(get_ini_int_component("sony", "GIMBAL_IS_CONNECTED"), 0, 1);
	} catch (char *e) { cout << "Ini file reader couldnt find GIMBAL_IS_CONNECTED Exception : " << e << '\n'; }
	
    cli::tout << " gimbal_conn " << g_gimbal_connected << "\n";
	
    // ====== this code is for the gimbal connection on serial 
    //
//#if defined(__focus_adjustment)
    sony_focus_settings_t sony_focus_settings;
    boost::asio::io_service ios;
    serialport gimbal_cmd(ios);
    std::uint8_t send_bytes[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    // this bit is done in the mavlink code which is in the DronePayload project
    std::uint32_t raw_ss_val;
    std::uint32_t raw_aper_val;
    std::uint32_t raw_iso_val;	

//#endif

    int camId_photo = MAV_COMP_ID_CAMERA;                      // default to camera No.1
    int camId_actions = MAV_COMP_ID_CAMERA;                    // default to camera No.1
    int one_shot = 1;
    int xml_def_on = Settings_From_Ini;
    std::uint32_t ini_ap = 0;
    std::uint32_t ini_iso = 0;
    std::uint32_t ini_ss = 0;
    std::uint32_t ini_sc = 0;
    std::uint32_t ini_fm = 0;
    std::uint32_t ini_fa = 0;
    std::uint32_t ini_exp = 0;
    std::uint32_t ini_wb = 0;
    std::uint32_t ini_im_store = 0;

    // confirm the ini file exists if so read it
    //
    boost_fs::path const ini_file("/home/anthony/dpm_init.ini");
    if (boost_fs::is_regular_file(ini_file)) {
        cli::tout << "the ini file exists" << "\n";
        xml_def_on = Get_starting_Settings;
    } else {
        cli::tout << "the ini file is not existing" << "\n";
        xml_def_on = No_Settings;
    }
    // read the .ini file to get the camera settings or read the xml defaults or the current last value depending on xml_defaults value 0 1 or 2
    //
    if (g_gimbal_connected==1) {
//#if defined(__focus_adjustment)
        if (xml_def_on == Get_starting_Settings) {
            sony_focus_settings.mo_rc12fmu_mode_swap = clamp(get_ini_int_component("sony", "MO_RC12FMU_MODE_SWAP"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.gi_rc11fmu_gim_spt1 = clamp(get_ini_int_component("sony", "GI_RC11FMU_GIM_SPT1"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.gi_rc11fmu_gim_spt2 = clamp(get_ini_int_component("sony", "GI_RC11FMU_GIM_SPT2"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.gi_rc11fmu_gim_lim1 = clamp(get_ini_int_component("sony", "GI_RC11FMU_GIM_LIM1"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt1 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT1"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt2 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT2"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt3 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT3"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt4 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT4"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt5 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT5"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt6 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT6"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt7 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT7"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt8 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT8"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt9 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT9"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt10 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT10"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt11 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT11"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.fo_rc10fmu_spt12 = clamp(get_ini_int_component("sony", "FO_RC10FMU_SPT12"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.mo_rc12fmu_mode_swap = clamp(get_ini_int_component("sony", "SH_RC10FMU_SPT1"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.mo_rc12fmu_mode_swap = clamp(get_ini_int_component("sony", "SH_RC10FMU_SPT2"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);
            sony_focus_settings.mo_rc12fmu_mode_swap = clamp(get_ini_int_component("sony", "SH_RC10FMU_SPT3"), MOD_LO_VAL_CLAMP, MOD_HI_VAL_CLAMP);

            sony_focus_settings.mo_rc12fmu_tm1 = clamp(get_ini_int_component("sony", "MO_RC12FMU_TM1"), MOD_MIN_TIMER, MOD_MAX_TIMER);
            sony_focus_settings.mo_rc12fmu_tm1 = clamp(get_ini_int_component("sony", "FO_RC10FMU_TM1"), MOD_MIN_TIMER, MOD_MAX_TIMER);
            sony_focus_settings.mo_rc12fmu_tm1 = clamp(get_ini_int_component("sony", "SH_RC10FMU_TM1"), MOD_MIN_TIMER, MOD_MAX_TIMER);

            sony_focus_settings.hw_def_mode = clamp(get_ini_int_component("sony", "HW_DEF_MODE"), modGimbal, Number_of_cam_modes);
            sony_focus_settings.use_focus_setting = clamp(get_ini_int_component("sony", "DEF_FOCUS_NF"), SONY_SDK_FOCUS_FAR_FAST, SONY_SDK_FOCUS_Near_Fast);
            sony_focus_settings.prev_focus_setting = sony_focus_settings.use_focus_setting;
        }
    }
//#endif

    if (xml_def_on == Get_starting_Settings) {
        camId_photo = clamp(get_ini_int_component("sony_photo", "camera_id"), MAV_COMP_ID_CAMERA,MAV_COMP_ID_CAMERA6 );
        std::cout << " read the photo ID from the .ini " << camId_photo << std::endl;
        camId_actions = clamp(get_ini_int_component("sony", "camera_id"), MAV_COMP_ID_CAMERA, MAV_COMP_ID_CAMERA6);
        std::cout << " read the actions ID from the .ini " << camId_actions << std::endl;
        one_shot = clamp(get_ini_int_component("sony_photo", "one_shot"), 0, 1);
        xml_def_on = clamp(get_ini_int_component("sony", "xml_defaults"), No_Settings, Settings_From_Last);
    }

    // read the .ini file to get the camera settings or read the xml defaults or the current last value depending on xml_defaults value 0 1 or 2
    //
    if (xml_def_on == Settings_From_Ini) {
        ini_ap = clamp(get_ini_int_component("sony", "aperture"), 250, 2200);
        ini_iso = clamp(get_ini_int_component("sony", "iso"), 0, 102400);
        ini_ss = clamp(get_ini_int_component("sony", "shutter_s"), 65539, 19660810);
        ini_fm = clamp(get_ini_int_component("sony", "focus_mode"), SDK::CrFocus_MF, SDK::CrFocus_PF);
        ini_fa = clamp(get_ini_int_component("sony", "focus_area"), SDK::CrFocusArea_Unknown, SDK::CrFocusArea_Tracking_Flexible_Spot);
        ini_sc = clamp(get_ini_int_component("sony", "still_cap"), SDK::CrDrive_Single, SDK::CrDrive_SelfPortrait_2);
        ini_exp = clamp(get_ini_int_component("sony", "expos"), SDK::CrExposure_M_Manual, SDK::CrExposure_Movie_F_Mode);
        ini_wb = clamp(get_ini_int_component("sony", "white_bal"), SDK::CrWhiteBalance_AWB, SDK::CrWhiteBalance_Custom);
        ini_im_store = clamp(get_ini_int_component("sony", "image_store"), 0 , 1);
		int r1;
        if (g_gimbal_connected==1) {
#if defined(__focus_adjustment)
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss, sony_focus_settings.use_focus_setting);   // write the new set-up
#endif
        } else {
#ifndef __focus_adjustment
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss);   // write the new set-up
#else
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss, 0);   // write the new set-up	
#endif
        }
    } 
    else if (xml_def_on == Settings_From_Xml) {
        if (boost_fs::is_regular_file(SONY_XML_FILE)) {
            cli::tout << "the xml file exists" << "\n";
        }
        else {
            cli::tout << "the xml file is not existing" << "\n";
            xml_def_on = No_Settings;
            goto done_reading;
        }
        std::string xml_file(SONY_XML_FILE);
        Parent xml_param;
        xml_load(xml_file, xml_param);
        for (std::vector<Child>::const_iterator cit = xml_param.children.begin(); cit != xml_param.children.end(); cit++) {
            if ((levenstein("APERTURE", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "APERTURE") == true)) {
                sony_aper.change_tag_name(cit->option_name);
                ini_ap = clamp(cit->xml_val, 250, 2200);                                                  // TODO :: consider reading these limits from XML file first
                std::cout << "found " << cit->option_name << " similar to APERTURE with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("ISO", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "ISO") == true)) {
                sony_iso.change_tag_name(cit->option_name);
                ini_iso = clamp(cit->xml_val, 0, 102400);
                std::cout << "found " << cit->option_name << " similar to ISO with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("SHUTTERSPD", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "SHUTTER_SPEED") == true)) {
                sony_ss.change_tag_name(cit->option_name);
                ini_ss = clamp(cit->xml_val, 65539, 19660810);
                std::cout << "found " << cit->option_name << " similar to SHUTTER_SPEED with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("STILL_CAP", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "STILL_CAP") == true)) {
                sony_sc.change_tag_name(cit->option_name);
                ini_sc = clamp(cit->xml_val, SDK::CrDrive_Single, SDK::CrDrive_SelfPortrait_2);
                std::cout << "found " << cit->option_name << " similar to STILL_CAP with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("EXPMODE", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "EXPO") == true)) {
                sony_expro.change_tag_name(cit->option_name);
                ini_exp = clamp(cit->xml_val, SDK::CrExposure_M_Manual, SDK::CrExposure_Movie_F_Mode);
                std::cout << "found " << cit->option_name << " similar to EXPOSURE_PROGRAM with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if (((levenstein("_WBMODE", cit->option_name) <= LEVENSTEIN_THRESH) || (levenstein("_WHITE_BAL", cit->option_name) <= LEVENSTEIN_THRESH)) || (algo::contains(cit->option_name, "WHITE") == true)) {
                sony_white_bal.change_tag_name(cit->option_name);
                ini_wb = clamp(cit->xml_val, SDK::CrWhiteBalance_AWB, SDK::CrWhiteBalance_Custom);
                std::cout << "found " << cit->option_name << " similar to WHITE_BALANCE with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("FOCUS_AREA", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "FOCUS_AREA") == true)) {
                sony_foc_mode.change_tag_name(cit->option_name);
                ini_fa = clamp(cit->xml_val, SDK::CrFocusArea_Unknown, SDK::CrFocusArea_Tracking_Flexible_Spot);
                std::cout << "found " << cit->option_name << " similar to FOCUS_AREA with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("FOCUS_MODE", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "FOCUS_MODE") == true)) {
                sony_foc_area.change_tag_name(cit->option_name);
                ini_fm = clamp(cit->xml_val, SDK::CrFocus_MF, SDK::CrFocus_PF);
                std::cout << "found " << cit->option_name << " similar to FOCUS_MODE with value : " << cit->xml_val << " using as a default " << std::endl;
            }
        }
		int r1;
	    if (g_gimbal_connected==1) {
#if defined(__focus_adjustment)
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss, sony_focus_settings.use_focus_setting);   // write the new set-up
#endif
		} else {
#ifndef __focus_adjustment
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss);   // write the new set-up
#else
            r1 = write_camsets_to_json2(ini_ap, ini_iso, ini_sc, ini_fm, ini_fa, ini_wb, ini_exp, ini_ss, 0);   // write the new set-up	
#endif
        }
    }
    else if (xml_def_on == Settings_From_Last) {       // read the current settings json file (this is spooled from the appliction on change
        boost_fs::path const json_file("/home/pi/cam_init/filename.json");
        if (boost_fs::is_regular_file(json_file)) {
            cli::tout << "the json file containing last settings exists" << "\n";
        }
        else {
            cli::tout << "the json file is not existing" << "\n";
            xml_def_on = No_Settings;
            goto done_reading;
        }
		std::tuple< int, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t > json_we_read;
		if (g_gimbal_connected==1) {
//#if defined(__focus_adjustment)
            json_we_read = read_camsets_from_json();
            sony_focus_settings.use_focus_setting = clamp(std::get<9>(json_we_read), SONY_SDK_FOCUS_FAR_FAST, SONY_SDK_FOCUS_Near_Fast);
            std::cout << "Near Far " << std::get<9>(json_we_read) << "\n";
            sony_focus_settings.prev_focus_setting = sony_focus_settings.use_focus_setting;
		} else {
//#else
            json_we_read = read_camsets_from_json();
        }
//#endif
        ini_ap = clamp(std::get<1>(json_we_read), 250, 2200);
        ini_iso = clamp(std::get<2>(json_we_read), 0, 102400);
        ini_ss = clamp(std::get<8>(json_we_read), 65539, 19660810);
        ini_fm = clamp(std::get<4>(json_we_read), SDK::CrFocus_MF, SDK::CrFocus_PF);
        ini_fa = clamp(std::get<5>(json_we_read), SDK::CrFocusArea_Unknown, SDK::CrFocusArea_Tracking_Flexible_Spot);
        ini_sc = clamp(std::get<3>(json_we_read), SDK::CrDrive_Single, SDK::CrDrive_SelfPortrait_2);
        ini_exp = clamp(std::get<7>(json_we_read), SDK::CrExposure_M_Manual, SDK::CrExposure_Movie_F_Mode);
        ini_wb = clamp(std::get<6>(json_we_read), SDK::CrWhiteBalance_AWB, SDK::CrWhiteBalance_Custom);

        std::cout << "xml_option " << std::get<0>(json_we_read) << "\n";
        std::cout << "Aperture " << std::get<1>(json_we_read) << "\n";
        std::cout << "Iso " << std::get<2>(json_we_read) << "\n";
        std::cout << "Still Cap Mode " << std::get<3>(json_we_read) << "\n";
        std::cout << "Focus Mode " << std::get<4>(json_we_read) << "\n";
        std::cout << "Focus Area " << std::get<5>(json_we_read) << "\n";
        std::cout << "White Bal " << std::get<6>(json_we_read) << "\n";
        std::cout << "Exposure Program Mode " << std::get<7>(json_we_read) << "\n";
        std::cout << "Shutter Speed " << std::get<8>(json_we_read) << "\n";
    }

done_reading:

    // we need to make the "wheel" bumpless for setting the shutter speed from the 
    // herelink raw_channel_radio signals so we set it accroding to the set-up instruction
    // 
    std::vector<std::uint32_t> poss_shutter_speeds;		
	std::vector<std::uint32_t> poss_iso;		
	std::vector<std::uint32_t> poss_aper;	
	
    if (g_gimbal_connected==1) {
        gimbal_cmd.setPort(GIMBAL_USB_CONNECTION);                                 // where you conencted your gimbal
        gimbal_cmd.setSpeed(GREMSY_GIMBAL_BAUD);                                   // gimbal BAUD RATE
        gimbal_cmd.setTimeout_sec(1);                                              // port timeout to 1 second

        if (gimbal_cmd.open() == false) {
            cli::tout << " gimbal serial port could not be opened please check and try again ....." << gimbal_cmd.getPort() << "\n";
        }    

        // this is an example showing how to send from serial port to gimbal its only a test
        //
        gimbal_cmd.write_byte_array(&send_bytes[0]); 

        //
        // test the shutter speed library it reads all the settings from the xml
        //
        poss_shutter_speeds = get_shutter_speeds_vector(SONY_XML_FILE);
		poss_iso = get_iso_from_vector(SONY_XML_FILE);
	    poss_aper = get_aper_from_vector(SONY_XML_FILE);
		
        //cli::tout << "\033[33m got the value for Iso as : " << ini_iso << std::endl;
        //cli::tout << " got the value for Shutter Speed as : " << ini_ss << std::endl;		
        //cli::tout << " got the value for Aperture as : \033[0m" << ini_ap << std::endl;
        //cli::tout << " xml def on " << xml_def_on << " " << No_Settings << std::endl;	
		
		if (xml_def_on != No_Settings) {
            get_index_from_iso(poss_iso, &focus_obj, ini_iso);	                              // set the index of the step advancer to that of the default that was read at set-up
			raw_iso_val = ini_iso;
            get_index_from_aper(poss_aper, &focus_obj, ini_ap);	                              // set the index of the step advancer to that of the default that was read at set-up
			raw_aper_val = ini_ap;
            get_index_from_shutter(poss_shutter_speeds, &focus_obj, ini_ss);	              // set the index of the step advancer to that of the default that was read at set-up
			raw_ss_val = ini_ss;			
        } else {
			focus_obj.use_sdk_iso = 1;                                                        // choose the default setting for the step advancer
			cli::tout << "poss iso " << poss_iso.size() << " " << poss_shutter_speeds.size() << std::endl;
		    raw_iso_val = get_iso_from_index(poss_iso, &focus_obj);			
            focus_obj.use_sdk_shut = 7;                                                       // set the focus object
            raw_ss_val = get_shutter_from_index(poss_shutter_speeds, &focus_obj);             // now get the actual value for it from the xml read
            focus_obj.use_sdk_fnum = 1;  
		    raw_aper_val = get_aper_from_index(poss_aper, &focus_obj);
        }			

        cli::tout << " got the value for Iso as : " << raw_iso_val << std::endl;
        cli::tout << " got the value for Shutter Speed as : " << raw_ss_val << std::endl;		
        cli::tout << " got the value for Aperture as : " << raw_aper_val << " index " << focus_obj.use_sdk_fnum << std::endl;
    }
	
//#if defined(__focus_adjustment)
    if (g_gimbal_connected==1) {
        get_index_from_shutter(poss_shutter_speeds, &focus_obj, ini_ss);
	}
//#endif

    // This would be set over mavlink as the new value
    // when the PARAM_EXT_VALUE is written for each respective tag name
    //
    // here we are loading the settings from the chosen sources
    //
	cli::tout << "xml_def_on == " << xml_def_on << "\n";
    if (xml_def_on != No_Settings) {
        sony_iso.add_new_req(ini_iso);
        sony_aper.add_new_req(ini_ap);
        sony_ss.add_new_req(ini_ss);
        sony_expro.add_new_req(ini_exp);
        sony_foc_mode.add_new_req(ini_fm);
        sony_foc_area.add_new_req(ini_fa);
        sony_white_bal.add_new_req(ini_wb);
        sony_sc.add_new_req(ini_sc);
    }

    // reset the power on the usb link to ensure it stays up
    //	
    // reset_usb_link();

    // initialise the boost logger system
    //
    init();
    logging::add_common_attributes();
    using namespace logging::trivial;
    src::severity_logger< severity_level > lg;
    BOOST_LOG_SEV(lg, info) << "sony cam app started";

    // lets try these
	// https://github.com/termoshtt/boost_log_sample/blob/master/src/set_filter.cpp
	//
    BOOST_LOG_TRIVIAL(trace) << "A trace severity message";
    BOOST_LOG_TRIVIAL(debug) << "A debug severity message";
    BOOST_LOG_TRIVIAL(info) << "An informational severity message";
    BOOST_LOG_TRIVIAL(warning) << "A warning severity message";
    BOOST_LOG_TRIVIAL(error) << "An error severity message";
    BOOST_LOG_TRIVIAL(fatal) << "A fatal severity message";	
	
    // set-up the wiring pi lib for DIN to control photo_taking
    // 
    //if (wiringPiSetup() < 0)
    //{
    //    fprintf(stderr, "Unable to setup wiringPi: %s\n", strerror(errno));
    //    return 1;
    //}
    //pinMode(pin, OUTPUT);                      //define this pin as an output.
    //pinMode(ip, INPUT);                        //define this pin as an input.

    // set-up falling edge interrupt fall to low state 0V
    //
    //if (wiringPiISR(ip, INT_EDGE_FALLING, &gpio4_interrupt) < 0)
    //{
    //    fprintf(stderr, "Unable to setup ISR: %s\n", strerror(errno));
    //    return 1;
    //}

    // Create a object for each item (I made them globals for the multitasker
    //
    //storage_object sony_iso("S_ISO");
    //storage_object sony_aper("S_APERTURE");

    // now the driver performs the actions as requested
    // if it sees a new request for a change of option was written to the object
    //
    if (sony_iso.my_value != sony_iso.my_prev_value) {
        cli::tout << "new value for sony iso " << sony_iso.my_value << "\n";
        BOOST_LOG_SEV(lg, debug) << "new value for sony iso " << sony_iso.my_value;
    }
    if (sony_aper.my_value != sony_aper.my_prev_value) {
        cli::tout << "new value for sony aper " << sony_aper.my_value << "\n";
        BOOST_LOG_SEV(lg, debug) << "new value for sony aper " << sony_aper.my_value;
    }

    // if the camera was successful this is after the method was performed
    // you would use this to update the object
    // 
    //sony_iso.add_new_value();

    // when you sent the PARAM_EXT_ACK over mavlink clear the update flag
    //
    //sony_iso.clr_update();

    // if the action was unsuccessful 
    // and you cant do it as its write protected then clear the request
    // to change the option that came over mavlink with this command
    //
    //sony_aper.clr_new_req();


    // Change global locale to native locale
    //
    std::locale::global(std::locale(""));
    std::string input = "";

    cli::tin.imbue(std::locale());
    cli::tout.imbue(std::locale());

    // timer defined interrupt code
    struct sigaction sa_alarm;
    struct itimerval itimer;

    // this is an alternate signal handler
    //
    memset(&sa_alarm, 0, sizeof(sa_alarm));
    sa_alarm.sa_handler = &alrm;
    sa_alarm.sa_flags = SA_RESTART;
    if (sigaction(SIGALRM, &sa_alarm, NULL) < 0) {
        perror("sigaction");
        exit(1);
    }

    // define the signal handlers which will clean up if needed
    //
    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGINT " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGINT";
    }

    if (signal(SIGQUIT, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGQUIT " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGQUIT";
    }
	
    if (signal(SIGFPE, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGFPE " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGFPE";
    }

    /*
        cant do

    if (signal(SIGKILL, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGKILL " << "\n";
        BOOST_LOG_SEV(lg,error) << "cat catch SIGKILL";
    }
    */

    if (signal(SIGSEGV, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGSEGV " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGSEGV";
    }

    if (signal(SIGPIPE, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGPIPE " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGPIPE";
    }

    if (signal(SIGTERM, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGTERM " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGTERM";
    }

    if (signal(SIGCONT, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGCONT " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGCONT";
    }

    /*
    if (signal(SIGSTOP, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGSTOP " << "\n";
        BOOST_LOG_SEV(lg,error) << "cat catch SIGSTOP";
    }
    */

    if (signal(SIGTSTP, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGTSTP " << "\n";
        BOOST_LOG_SEV(lg, error) << "cat catch SIGTSTP";
    }

    // kill -10 <pid>
	if (signal(SIGUSR1, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGUSR1 " << "\n";
        BOOST_LOG_SEV(lg, error) << "cant catch SIGUSR1";
    }

    // kill -12 <pid>
	if (signal(SIGUSR2, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGUSR2 " << "\n";
        BOOST_LOG_SEV(lg, error) << "cant catch SIGUSR2";
    }

    // Bus error (incorrect memory access)
	if (signal(SIGBUS, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGBUS " << "\n";
        BOOST_LOG_SEV(lg, error) << "cant catch SIGBUS";
    }

    // CPU Time Limit Exceeded (4.2BSD) report this error
	if (signal(SIGXCPU, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGBUS " << "\n";
        BOOST_LOG_SEV(lg, error) << "cant catch SIGBUS";
    }
	
    if (signal(SIGHUP, sig_handler) == SIG_ERR) {
        cli::tout << "cant catch SIGHUP " << "\n";
    }

    if (signal(SIGALRM, alrm) == SIG_ERR) {
        cli::tout << "cant catch SIGALRM " << "\n";
    }
	
    CrInt32u version = SDK::GetSDKVersion();
    int major = (version & 0xFF000000) >> 24;
    int minor = (version & 0x00FF0000) >> 16;
    int patch = (version & 0x0000FF00) >> 8;
    // int reserved = (version & 0x000000FF);

    cli::tout << "Remote SDK version: ";
    cli::tout << major << "." << minor << "." << std::setfill(TEXT('0')) << std::setw(2) << patch << "\n";

    // Load the library dynamically
    // cr_lib = cli::load_cr_lib();

    //cli::tout << "Initialize Remote SDK...\n";

#if defined(__APPLE__)
    char path[255]; /*MAX_PATH*/
    getcwd(path, sizeof(path) - 1);
    //cli::tout << "Working directory: " << path << '\n';
#else
        //cli::tout << "Working directory: " << fs::current_path() << '\n';
#endif
    // auto init_success = cr_lib->Init(0);
    auto init_success = SDK::Init();
    if (!init_success) {
        cli::tout << "Failed to initialize Remote SDK. Terminating.\n";
        // cr_lib->Release();
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    //cli::tout << "Remote SDK successfully initialized.\n\n";

    //cli::tout << "Enumerate connected camera devices...\n";
    // @@@@ made a global from this
    //SDK::ICrEnumCameraObjectInfo* camera_list = nullptr;
    // auto enum_status = cr_lib->EnumCameraObjects(&camera_list, 3);
    auto enum_status = SDK::EnumCameraObjects(&camera_list);
    if (CR_FAILED(enum_status) || camera_list == nullptr) {
        cli::tout << "No cameras detected. Connect a camera and retry.\n";
        BOOST_LOG_SEV(lg, error) << "No cameras detected connect camera and retry";
        // cr_lib->Release();
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    auto ncams = camera_list->GetCount();
    //cli::tout << "Camera enumeration successful. " << ncams << " detected.\n\n";

    //
    // !!!!!!!!!!!!!!!!!!! check but i suspect we just always use zero and exit if ncams is > 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //
    for (CrInt32u i = 0; i < ncams; ++i) {
        auto camera_info = camera_list->GetCameraObjectInfo(i);
        cli::text conn_type(camera_info->GetConnectionTypeName());
        cli::text model(camera_info->GetModel());
        cli::text id = TEXT("");
        if (TEXT("IP") == conn_type) {
            cli::NetworkInfo ni = cli::parse_ip_info(camera_info->GetId(), camera_info->GetIdSize());
            id = ni.mac_address;
        }
        else id = ((TCHAR*)camera_info->GetId());
        //cli::tout << '[' << i + 1 << "] !!!!!!!!!!!!!!!!!! model data " << model.data() << " ( id data " << id.data() << ")\n";
    }

    // ======================= dont need to choose ===========================================

    //cli::tout << std::endl << "Connect to camera with input number... 1 \n";
    //cli::tout << "input> ";
    cli::text connectNo = "1";
    //std::getline(cli::tin, connectNo);
    cli::tout << '\n';

    cli::tsmatch smatch;
    CrInt32u no = 0;
    while (true) {
        no = 0;
        if (std::regex_search(connectNo, smatch, cli::tregex(TEXT("[0-9]")))) {
            no = stoi(connectNo);
            if (no == 0)
                break;                                                               // finish

            if (camera_list->GetCount() < no) {
                //cli::tout << "input value over \n";
                //cli::tout << "input> ";                                              // Redo
                //std::getline(cli::tin, connectNo);
                connectNo = "1";
                continue;
            }
            else
                break;                                                               // ok
        }
        else
            break;                                                                   // not number
    }

    if (no == 0) {
        cli::tout << "Invalid Number. Finish App.\n";
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }

    // ============================================================================================= */

    //CrInt32u no = 1;                                                                 // we are only using one camera

    // @@@@ : made this a global
    //
    //typedef std::shared_ptr<cli::CameraDevice> CameraDevicePtr;
    //typedef std::vector<CameraDevicePtr> CameraDeviceList;
    //CameraDeviceList cameraList;                                                     // all
    //std::int32_t cameraNumUniq = 1;
    //std::int32_t selectCamera = 1;

    //cli::tout << "Connect to selected camera...\n";
    // @@@@ make global
    auto* camera_info = camera_list->GetCameraObjectInfo(no - 1);

    //cli::tout << "Create camera SDK camera callback object.\n";
    // @@@@ : made this a global
    //
    //CameraDevicePtr camera = CameraDevicePtr(new cli::CameraDevice(cameraNumUniq, nullptr, camera_info));
    camera = CameraDevicePtr(new cli::CameraDevice(cameraNumUniq, nullptr, camera_info));
    cameraList.push_back(camera);                                                   // add 1st

    //cli::tout << "Release enumerated camera list.\n";
    camera_list->Release();
    auto connect_status = camera->connect(SDK::CrSdkControlMode_Remote);
    if (!connect_status) {
        cli::tout << "Camera connection failed to initiate. Abort.\n";
        // cr_lib->Release();
        SDK::Release();
        std::exit(EXIT_FAILURE);
    }
    //cli::tout << "Camera connection successfully initiated!\n\n";

    // Interactive loop
    // for (;;) {
        /*
        cli::tout << "What would you like to do? Enter the corresponding number.\n";
        cli::tout
            << "(s) Status display and camera switching \n"
            << "(0) Connect / Disconnect \n"
            << "(1) Shutter Release \n"
            << "(2) Shutter Half Release in AF mode \n"
            << "(3) Shutter Half and Full Release in AF mode \n"
            << "(4) Continuous Shooting \n"
            << "(5) Aperture \n"
            << "(6) ISO \n"
            << "(7) Shutter Speed \n"
            << "(8) Live View \n"
            << "(9) Live View Imege Quality \n"
            << "(a) Position Key Setting \n"
            << "(b) Exposure Program Mode \n"
            << "(c) Still Capture Mode(Drive mode) \n"
            << "(d) Focus Mode \n"
            << "(e) Focus Area \n"
            // << "(f) Release Device \n"
            << "(11) FELock \n"
            << "(12) AWBLock \n"
            << "(13) AF Area Position(x,y) \n"
            << "(14) Selected MediaFormat \n"
            << "(15) Movie Rec Button \n"
            << "(16) White Balance \n"
            << "(17) Custom WB \n"
            << "(18) Zoom Operation \n"
#if defined(LIVEVIEW_ENB)
            << "(lv) LiveView Enable \n"
#endif
            << "(x) Exit\n";
        cli::tout << "input> ";
        cli::text action;
        std::getline(cli::tin, action);
        cli::tout << '\n';
        */
        //cli::tout << "Continuous shooting mode......";

        /* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        if (action == TEXT("x")) {
            CameraDeviceList::const_iterator it = cameraList.begin();
            for (std::int32_t j = 0; it != cameraList.end(); ++j, ++it) {
                if ((*it)->is_connected()) {
                    cli::tout << "Initiate disconnect sequence.\n";
                    auto disconnect_status = (*it)->disconnect();
                    if (!disconnect_status) {
                        // try again
                        disconnect_status = (*it)->disconnect();
                    }
                    if (!disconnect_status)
                        cli::tout << "Disconnect failed to initiate.\n";
                    //else
                    //  cli::tout << "Disconnect successfully initiated!\n\n";
                }
                (*it)->release();
            }
            break; // quit application loop
        }
        else if (action == TEXT("s")) {
            cli::tout << "Status display and camera switching.\n";
#if defined(LIVEVIEW_ENB)
            cli::tout << "number - connected - lvEnb - model - id\n";
#else
            cli::tout << "number - connected - model - id\n";
#endif
            CameraDeviceList::const_iterator it = cameraList.begin();
            for (std::int32_t i = 0; it != cameraList.end(); ++i, ++it)
            {
                cli::text model = (*it)->get_model();
                if (model.size() < 10) {
                    int32_t apendCnt = 10 - model.size();
                    model.append(apendCnt, TEXT(' '));
                }
                cli::text id = (*it)->get_id();
                std::uint32_t num = (*it)->get_number();
                if (selectCamera == num) { cli::tout << "* "; }
                else { cli::tout << "  "; }
                cli::tout << std::setfill(TEXT(' ')) << std::setw(4) << std::left << num
                    << " - " << ((*it)->is_connected() ? "true " : "false")
#if defined(LIVEVIEW_ENB)
                    << " - " << ((*it)->is_live_view_enable() ? "true " : "false")
#endif
                    << " - " << model.data()
                    << " - " << id.data() << std::endl;
            }
            cli::tout << std::endl << "Selected camera number = [" << selectCamera << "]" << std::endl << std::endl;
            cli::tout << "Choose a number :\n";
            cli::tout << "[-1] Cancel input\n";
            cli::tout << "[0]  Create new CameraDevice\n";
            cli::tout << "[1]  Switch cameras for controls\n";
            cli::tout << std::endl << "input> ";
            cli::text input;
            std::getline(cli::tin, input);
            cli::text_stringstream ss(input);
            int selected_index = 0;
            ss >> selected_index;
            if (selected_index < 0 || 1 < selected_index) {
                cli::tout << "Input cancelled.\n";
            }
            // new camera connect
            if (0 == selected_index) {
                enum_status = SDK::EnumCameraObjects(&camera_list);
                if (CR_FAILED(enum_status) || camera_list == nullptr) {
                    cli::tout << "No cameras detected. Connect a camera and retry.\n";
                }
                else
                {
                    cli::tout << "[-1] Cancel input\n";
                    ncams = camera_list->GetCount();
                    for (CrInt32u i = 0; i < ncams; ++i) {
                        auto camera_info = camera_list->GetCameraObjectInfo(i);
                        cli::text conn_type(camera_info->GetConnectionTypeName());
                        cli::text model(camera_info->GetModel());
                        cli::text id = TEXT("");
                        if (TEXT("IP") == conn_type) {
                            cli::NetworkInfo ni = cli::parse_ip_info(camera_info->GetId(), camera_info->GetIdSize());
                            id = ni.mac_address;
                        }
                        else id = ((TCHAR*)camera_info->GetId());
                        cli::tout << '[' << i + 1 << "] " << model.data() << " (" << id.data() << ") ";
                        CameraDeviceList::const_iterator it = cameraList.begin();
                        for (std::int32_t j = 0; it != cameraList.end(); ++j, ++it){
                            cli::text alreadyId = (*it)->get_id();
                            if (0 == id.compare(alreadyId)) {
                                cli::tout << "*";
                                break;
                            }
                        }
                        cli::tout << std::endl;
                    }
                    cli::tout << std::endl << "idx input> ";
                    std::getline(cli::tin, input);
                    cli::text_stringstream ss2(input);
                    int selected_no = 0;
                    ss2 >> selected_no;
                    if (selected_no < 1 || (std::int32_t)ncams < selected_no) {
                        cli::tout << "Input cancelled.\n";
                    }
                    else {
                        camera_info = camera_list->GetCameraObjectInfo(selected_no - 1);
                        cli::text conn_type(camera_info->GetConnectionTypeName());
                        cli::text model_select(camera_info->GetModel());
                        cli::text id_select = TEXT("");
                        if (TEXT("IP") == conn_type) {
                            cli::NetworkInfo ni = cli::parse_ip_info(camera_info->GetId(), camera_info->GetIdSize());
                            id_select = ni.mac_address;
                        }
                        else id_select = ((TCHAR*)camera_info->GetId());
                        bool findAlready = false;
                        CameraDeviceList::const_iterator it = cameraList.begin();
                        for (std::int32_t j = 0; it != cameraList.end(); ++j, ++it) {
                            if ((0 == (*it)->get_model().compare(model_select)) &&
                                (0 == (*it)->get_id().compare(id_select))) {
                                findAlready = true;
                                cli::tout << "Already connected!\n";
                                break;
                            }
                        }
                        if (false == findAlready) {
                            std::int32_t newNum = cameraNumUniq + 1;
                            CameraDevicePtr newCam = CameraDevicePtr(new cli::CameraDevice(newNum, nullptr, camera_info));
                            if (true == newCam->connect()) {
                                cameraNumUniq = newNum;
                                cameraList.push_back(newCam);                       // add
                                camera = newCam;                                    // switch target
                                selectCamera = cameraNumUniq;                       // latest
                            }
                        }
                    }
                    camera_list->Release();
                }
            }
            // switch device
            if (1 == selected_index) {
                cli::tout << std::endl << "number input> ";
                std::getline(cli::tin, input);
                cli::text_stringstream ss3(input);
                int input_no = 0;
                ss3 >> input_no;
                if (input_no < 1) {
                    cli::tout << "Input cancelled.\n";
                }
                else {
                    bool findTarget = false;
                    CameraDeviceList::const_iterator it = cameraList.begin();
                    for (; it != cameraList.end(); ++it) {
                        if ((*it)->get_number() == input_no) {
                            findTarget = true;
                            camera = (*it);
                            selectCamera = input_no;
                            break;
                        }
                    }
                    if (!findTarget) {
                        cli::tout << "The specified camera cannot be found!\n";
                    }
                }
            }
        }
        else if (action == TEXT("0")) {
            if (camera->is_connected()) {
                camera->disconnect();
            }
            else {
                camera->connect(SDK::CrSdkControlMode_Remote);
            }
        }

        >>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

        //else if (action == TEXT("f")) { /* ReleaseDevice */
        //    if (2 > cameraList.size()) {
        //        cli::tout << std::endl << "Does not execute if there is no camera to switch control after the release device of the specified camera." << std::endl;
        //    }
        //    else {
        //        CameraDeviceList::const_iterator it = cameraList.begin();
        //        for (; it != cameraList.end(); ++it) {
        //            if ((*it)->get_number() == camera->get_number()) {
        //                (*it)->release();
        //                cameraList.erase(it);
        //                break;
        //            }
        //        }
        //        it = cameraList.begin(); // switch to top of list
        //        camera = (*it);
        //        selectCamera = camera->get_number();
        //    }
        //}

        //else {
            //while (!camera->is_connected()) {
            //    cli::tout << "This camera is disabled or disconnected attempt reconnection\n" << std::endl;
            //    camera->connect();
            //cli::tout << "waiting 5 seconds!\n";
            // sleep(5);
            //cli::tout << "ready to capture image!\n";
            //continue;
            //}

            //if (action == TEXT("1")) {                             /* wait to take photo */


            // uncomment to activate the timed interrupt code int imgCnt = 50;
    int imgCnt = 0;

    // ------- activate a change of enumerated type at this rate
    //
    //itimer.it_value.tv_sec = itimer.it_interval.tv_sec = 5;
    //itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 300000;
    //itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 0;
    //itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 0;
    //if (setitimer(ITIMER_REAL, &itimer, NULL) < 0) {
    //    perror("set timer");
    //    exit(1);
    //}

    // ================== WAIT HERE FOR SIGNALS ===========================
    //

    // This is the timers which allow the code to continually cycle with time
    //
    auto t_start = std::chrono::high_resolution_clock::now();
    auto t_end = std::chrono::high_resolution_clock::now();
    auto tdiff = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    while (imgCnt > 0)
    {
        int st = clock();
        t_start = std::chrono::high_resolution_clock::now();
        cli::tout << "wall GAp : " << std::chrono::duration<double, std::milli>(t_start - t_end).count() << "\n";
        camera->capture_image();
        int en = clock();
        t_end = std::chrono::high_resolution_clock::now();
        cli::tout << "time : " << 1000.0f * (en - st) / double(CLOCKS_PER_SEC) << "\n";
        cli::tout << "wall : " << std::chrono::duration<double, std::milli>(t_end - t_start).count() << "\n";
        //imgCnt--;
        //usleep(250000);
        sleep(1);                                           /* always sleep and wait for the signal */
    }

    // invoke this method once on start-up to get the current set up into memory
    //
    // its private either move or just call one that calls it camera->load_properties();
    camera->get_aperture();
    sleep(2);

    // grab all data as a single shot and update the objects with the data
    //
    std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>> p;
    if (camera->LoadMavProperties(p) == 1) {
        if (xml_def_on == No_Settings) {
            sony_iso.update_from_cam_vec(p);
            sony_aper.update_from_cam_vec(p);
            sony_ss.update_from_cam_vec(p);
            sony_expro.update_from_cam_vec(p);
            sony_foc_mode.update_from_cam_vec(p);
            sony_foc_area.update_from_cam_vec(p);
            sony_white_bal.update_from_cam_vec(p);
            sony_sc.update_from_cam_vec(p);
            sony_focus_settings.use_focus_setting = get_nf_from_cam_vec(p);
            sony_focus_settings.prev_focus_setting = sony_focus_settings.use_focus_setting;
            sony_focus_settings.use_sdk_shut = get_shutspd_from_cam_vec(p);
            sony_focus_settings.prev_sdk_shut = sony_focus_settings.use_sdk_shut;
        }
        cli::tout << "done init read of camera properties " << "\n";
    }

    // to test the interrupt uncomment here 
    //
	/*  ======= @@ no pics uncomment back @@
    itimer.it_value.tv_sec = itimer.it_interval.tv_sec = 10;
    //itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 500000;
	itimer.it_value.tv_usec = itimer.it_interval.tv_usec = 0;
    if (setitimer(ITIMER_REAL, &itimer, NULL) < 0) {
        perror("set timer");
        exit(1);
    }
    */
	
    // if the camera was successful this is after the method was performed
    // you would use this to update the object
    // 
    //sony_iso.add_new_value();

    // when you sent the PARAM_EXT_ACK over mavlink clear the update flag
    //
    //sony_iso.clr_update();

    // if the action was unsuccessful 
    // and you cant do it as its write protected then clear the request
    // to change the option that came over mavlink with this command
    //
    //sony_aper.clr_new_req();

    // ==================== set each value to the defaults
    //

int fails=0;

#define CAM_UST 500000
//#define CAM_UST 1000000 
//#define CAM_UST 2500000     // the microseconds time to wait between each respective camera action
//#define CAM_UST 5500000
//#define NO_RANDOM           // uncomment when your using the mavlink interface rather than the regression test generator

write_json_data_into_file("/home/anthony/test.json");

for(;;) {
	    // test the stepper functionality this should set the values to their default as per their position in the xml values list
	    //
		// •••••••••••• Shutter Speed Step up/down ••••••••••••
        if (focus_obj.use_sdk_shut != focus_obj.prev_sdk_shut) {
            raw_ss_val = get_shutter_from_index(poss_shutter_speeds, &focus_obj);
            std::uint32_t enum_ss = enumerate_shutter_sony_a7(raw_ss_val);
            if (camera->SetShutterSpeedArgsInt(enum_ss) == 1) {
                g_new_value = true;
                g_write_end = std::chrono::high_resolution_clock::now();
                // bumpless transfer the new setting to the mavlink setpoint
                sony_ss.my_value = raw_ss_val;
                sony_ss.my_prev_value = raw_ss_val;
				focus_obj.prev_sdk_shut = focus_obj.use_sdk_shut;
				++fails;
			} else {
				fails = 0;
			}
        }	

        // •••••••••••• ISO Step up/down ••••••••••••
        if (focus_obj.use_sdk_iso != focus_obj.prev_sdk_iso) {
            raw_iso_val = get_iso_from_index(poss_iso, &focus_obj);
            std::uint32_t enum_iso = enumerate_iso_sony_a7(raw_iso_val);
            if (camera->SetIsoArgsInt(enum_iso) == 1) {
                g_new_value = true;
                g_write_end = std::chrono::high_resolution_clock::now();
                // bumpless transfer the new setting to the mavlink setpoint
                sony_iso.my_value = raw_iso_val;
                sony_iso.my_prev_value = raw_iso_val;
				focus_obj.prev_sdk_iso = focus_obj.use_sdk_iso;
				++fails;
			} else {
				fails = 0;
			}
        }	

        // •••••••••••• FNUM / Aperture Step up/down ••••••••••••		
        if (focus_obj.use_sdk_fnum != focus_obj.prev_sdk_fnum) {
            raw_aper_val = get_aper_from_index(poss_aper, &focus_obj);
            std::uint32_t enum_aper = enumerate_aperture_sony_a7(raw_aper_val);
            if (camera->SetApertureArgsInt(enum_aper) == 1) {
                g_new_value = true;
                g_write_end = std::chrono::high_resolution_clock::now();
                // bumpless transfer the new setting to the mavlink setpoint
                sony_aper.my_value = raw_aper_val;
                sony_aper.my_prev_value = raw_aper_val;
				focus_obj.prev_sdk_fnum = focus_obj.use_sdk_fnum;
				fails = 0;
			} else {
				++fails;
			}
        }	
		
	// same as above using a vector of tuples as the globally passed object
	//
	
    //for (auto v : g_vt) {
	//	std::regex re("shutter_speed");
	//	if (std::regex_match(get<2>(v), re)) {
	//		if (get<0>(v) != get<1>(v)) {			
	//		    std::uint32_t enum_ss = enumerate_shutter_sony_a7(get<0>(v));
	//		    if (camera->SetShutterSpeedArgsInt(enum_ss) == 1) {
	//              g_vt.emplace_back(std::make_tuple(get<0>(v), get<0>(v), get<2>(v)));				
	//		    } else {
	//               g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));				
	//		    }
	//		} else {
	//		    g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));	
	//		}
	//	} 
		
	//	std::regex re1("fnum");
	//	if (std::regex_match(get<2>(v), re1)) {
	//		if (get<0>(v) != get<1>(v)) {			
	//		    std::uint32_t enum_fnum = enumerate_aperture_sony_a7(get<0>(v));
	//		    if (camera->SetApertureArgsInt(enum_fnum) == 1) {
	//              g_vt.emplace_back(std::make_tuple(get<0>(v), get<0>(v), get<2>(v)));				
	//		    } else {
	//               g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));				
	//		    }
	//		} else {
	//		    g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));	
	//		}				
	//	}
		
	//	std::regex re2("iso");
	//	if (std::regex_match(get<2>(v), re2)) {
	//		if (get<0>(v) != get<1>(v)) {			
	//		    std::uint32_t enum_iso = enumerate_iso_sony_a7(get<0>(v));
	//		    if (camera->SetIsoArgsInt(enum_iso) == 1) {
	//               g_vt.emplace_back(std::make_tuple(get<0>(v), get<0>(v), get<2>(v)));				
	//		    } else {
	//               g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));				
	//		    }
	//		} else {
	//		    g_vt.emplace_back(std::make_tuple(get<0>(v), get<1>(v), get<2>(v)));	
	//		}						
	//	}		
    //}
	
    // aperture
    if (sony_aper.my_value != sony_aper.my_prev_value) {	
	    #if defined(NO_RANDOM)
        std::uint16_t cam_enum_aper = enumerate_aperture_sony_a7(sony_aper.my_value);
		#else
		std::uint16_t cam_enum_aper = sony_aper.my_value;	
		#endif
		if (cam_enum_aper != SONY_ENUM_INVALID) {
            if (camera->SetApertureArgsInt(cam_enum_aper)==1)  {
               cli::tout << " success setting aperture " << sony_aper.my_value << "\n";
               sony_aper.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_aper.clr_update();
               fails = 0;			   
	        } else {
               cli::tout << " failure setting aperture \n";
               sony_aper.clr_new_req();
               ++fails;			
	        }
		} else {
            sony_aper.clr_new_req();            
		}
		usleep(CAM_UST);
	}

    // shutter speed
    if (sony_ss.my_value != sony_ss.my_prev_value) {
		#if defined(NO_RANDOM)
        std::uint32_t cam_enum_ss = enumerate_shutter_sony_a7(sony_ss.my_value);
		#else
		std::uint32_t cam_enum_ss = sony_ss.my_value;	
		#endif
		if (cam_enum_ss != SONY_ENUM_INVALID) {
            if (camera->SetShutterSpeedArgsInt(cam_enum_ss)==1) {
               cli::tout << " success setting shutter speed" << sony_ss.my_value << "\n";
               sony_ss.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_ss.clr_update();
               fails = 0;			   
	        } else {
                cli::tout << " failure setting shutter speed \n";
                sony_ss.clr_new_req();
                ++fails;			
	        }
		}
		usleep(CAM_UST);
	}

    // iso
    if (sony_iso.my_value != sony_iso.my_prev_value) {
		#if defined(NO_RANDOM)
        std::uint32_t cam_enum_iso = enumerate_iso_sony_a7(sony_iso.my_value);
		#else
		std::uint32_t cam_enum_iso = sony_iso.my_value;
        #endif	
		if (cam_enum_iso != SONY_ENUM_INVALID) {
            if (camera->SetIsoArgsInt(cam_enum_iso)==1) {
               cli::tout << " success setting iso " << sony_iso.my_value << "\n";
               sony_iso.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_iso.clr_update();		 
               fails = 0;	
	        } else {
               cli::tout << " failure setting iso \n";
               sony_iso.clr_new_req();	
               ++fails;		   
	        }
		}
		usleep(CAM_UST);
	}

    // white balance
    if (sony_white_bal.my_value != sony_white_bal.my_prev_value) {
		#if defined(NO_RANDOM)
        std::uint16_t cam_enum_wb = enumerate_white_bal_sony_a7(sony_white_bal.my_value);
		#else
		std::uint16_t cam_enum_wb = sony_white_bal.my_value;	
		#endif
		if (cam_enum_wb != SONY_ENUM_INVALID) {
           if (camera->SetWhiteBalanceArgsInt(cam_enum_wb)==1) {
               cli::tout << " success setting white balance \n";
    		   camera->printCrWhiteBalanceSetting(static_cast<SCRSDK::CrWhiteBalanceSetting>(sony_white_bal.my_value));
               sony_white_bal.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_white_bal.clr_update();		 
               fails = 0;	
     	   } else {
               cli::tout << " failure setting white balance \n";
               sony_white_bal.clr_new_req();
               ++fails;		   
	       }
		}
		usleep(CAM_UST);
	}

    // focus area
    if (sony_foc_area.my_value != sony_foc_area.my_prev_value) {
		#if defined(NO_RANDOM)
        std::uint16_t cam_enum_fa = enumerate_focus_area_sony_a7(sony_foc_area.my_value);
		#else
		std::uint16_t cam_enum_fa = sony_foc_area.my_value;
		#endif
		if (cam_enum_fa != SONY_ENUM_INVALID) {
            if (camera->SetFocusAreaArgsInt(cam_enum_fa)==1) {
               cli::tout << " success setting focus area \n";
		       camera->printCrFocusArea(static_cast<SCRSDK::CrFocusArea>(sony_foc_area.my_value));
               sony_foc_area.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_foc_area.clr_update();	
               fails = 0;				   
	        } else {
               cli::tout << " failure setting focus area \n";
               sony_foc_area.clr_new_req();
               ++fails;		   
	        }
		}
		usleep(CAM_UST);
	}
	
    // focus mode
    if (sony_foc_mode.my_value != sony_foc_mode.my_prev_value) {
		#if defined(NO_RANDOM)
        std::uint16_t cam_enum_fm = enumerate_focus_sony_a7(sony_foc_mode.my_value);
		#else
		std::uint16_t cam_enum_fm = sony_foc_mode.my_value;	
		#endif
		if (cam_enum_fm != SONY_ENUM_INVALID) {
            if (camera->SetFocusModeArgsInt(cam_enum_fm)==1) {
               cli::tout << " success setting focus mode \n";
		       camera->printCrFocusMode(static_cast<SCRSDK::CrFocusMode>(sony_foc_mode.my_value));
               sony_foc_mode.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_foc_mode.clr_update();		
               fails = 0;				   
	        } else {
               cli::tout << " failure setting focus mode \n";
               sony_foc_mode.clr_new_req();	
               ++fails;		   
	        }
		}
		usleep(CAM_UST);
	}
	
    // still capture mode
    if (sony_sc.my_value != sony_sc.my_prev_value) {
	    #if defined(NO_RANDOM)
        std::uint32_t cam_enum_sc1 = enumerate_still_cap_sony_a7(sony_sc.my_value);
		#else
		std::uint32_t cam_enum_sc1 = sony_sc.my_value;
        #endif	
		if (cam_enum_sc1 != SONY_ENUM_INVALID) {
            if (camera->SetStillCaptureModeArgsInt(cam_enum_sc1)==1) {
               cli::tout << " success setting still capture mode \n";
		       camera->printCrDriveMode(static_cast<SCRSDK::CrDriveMode>(sony_sc.my_value));
               sony_sc.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_sc.clr_update();	
               fails = 0;				   
	        } else {
               cli::tout << " failure setting still capture mode \n";
               sony_sc.clr_new_req();	
               ++fails;		   
	        }
		}
		usleep(CAM_UST);
	}

    // exposure program mode
    if (sony_expro.my_value != sony_expro.my_prev_value) {
	    #if defined(NO_RANDOM)
        std::uint16_t cam_enum_ep = enumerate_ex_pro_sony_a7(sony_expro.my_value);
		#else
		std::uint16_t cam_enum_ep = sony_expro.my_value;
	    #endif
		if (cam_enum_ep != SONY_ENUM_INVALID) {		
            if (camera->SetExposureProgramModeArgsInt(cam_enum_ep)==1) {
               cli::tout << " success setting still capture mode \n";
		       camera->printCrExposureProgram(static_cast<SCRSDK::CrExposureProgram>(sony_expro.my_value));
               sony_expro.add_new_value();	
               // when you send the PARAM_EXT_ACK send this (would be in the other program
               sony_expro.clr_update();	
               fails = 0;			   
	       } else {
               cli::tout << " failure setting still capture mode \n";
               sony_expro.clr_new_req();
               ++fails;		   
	       }
		}
		usleep(CAM_UST);
	}
	
	// focus position
    if (focus_obj.use_focus_setting != focus_obj.prev_focus_setting) {
        camera->set_focus_position(focus_obj.use_focus_setting);
        focus_obj.prev_focus_setting = focus_obj.use_focus_setting;
        g_new_value = true;
        g_write_end = std::chrono::high_resolution_clock::now();
    }
	
    // write the new setting if we got a change of state
    //
    auto g_write_tdiff = std::chrono::duration<double, std::milli>(g_write_end - g_write_start).count();
    if ((g_new_value == true) && (g_write_tdiff > NO_ACTIVITY_DURATION)) {
		cli::tout << " \033[32m Time diff meant write the file \033[0m" << std::endl; 
        g_write_start = std::chrono::high_resolution_clock::now();
        g_write_end = std::chrono::high_resolution_clock::now();
		int r1;
		if (g_gimbal_connected==1) {
#if defined(__focus_adjustment)
            r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value, sony_focus_settings.use_focus_setting);   // write the new set-up
#endif
        } else {
#ifndef __focus_adjustment
            r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value);   // write the new set-up
#else
            r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value, 0);	
#endif
        }
        g_new_value = false;
    }
	
	// if we lost connection we should attempt a reconnection
    if (!camera->is_connected()) {
		cli::tout << "\033[31m Camera disconnect occurred - reconnect it ! \033[0m" << std::endl;
		try {
            auto conn_stat = camera->connect(SDK::CrSdkControlMode_Remote);
			if (!conn_stat) {
                cli::tout << "Camera connection failed to initiate. Aborting.\n";
                // cr_lib->Release();
                SDK::Release();
                std::exit(EXIT_FAILURE);
            } else {
				fails=0;                      // success doesnt reset the usb power line 
			}
		} catch (std::bad_alloc e) {
            std::cerr << "Sony Connect :: failed to alloc memory: " << e.what() << std::endl;
			SDK::Release();
            std::exit(EXIT_FAILURE);
        } catch (std::runtime_error e) {
            std::cerr << "Sony Connect :: runtime error: " << e.what() << std::endl;
			SDK::Release();
            std::exit(EXIT_FAILURE);			
		} catch (...)  {
            std::cerr << "Sony Connect :: unknown exeption" << std::endl;
			SDK::Release();
            std::exit(EXIT_FAILURE);
        }
		sleep(5);
    }
	
    // 10 consecutive fails will reset the usb power line
    if (fails >= 10 ) {
		fails=0;
		cli::tout << "\033[31m FAILURE RESET APPLIED \033[0m" << std::endl;
		reset_usb_sony();
	}

} // ============== end for loop =====================================

	
    // get the recording state
    std::uint32_t recording_state = get_rec_state_from_cam_vec(p);

    // set the aperture using a global structure
    //
    // ------------ this is the example of stranding via boost (does it work okay) -------------
    //
	
/* ========================= took out the stranding dont think its needed anymore ===================================================

    event_manager sony_alpha7_cam;
    Button set_aperture("aperture");
    Button set_wb("white_balance");
    Button set_fm("focus_mode");
    Button set_fa("focus_area");
    Button set_iso("iso");
    Button set_ss("shutter_speed");
    Button set_sc("still_capture");
    Button set_expro("exposure_program");

    g_settings.set_ap = 4;
    g_settings.set_fa = 1;
    g_settings.set_fm = 2;
    g_settings.set_ss = 2;
    g_settings.set_sc = 2;
    g_settings.set_ex = 2;
    g_settings.set_iso = 2;
    g_settings.set_wb = 3;

    sony_alpha7_cam.set_sony_aper_event(boost::bind(&Button::set_cam_aper, &set_aperture));
    sony_alpha7_cam.do_sony_aper();
    sony_alpha7_cam.set_sony_wb_event(boost::bind(&Button::set_cam_wb, &set_wb));
    sony_alpha7_cam.set_sony_fm_event(boost::bind(&Button::set_cam_fm, &set_fm));
    sony_alpha7_cam.set_sony_fa_event(boost::bind(&Button::set_cam_fa, &set_fa));
    sony_alpha7_cam.set_sony_iso_event(boost::bind(&Button::set_cam_iso, &set_iso));
    sony_alpha7_cam.set_sony_ss_event(boost::bind(&Button::set_cam_ss, &set_ss));
    sony_alpha7_cam.set_sony_sc_event(boost::bind(&Button::set_cam_sc, &set_sc));
    sony_alpha7_cam.set_sony_expro_event(boost::bind(&Button::set_cam_expro, &set_expro));
    sony_alpha7_cam.do_sony_wb();
    sony_alpha7_cam.do_sony_fa();
    sony_alpha7_cam.do_sony_fm();
    sony_alpha7_cam.do_sony_iso();
    sony_alpha7_cam.do_sony_expro();
    sony_alpha7_cam.cancel_button_click();
    sony_alpha7_cam.do_sony_fa();

    // now do another set of objects
    //
    Button set_aperture1("aperture");
    Button set_wb1("white_balance");
    Button set_fm1("focus_mode");
    Button set_fa1("focus_area");
    Button set_iso1("iso");
    Button set_ss1("shutter_speed");
    Button set_sc1("still_capture");
    Button set_expro1("exposure_program");
    // can also be set like this
    // 
    set_aperture1.set_value(1);
    set_wb1.set_value(1);
    set_fm1.set_value(1);
    set_fa1.set_value(2);
    set_iso1.set_value(2);
    set_ss1.set_value(2);
    set_sc1.set_value(2);
    set_expro1.set_value(3);
    // now call these methods
    //
    sony_alpha7_cam.do_sony_aper1();
    sony_alpha7_cam.do_sony_wb1();
    sony_alpha7_cam.do_sony_fa1();
    sony_alpha7_cam.do_sony_fm1();
    sony_alpha7_cam.do_sony_iso1();
    sony_alpha7_cam.do_sony_expro1();
    sony_alpha7_cam.do_sony_sc1();
    sony_alpha7_cam.do_sony_ss1();
	
	
    ===================================== end of stranding ========================================== */

      // this repeat is just used for testing
      //
      // ===========================================================>> for (int z=0; z < 5; z++) { <<==================================================================================

/* ---------------------------------------------- uncommented out -------------------------------------------------------------------------------------------------------------------
	for (;;) {

        // use a random number generator to send values to the camera quickly and test stranding
        //
		
     ==================== take out the boost stranding for now ===========================================================
        vector<vector<int>> v;
        v = get_test_indexs(19, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_aperture1.set_value(i);
                sony_alpha7_cam.do_sony_aper1();
                std::cout << " aperture set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(14, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_wb1.set_value(i);
                sony_alpha7_cam.do_sony_wb1();
                std::cout << " white balance set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(14, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_fa1.set_value(i);
                sony_alpha7_cam.do_sony_fa1();
                std::cout << " fa set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(4, 3);
        for (auto z : v) {
            for (auto i : z) {
                set_fm1.set_value(i);
                sony_alpha7_cam.do_sony_fm1();
                std::cout << " fm set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(7, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_fa1.set_value(i);
                sony_alpha7_cam.do_sony_fa1();
                std::cout << " fa set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(34, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_iso1.set_value(i);
                sony_alpha7_cam.do_sony_iso1();
                std::cout << " fa set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(49, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_sc1.set_value(i);
                sony_alpha7_cam.do_sony_sc1();
                std::cout << " still capture set val :: " << i << std::endl;
            }
        }
        v = get_test_indexs(55, 5);
        for (auto z : v) {
            for (auto i : z) {
                set_ss1.set_value(i);
                sony_alpha7_cam.do_sony_ss1();
                std::cout << " shutter speed set val :: " << i << std::endl;
            }
        }
		


        if (g_gimbal_connected==1) {
//#if defined(__focus_adjustment)
        // ============ this is the code for enumerting the herelink shutter speed controls ==============
        //
		//  $$$$$$$$$$ commwnted out shutter speed adjustments as casuing issues
		//
        //if (focus_obj.use_sdk_shut != focus_obj.prev_sdk_shut) {
        //    raw_ss_val = get_shutter_from_index(poss_shutter_speeds, &focus_obj);
        //    std::uint32_t enum_ss = enumerate_shutter_sony_a7(raw_ss_val);
        //    camera->SetShutterSpeedArgsInt(enum_ss);
        //    g_new_value = true;
        //    g_write_end = std::chrono::high_resolution_clock::now();
            // bumpless transfer the new setting to the mavlink setpoint
        //    sony_ss.my_value = raw_ss_val;
        //    sony_ss.my_prev_value = raw_ss_val;
        //}

        // =========== this is the code for enumerting the herelink for focus near far ==============
        //
            if (focus_obj.use_focus_setting != focus_obj.prev_focus_setting) {
                camera->set_focus_position(focus_obj.use_focus_setting);
                focus_obj.prev_focus_setting = focus_obj.use_focus_setting;
                g_new_value = true;
                g_write_end = std::chrono::high_resolution_clock::now();
            }
		}
//#endif

        // write the new setting if we got a change of state
        //
        auto g_write_tdiff = std::chrono::duration<double, std::milli>(g_write_end - g_write_start).count();
        if ((g_new_value == true) && (g_write_tdiff > NO_ACTIVITY_DURATION)) {
            g_write_start = std::chrono::high_resolution_clock::now();
            g_write_end = std::chrono::high_resolution_clock::now();
			int r1;
			if (g_gimbal_connected==1) {
#if defined(__focus_adjustment)
                r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value, sony_focus_settings.use_focus_setting);   // write the new set-up
#endif
            } else {
#ifndef __focus_adjustment
                r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value);   // write the new set-up
#else
                r1 = write_camsets_to_json2(sony_aper.my_value, sony_iso.my_value, sony_sc.my_value, sony_foc_mode.my_value, sony_foc_area.my_value, sony_white_bal.my_value, sony_expro.my_value, sony_ss.my_value, 0);	
#endif
            }
            g_new_value = false;
        }

    } // $$$$$$$$$$$$$$$$$$$$$$$$$$$ for ever
*/

            //}  ===============================================>> end of for loop repeat <<====================================================================================

            //else if (action == TEXT("6")) { /* ISO */
                //sleep(5);
                //camera->get_iso();
                //sleep(5);
                //camera->set_iso_args( input );
                //sleep(5);
                //camera->get_iso();
            //}
            //else if (action == TEXT("7")) { /* Shutter Speed */
                //sleep(5);
                //camera->get_shutter_speed();
                //camera->set_shutter_speed_args( input );
                //sleep(5);
                //camera->get_shutter_speed();
            //}
            //else if (action == TEXT("8")) { /* Live View */
            //    camera->get_live_view();
            //}
            //else if (action == TEXT("9")) { /* Live View Image Quality */
            //    camera->get_live_view_image_quality();
            //    camera->set_live_view_image_quality();
            //}
            //else if (action == TEXT("10")) { /* Live View Image Status */
            //    camera->get_live_view_status();
            //    camera->set_live_view_status();
            //}
            //else if (action == TEXT("a")) { /* Position Key Setting */
            //    camera->get_position_key_setting();
            //    camera->set_position_key_setting();
            //}
            //else if (action == TEXT("b")) { /* Exposure Program Mode */
            //sleep(5);
            //camera->get_exposure_program_mode();
            //sleep(5);
            //camera->set_exposure_program_mode_args( input );
            //sleep(5);
            //camera->get_exposure_program_mode();
            //}
            //else if (action == TEXT("c")) { /* Still Capture Mode(Drive mode) */
            //  sleep(5);
            //  camera->get_still_capture_mode();
            //  camera->set_still_capture_mode_args( input );
            //  sleep(5);
            //  camera->get_still_capture_mode();
            //}
            //else if (action == TEXT("d")) { /* Focus Mode */
            //  sleep(5);
            //  camera->get_focus_mode();
            //  sleep(5);
            //  camera->set_focus_mode_args( input );
            //  sleep(5);
            //  camera->get_focus_mode();
            //}
            //else if (action == TEXT("e")) { /* Focus Area */
            //    sleep(5);
            //    camera->get_focus_area();
            //    camera->set_focus_area();
            //}
            //else if (action == TEXT("11")) { /* FELock */
            //    cli::tout << "Flash device required.";
            //    camera->execute_lock_property((CrInt16u)SDK::CrDevicePropertyCode::CrDeviceProperty_FEL);
            //}
            //else if (action == TEXT("12")) { /* AWBLock */
            //    camera->execute_lock_property((CrInt16u)SDK::CrDevicePropertyCode::CrDeviceProperty_AWBL);
            //}
            //else if (action == TEXT("13")) { /* AF Area Position(x,y) */
            //    camera->set_af_area_position();
            //}
            //else if (action == TEXT("14")) { /* Selected MediaFormat */
            //    camera->get_select_media_format();
            //    camera->set_select_media_format();
            //}
            //else if (action == TEXT("15")) { /* Movie Rec Button */
            //  sleep(5);
            //  camera->execute_movie_rec();
            //}
            //else if (action == TEXT("16")) { /* White Balance */
            //  sleep(5);
            //  camera->get_white_balance();
            //  camera->set_white_balance_args( input );
            //  sleep(5);
            //  camera->get_white_balance();
            //}
            //else if (action == TEXT("17")) { /* Custom WB */
            //    camera->get_custom_wb();
            //    camera->set_custom_wb();
            //}
            //else if (action == TEXT("18")) { /* Zoom Operation */
            //    camera->get_zoom_operation();
            //    camera->set_zoom_operation();
            //}
#if defined(LIVEVIEW_ENB)
            //else if (action == TEXT("lv")) { /* LiveView Enable */
            //    camera->change_live_view_enable();
            //}
#endif
        //}
        //cli::tout << '\n';
    //}

    // now disconnect the device
    //
    CameraDeviceList::const_iterator it = cameraList.begin();
    for (std::int32_t j = 0; it != cameraList.end(); ++j, ++it) {
        if ((*it)->is_connected()) {
            //cli::tout << "Initiate disconnect sequence.\n";
            auto disconnect_status = (*it)->disconnect();
            if (!disconnect_status) {
                // try again
                disconnect_status = (*it)->disconnect();
            }
            if (!disconnect_status)
                cli::tout << "Disconnect failed to initiate.\n";
            //else
            //  cli::tout << "Disconnect successfully initiated!\n\n";
        }
        (*it)->release();
    }

    //cli::tout << "Release SDK resources.\n";
    // cr_lib->Release();
    SDK::Release();

    // cli::free_cr_lib(&cr_lib);

    //cli::tout << "Exiting application.\n";
    std::exit(EXIT_SUCCESS);
}

//
//  •••••••••••• its a test simulating the actions of the incoming mavlink communicator (GCS commanded actions) ••••••••••••
//
void mavlink_communicator()
{
    static int cnt2 = 0;
	// list the possible options for these actions
	std::vector<std::uint32_t> poss_shutter_speeds_list = get_shutter_speeds_vector(SONY_XML_FILE);
	std::vector<std::uint32_t> poss_iso_list = get_iso_from_vector(SONY_XML_FILE);
	std::vector<std::uint32_t> poss_aper_list = get_aper_from_vector(SONY_XML_FILE);
	
    while (isRuning)
    {
        //cout << "\tfunc2:" << ++cnt2 << endl;
        sleep(2);
        //sony_aper.add_new_req((std::uint32_t)cnt2);
        if ((cnt2 % 10) == 0)  cnt2 = 0;
			vector<vector<int>> v;	
    for (;;) {

		// test setting the aperture
        v = get_test_indexs(19, 5);
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m aperture request val :: \033[0m " << i << std::endl;
				usleep(CAM_UST);
				sony_aper.add_new_req(i);
            }
        }
		
		// test setting of white balance
        v = get_test_indexs(14, 5);
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m white balance request val :: \033[0m " << i << std::endl;
				usleep(CAM_UST);
				sony_white_bal.add_new_req(i);
            }
        }
		
        // test setting of iso
        v = get_test_indexs(34, 5);
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m iso request val :: \033[0m " << i << std::endl;
				usleep(CAM_UST);
				sony_iso.add_new_req(i);
            }
        }
		
        // test setting of shutter speed
        v = get_test_indexs(55, 5);
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m shutter speed request val :: \033[0m " << i << std::endl;
				usleep(CAM_UST);
				sony_ss.add_new_req(i);
            }
        }

        // test of near/far focus
        v = get_test_indexs(7, 5);
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m near/far focus :: \033[0m " << i << std::endl;
				usleep(CAM_UST);
				focus_obj.use_focus_setting = i;
            }
        }
		// now for -ve signals
        for (auto z : v) {
            for (auto i : z) {
                std::cout << " \033[32m near/far focus with negative :: \033[0m " << -i << std::endl;
				usleep(CAM_UST);
				focus_obj.use_focus_setting = -i;
            }
        }
		
		// increment the option as if we sent an RC Channel from the handheld
        if (focus_obj.use_sdk_shut == focus_obj.prev_sdk_shut) {
			++focus_obj.use_sdk_shut % poss_shutter_speeds_list.size();
            std::cout << "\033[34m set shut increment \033[0m" << std::endl;
        }
        if (focus_obj.use_sdk_fnum == focus_obj.prev_sdk_fnum) {
			++focus_obj.use_sdk_fnum % poss_aper_list.size();
            std::cout << "\033[33m set fnum increment \033[0m" << std::endl;
        }	
        if (focus_obj.use_sdk_iso == focus_obj.prev_sdk_iso) {
			++focus_obj.use_sdk_iso % poss_iso_list.size();
            std::cout << "\033[33m set iso increment \033[0m" << std::endl;
        }			
    }
    }
    sleep(3);
}
void mavlink_heartbeat()
{
    static int cnt2 = 0;
    while (isRuning)
    {
        cout << "\theartbeat:" << ++cnt2 << endl;
        sleep(1);
    }
    sleep(3);
}

int main(void)
{
    // start your threads here
    boost::thread thread1(&cam_main);
    boost::thread thread2(&mavlink_communicator);
    boost::thread thread3(&mavlink_heartbeat);
    system("read");
    // tell the threads to complete using the global
    // isRuning = false;
    // wait for the threads here
    thread3.join();
    thread2.join();
    thread1.join();
    cout << "exit" << endl;
    return 0;
}


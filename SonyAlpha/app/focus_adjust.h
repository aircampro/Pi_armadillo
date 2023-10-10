#ifndef __focus_adjustment
#define __focus_adjustment
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include "mavlink_status_message.h"

// define the constraints
#define MOD_MIN_TIMER 0
#define MOD_MAX_TIMER 60000
#define MOD_HI_VAL_CLAMP 1500
#define MOD_LO_VAL_CLAMP 0
#define SH_LOW_LIM -7
#define SH_HI_LIM 7

typedef struct {
    std::uint16_t mo_rc12fmu_mode_swap;
    std::uint16_t mo_rc12fmu_tm1;
    std::uint16_t hw_def_mode;
    std::uint16_t prev_def_mode;
    std::chrono::high_resolution_clock::time_point tim_start_1;
    std::uint16_t gi_rc11fmu_gim_spt1;
    std::uint16_t gi_rc11fmu_gim_spt2;
    std::uint16_t gi_rc11fmu_gim_lim1;
    std::uint16_t fo_rc10fmu_spt1;
    std::uint16_t fo_rc10fmu_spt2;
    std::uint16_t fo_rc10fmu_spt3;
    std::uint16_t fo_rc10fmu_spt4;
    std::uint16_t fo_rc10fmu_spt5;
    std::uint16_t fo_rc10fmu_spt6;
    std::uint16_t fo_rc10fmu_spt7;
    std::uint16_t fo_rc10fmu_spt8;
    std::uint16_t fo_rc10fmu_spt9;
    std::uint16_t fo_rc10fmu_spt10;
    std::uint16_t fo_rc10fmu_spt11;
    std::uint16_t fo_rc10fmu_spt12;
    std::uint16_t fo_rc10fmu_tm1;
    std::uint16_t sh_rc10fmu_spt1;
    std::uint16_t sh_rc10fmu_spt2;
    std::uint16_t sh_rc10fmu_spt3;
    std::uint16_t sh_rc10fmu_tm1;
    std::uint16_t use_focus_setting;
    std::uint16_t prev_focus_setting;
    std::uint16_t use_sdk_shut;
    std::uint16_t prev_sdk_shut;
    std::uint16_t mode_step : 12;
    std::uint16_t shut_step : 4;
    std::uint16_t rc10_gim_out;
    std::uint16_t rc11_gim_out;
} sony_focus_settings_t;

typedef enum {
    modGimbal,
    modFocus,
    modShutter,
    modAperture,
    modIso,
    Number_of_cam_modes
} sony_cam_modes_e;

typedef enum {
    mode_idle,
    mode_timing,
    mode_timed_out
} timer_e;

typedef enum {
    SONY_SDK_FOCUS_FAR_FAST = 7,
    SONY_SDK_FOCUS_FAR_Meduim = 3,
    SONY_SDK_FOCUS_FAR_SLOW = 1,
    SONY_SDK_FOCUS_Near_Slow = -1,
    SONY_SDK_FOCUS_Near_Meduim = -3,
    SONY_SDK_FOCUS_Near_Fast = -7,
    Number_of_focus_settings = 6
} focus_set_e;

typedef enum {
    shut_idle,
    shut_go_hi,
    shut_go_lo
} shut_step_e;

// initialise the state engines and variables
// the values will be overwritten if requested to read the .ini file
//
void init_focus_mode(sony_focus_settings_t* fs) {
    fs->mode_step = mode_idle;
    fs->shut_step = shut_idle;
    fs->use_sdk_shut = 0;
    fs->mo_rc12fmu_mode_swap = 1500u;
    fs->mo_rc12fmu_tm1 = 500u;
    fs->hw_def_mode = modGimbal;
    fs->prev_def_mode = modGimbal;
    fs->gi_rc11fmu_gim_spt1 = 1500u;
    fs->gi_rc11fmu_gim_spt2 = 1500u;
    fs->gi_rc11fmu_gim_lim1 = 1500u;
    fs->fo_rc10fmu_spt1 = 1490u;
    fs->fo_rc10fmu_spt2 = 1510u;
    fs->fo_rc10fmu_spt3 = 1190u;
    fs->fo_rc10fmu_spt4 = 1210u;
    fs->fo_rc10fmu_spt5 = 1290u;
    fs->fo_rc10fmu_spt6 = 1310u;
    fs->fo_rc10fmu_spt7 = 1390u;
    fs->fo_rc10fmu_spt8 = 1510u;
    fs->fo_rc10fmu_spt9 = 1690u;
    fs->fo_rc10fmu_spt10 = 1710u;
    fs->fo_rc10fmu_spt11 = 1790u;
    fs->fo_rc10fmu_spt12 = 1800u;
    fs->fo_rc10fmu_tm1 = 50u;
    fs->sh_rc10fmu_spt1 = 1500u;
    fs->sh_rc10fmu_spt2 = 1400u;
    fs->sh_rc10fmu_spt3 = 1600u;
    fs->sh_rc10fmu_tm1 = 50u;
    fs->use_focus_setting = SONY_SDK_FOCUS_FAR_SLOW;
    fs->prev_focus_setting = SONY_SDK_FOCUS_FAR_SLOW;
}

//
// Iterative function to check and set the mode using a chan_signal
//
void check_change_mode(sony_focus_settings_t* fs, std::uint16_t chan_sig) {

    std::chrono::high_resolution_clock::time_point t_end;
    if ((chan_sig >= fs->mo_rc12fmu_mode_swap) && (fs->mode_step == mode_idle)) {
        fs->tim_start_1 = std::chrono::high_resolution_clock::now();
        fs->mode_step = mode_timing;
    }
    else if ((chan_sig < fs->mo_rc12fmu_mode_swap) && (fs->mode_step != mode_idle)) {
        fs->mode_step = mode_idle;
    }
    else if (fs->mode_step == mode_timing) {
        t_end = std::chrono::high_resolution_clock::now();
        auto tdiff = std::chrono::duration<double, std::milli>(t_end - fs->tim_start_1).count();
        if (tdiff > fs->mo_rc12fmu_tm1) {
            fs->hw_def_mode = ++fs->hw_def_mode % Number_of_cam_modes;
            fs->mode_step = mode_timed_out;
        }
    }
}

//
// Iterative function to set shutter and focus variables
//
void check_focus_adjust(sony_focus_settings_t* fs, std::uint16_t chan_sig11, std::uint16_t chan_sig10, std::uint16_t is, std::uint8_t sid) {

    switch (fs->hw_def_mode)
    {
        case modGimbal:
        {
            if (fs->prev_def_mode != fs->hw_def_mode) {
                std::string ssi = "Camera_Controller :: Change mode to Gimbal from " + fs->prev_def_mode;
                rfc5424_facility_e f = user_level;
                rfc5424_severity_e s = rfc5424_info;
                sendStatusTextMessage(ssi, f, s, is, sid);
            }
            if (chan_sig11 < fs->gi_rc11fmu_gim_spt1) {
                fs->rc10_gim_out = chan_sig10;
                fs->rc11_gim_out = 0u;
            }
            else if (chan_sig11 >= fs->gi_rc11fmu_gim_spt2) {
                fs->rc10_gim_out = fs->gi_rc11fmu_gim_lim1;
                fs->rc11_gim_out = chan_sig10;
            }
            fs->shut_step = shut_idle;
            fs->prev_def_mode = fs->hw_def_mode;
         }
         break;

         case modFocus:
         {
             if (fs->prev_def_mode != fs->hw_def_mode) {
                 std::string ssi = "Camera_Controller :: Change mode to Focus Mode from " + fs->prev_def_mode;
                 rfc5424_facility_e f = user_level;
                 rfc5424_severity_e s = rfc5424_info;
                 sendStatusTextMessage(ssi, f, s, is, sid);
             }
             if ((chan_sig10 > fs->fo_rc10fmu_spt1) && (chan_sig10 < fs->fo_rc10fmu_spt2)) {
                    /* --------------------- nop ---------------------------------- */
             }
             else if (chan_sig10 <= fs->fo_rc10fmu_spt3) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_FAR_FAST;
             }
             else if ((chan_sig10 > fs->fo_rc10fmu_spt4) && (chan_sig10 < fs->fo_rc10fmu_spt5)) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_FAR_Meduim;
             }
             else if ((chan_sig10 > fs->fo_rc10fmu_spt6) && (chan_sig10 < fs->fo_rc10fmu_spt7)) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_FAR_SLOW;
             }
             else if ((chan_sig10 > fs->fo_rc10fmu_spt8) && (chan_sig10 < fs->fo_rc10fmu_spt9)) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_Near_Slow;
             }
             else if ((chan_sig10 > fs->fo_rc10fmu_spt10) && (chan_sig10 < fs->fo_rc10fmu_spt11)) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_Near_Meduim;
             }
             else if (chan_sig10 > fs->fo_rc10fmu_spt12) {
                 fs->use_focus_setting = SONY_SDK_FOCUS_Near_Fast;
             }
             fs->shut_step = shut_idle;
             fs->prev_def_mode = fs->hw_def_mode;
         }
         break;

        case modShutter:
        {
            if (fs->prev_def_mode != fs->hw_def_mode) {
                std::string ssi = "Camera_Controller :: Change mode to Shutter from " + fs->prev_def_mode;
                rfc5424_facility_e f = user_level;
                rfc5424_severity_e s = rfc5424_info;
                sendStatusTextMessage(ssi, f, s, is, sid);
            }
            if (chan_sig10 <= fs->sh_rc10fmu_spt2) {
                if ((SH_LOW_LIM < fs->use_sdk_shut) && (fs->shut_step != shut_go_lo)) {
                   fs->use_sdk_shut = --fs->use_sdk_shut;
                   fs->shut_step = shut_go_lo;
               }
            }
            else if (chan_sig10 >= fs->sh_rc10fmu_spt3) {
                if ((SH_HI_LIM > fs->use_sdk_shut) && (fs->shut_step != shut_go_hi)) {
                   fs->use_sdk_shut = ++fs->use_sdk_shut;
                   fs->shut_step = shut_go_hi;
                }
            }
            fs->prev_def_mode = fs->hw_def_mode;
        }
        break;

        case modAperture:
        {
            if (fs->prev_def_mode != fs->hw_def_mode) {
                std::string ssi = "Camera_Controller :: Change mode to Aperture from " + fs->prev_def_mode;
                rfc5424_facility_e f = user_level;
                rfc5424_severity_e s = rfc5424_info;
                sendStatusTextMessage(ssi, f, s, is, sid);
            }
            fs->shut_step = shut_idle;
            fs->prev_def_mode = fs->hw_def_mode;
        }
        break;

        case modIso:
        {
            if (fs->prev_def_mode != fs->hw_def_mode) {
                std::string ssi = "Camera_Controller :: Change mode to Iso from " + fs->prev_def_mode;
                rfc5424_facility_e f = user_level;
                rfc5424_severity_e s = rfc5424_info;
                sendStatusTextMessage(ssi, f, s, is, sid);
            }
            fs->shut_step = shut_idle;
            fs->prev_def_mode = fs->hw_def_mode;
        }
        break;

        default:
        {
            fs->shut_step = shut_idle;
            fs->prev_def_mode = fs->hw_def_mode;
        }
        break;
   }

}

#endif

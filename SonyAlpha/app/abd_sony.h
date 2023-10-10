#ifndef __abd_sony_h
#define __abd_sony_h

    // ------------ These are the acp added functions -------------	
    void execute_movie_rec_int(int selected_index);
    void set_focus_position(int value);	

    // ------------ These are new from my code --------------------
    void set_aperture_args( std::string myInput );
    void set_iso_args( std::string myInput );
    void set_shutter_speed_args( std::string myInput );
    void set_exposure_program_mode_args( std::string myInput );
    void set_still_capture_mode_args( std::string myInput );
    void set_focus_mode_args( std::string myInput );
    void set_focus_area_args( std::string myInput );
    void set_white_balance_args( std::string myInput );
	
	// ----------- Speed Functions -------------------------------
    std::tuple<std::string, CrInt8, std::uint32_t> GetAperture();
    std::tuple<std::string, CrInt8, std::uint32_t> GetIso();
    std::tuple<std::string, CrInt8, std::uint32_t> GetShutterSpeed();
    std::tuple<std::string, CrInt8, std::uint32_t> GetExposureMode();
    std::tuple<std::string, CrInt8, std::uint32_t> GetStillCapMode();
    std::tuple<std::string, CrInt8, std::uint32_t> GetFocusMode();
    std::tuple<std::string, CrInt8, std::uint32_t> GetFocusArea();
    std::tuple<std::string, CrInt8, std::uint32_t> GetWhiteBalance();
    CrInt8 LoadMavProperties(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& p);
	
	// -------------- new functions ------------------------------
    std::int32_t ParseStringToInt(std::string& arg);
	
    CrInt8 SetApertureArgs(std::string myInput);
    CrInt8 SetWhiteBalanceArgs(std::string myInput);
    CrInt8 SetFocusAreaArgs(std::string myInput);
    CrInt8 SetFocusModeArgs(std::string myInput);
    CrInt8 SetStillCaptureModeArgs(std::string myInput);
    CrInt8 SetExposureProgramModeArgs(std::string myInput);
    CrInt8 SetShutterSpeedArgs(std::string myInput);
    CrInt8 SetIsoArgs(std::string myInput);
    CrInt8 SetApertureArgsInt(std::uint16_t selected_index);
    CrInt8 SetWhiteBalanceArgsInt(std::uint16_t selected_index);
    CrInt8 SetFocusAreaArgsInt(std::uint16_t selected_index);
    CrInt8 SetFocusModeArgsInt(std::uint16_t selected_index);
    CrInt8 SetStillCaptureModeArgsInt(std::uint32_t selected_index);
    CrInt8 SetExposureProgramModeArgsInt(std::uint16_t selected_index);
    CrInt8 SetShutterSpeedArgsInt(std::uint32_t selected_index);
    CrInt8 SetIsoArgsInt(std::uint32_t selected_index);
	
	// -------------- Ref Mohameds email -------------------------
    void SetNearFarEnable(SCRSDK::CrCommandParam v);
    void SetFocusMagnifierSetting(SCRSDK::CrCommandParam v);
	
    // ------------- new console print functions
    CrInt8 printCrMovie_Recording_State(SCRSDK::CrMovie_Recording_State retVal);
    SCRSDK::CrMovie_Recording_State get_rec_state_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    SCRSDK::CrWhiteBalanceSetting get_white_bal_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrWhiteBalanceSetting(SCRSDK::CrWhiteBalanceSetting retVal);
    SCRSDK::CrDriveMode get_still_cap_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrDriveMode(SCRSDK::CrDriveMode retVal);
    SCRSDK::CrFocusArea get_focus_area_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrFocusArea(SCRSDK::CrFocusArea retVal);
    SCRSDK::CrFocusMode get_focus_mode_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrFocusMode(SCRSDK::CrFocusMode retVal);
    SCRSDK::CrExposureProgram get_ex_pro_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrExposureProgram(SCRSDK::CrExposureProgram retVal);
    SCRSDK::CrShutterSpeedSet get_shut_spd_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    void printCrFnumberSet(SCRSDK::CrFnumberSet retVal);
    SCRSDK::CrFnumberSet get_aper_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    SCRSDK::CrISOMode get_iso_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    SCRSDK::CrNearFarEnableStatus get_near_far_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec);
    std::uint32_t get_tag_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec, std::string name);
    std::int32_t get_tag_value( std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup);
    CrInt8 get_tag_write_prot( std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup);
    std::int32_t get_tag_value_from_vector( std::string tag_to_fetch, std::vector<std::tuple<std::string, CrInt8, std::uint32_t>>& p );
    std::pair<CrInt8, std::int32_t> get_tag_data( std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup);
    std::pair<CrInt8, std::int32_t> get_tag_data_from_vector( std::string tag_to_fetch, std::vector<std::tuple<std::string, CrInt8, std::uint32_t>>& p );
#endif

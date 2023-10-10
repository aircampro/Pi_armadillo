// C Function ADD IN for extra library functions
//
    void CameraDevice::execute_movie_rec_int(int selected_index)
    {
        //load_properties();

        CrInt64u ptpValue = 0;
        switch (selected_index) {
        case 0:
            ptpValue = SDK::CrCommandParam::CrCommandParam_Up;
            break;
        case 1:
            ptpValue = SDK::CrCommandParam::CrCommandParam_Down;
            break;
        default:
            selected_index = -1;
            break;
        }

        SDK::SendCommand(m_device_handle, SDK::CrCommandId::CrCommandId_MovieRecord, (SDK::CrCommandParam)ptpValue);
    }

    void CameraDevice::set_focus_position(int value)
    {

        SDK::CrDeviceProperty prop_focus;
        prop_focus.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_NearFar);
        prop_focus.SetCurrentValue(value);
        prop_focus.SetValueType(SDK::CrDataType::CrDataType_Int8);
        SDK::SetDeviceProperty(m_device_handle, &prop_focus);

        std::this_thread::sleep_for(1000ms);
    }
    
    void CameraDevice::set_aperture_args(std::string myInput)
    {
        if (!m_prop.f_number.writable) {
            // Not a settable property
            tout << "Aperture is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Aperture value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Aperture value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.f_number.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_f_number(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Aperture value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }


    CrInt8 CameraDevice::SetApertureArgs(std::string myInput)
    {
        if (!m_prop.f_number.writable) {
            // Not a settable property
            tout << "Aperture is not writable\n";
            return -1;
        }

        auto& values = m_prop.f_number.possible;

        cli::tout << "values size " << values.size() << "\n";
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;
        if (values.size() == 19) {
            --selected_index;
        }

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetApertureArgsInt(std::uint16_t selected_index)
    {
        if (!m_prop.f_number.writable) {
            // Not a settable property
            tout << "Aperture is not writable\n";
            return -1;
        }

        auto& values = m_prop.f_number.possible;

        cli::tout << "values size " << values.size() << "\n";

        if (values.size() == 19) {
            --selected_index;
        }

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }
	
    void CameraDevice::set_iso_args(std::string myInput)
    {
        if (!m_prop.iso_sensitivity.writable) {
            // Not a settable property
            tout << "ISO is not writable\n";
            return;
        }


        auto& values = m_prop.iso_sensitivity.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] ISO " << format_iso_sensitivity(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new ISO value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetIsoArgs(std::string myInput)
    {
        if (!m_prop.iso_sensitivity.writable) {
            // Not a settable property
            tout << "ISO is not writable\n";
            return -1;
        }

        auto& values = m_prop.iso_sensitivity.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] ISO " << format_iso_sensitivity(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new ISO value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetIsoArgsInt(std::uint32_t selected_index)
    {
        if (!m_prop.iso_sensitivity.writable) {
            // Not a settable property
            tout << "ISO is not writable\n";
            return -1;
        }

        auto& values = m_prop.iso_sensitivity.possible;


        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    void CameraDevice::set_shutter_speed_args(std::string myInput)
    {
        if (!m_prop.shutter_speed.writable) {
            // Not a settable property
            tout << "Shutter Speed is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Shutter Speed value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Shutter Speed value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.shutter_speed.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_shutter_speed(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Shutter Speed value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        // ----------- noticed the list could change values size adjusted for that scenario ------
        if (values.size() == 55)
        {
            if (selected_index <= 0)
            {
                tout << "Input cancelled.\n";
            }
            else
            {
                selected_index--;
            }
        }
        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        tout << "values size is " << values.size() << ":\n";
        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetShutterSpeedArgs(std::string myInput)
    {
        if (!m_prop.shutter_speed.writable) {
            // Not a settable property
            tout << "Shutter Speed is not writable\n";
            return -1;
        }

        auto& values = m_prop.shutter_speed.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        // ----------- noticed the list could change values size adjusted for that scenario ------
        if (values.size() == 55)
        {
            if (selected_index <= 0)
            {
                tout << "Input cancelled.\n";
                return -2;
            }
            else
            {
                selected_index--;
            }
        }
        if (selected_index <= 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        tout << "values size is " << values.size() << ":\n";
        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetShutterSpeedArgsInt(std::uint32_t selected_index)
    {
        if (!m_prop.shutter_speed.writable) {
            // Not a settable property
            tout << "Shutter Speed is not writable\n";
            return -1;
        }

        auto& values = m_prop.shutter_speed.possible;

        // ----------- noticed the list could change values size adjusted for that scenario ------
        if (values.size() == 55)
        {
            if (selected_index <= 0)
            {
                tout << "Input cancelled.\n";
                return -2;
            }
            else
            {
                selected_index--;
            }
        }
        if (selected_index <= 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        tout << "values size is " << values.size() << ":\n";
        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    void CameraDevice::set_exposure_program_mode_args(std::string myInput)
    {
        if (!m_prop.exposure_program_mode.writable) {
            // Not a settable property
            tout << "Exposure Program Mode is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Exposure Program Mode value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Exposure Program Mode value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.exposure_program_mode.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_exposure_program_mode(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Exposure Program Mode value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetExposureProgramModeArgs(std::string myInput)
    {
        if (!m_prop.exposure_program_mode.writable) {
            // Not a settable property
            tout << "Exposure Program Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.exposure_program_mode.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetExposureProgramModeArgsInt(std::uint16_t selected_index)
    {
        if (!m_prop.exposure_program_mode.writable) {
            // Not a settable property
            tout << "Exposure Program Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.exposure_program_mode.possible;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }
	
    void CameraDevice::set_still_capture_mode_args(std::string myInput)
    {
        if (!m_prop.still_capture_mode.writable) {
            // Not a settable property
            tout << "Still Capture Mode is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Still Capture Mode value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Still Capture Mode value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.still_capture_mode.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_still_capture_mode(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Still Capture Mode value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetStillCaptureModeArgs(std::string myInput)
    {
        if (!m_prop.still_capture_mode.writable) {
            // Not a settable property
            tout << "Still Capture Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.still_capture_mode.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetStillCaptureModeArgsInt(std::uint32_t selected_index)
    {
        if (!m_prop.still_capture_mode.writable) {
            // Not a settable property
            tout << "Still Capture Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.still_capture_mode.possible;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt32Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    void CameraDevice::set_focus_mode_args(std::string myInput)
    {
        if (!m_prop.focus_mode.writable) {
            // Not a settable property
            tout << "Focus Mode is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Focus Mode value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Focus Mode value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.focus_mode.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_focus_mode(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Focus Mode value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        tout << "value size is " << values.size() << " \n";
        // seems to be at least 2 lists under different conditions - adjust here the selector
        if (values.size() == 4)
        {
            int new_index = 0;
            switch (selected_index)
            {
            case 3:
                new_index = 2;
                break;

            case 4:
                new_index = 3;
                break;

            case 2:
                new_index = -1;
                break;

            default:
                new_index = selected_index;
                break;
            }
            selected_index = new_index;
        }
        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }
	
    void CameraDevice::set_focus_area_args(std::string myInput)
    {
        if (!m_prop.focus_area.writable) {
            // Not a settable property
            tout << "Focus Area is not writable\n";
            return;
        }

        //text input;
        //tout << "Would you like to set a new Focus Area value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << "Choose a number set a new Focus Area value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.focus_area.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_focus_area(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << "Choose a number set a new Focus Area value:\n";

        //tout << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusArea);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        // m_cr_lib->SetDeviceProperty(m_device_handle, &prop);
        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetFocusModeArgs(std::string myInput)
    {
        if (!m_prop.focus_mode.writable) {
            // Not a settable property
            tout << "Focus Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.focus_mode.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        tout << "value size is " << values.size() << " \n";
        // seems to be at least 2 lists under different conditions - adjust here the selector
        if (values.size() == 4)
        {
            int new_index = 0;
            switch (selected_index)
            {
            case 3:
                new_index = 2;
                break;

            case 4:
                new_index = 3;
                break;

            case 2:
                new_index = -1;
                break;

            default:
                new_index = selected_index;
                break;
            }
            selected_index = new_index;
        }
        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetFocusModeArgsInt(std::uint16_t selected_index)
    {
        if (!m_prop.focus_mode.writable) {
            // Not a settable property
            tout << "Focus Mode is not writable\n";
            return -1;
        }

        auto& values = m_prop.focus_mode.possible;

        tout << "value size is " << values.size() << " \n";
        // seems to be at least 2 lists under different conditions - adjust here the selector
        if (values.size() == 4)
        {
            int new_index = 0;
            switch (selected_index)
            {
                case 3:
                new_index = 2;
                break;

                case 4:
                new_index = 3;
                break;

                case 2:
                new_index = -1;
                break;

                default:
                new_index = selected_index;
                break;
            }
            selected_index = new_index;
        }
        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetFocusAreaArgs(std::string myInput)
    {
        if (!m_prop.focus_area.writable) {
            // Not a settable property
            tout << "Focus Area is not writable\n";
            return -1;
        }

        auto& values = m_prop.focus_area.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusArea);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetFocusAreaArgsInt(std::uint16_t selected_index)
    {
        if (!m_prop.focus_area.writable) {
            // Not a settable property
            tout << "Focus Area is not writable\n";
            return -1;
        }

        auto& values = m_prop.focus_area.possible;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_FocusArea);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

   void CameraDevice::set_white_balance_args(std::string myInput)
    {
        if (!m_prop.white_balance.writable) {
            // Not a settable property
            tout << "White Balance is not writable\n";
            return;
        }

        //text input;
        //tout << std::endl << "Would you like to set a new White Balance value? (y/n): ";
        //std::getline(tin, input);
        //if (input != TEXT("y")) {
        //    tout << "Skip setting a new value.\n";
        //    return;
        //}

        //tout << std::endl << "Choose a number set a new White Balance value:\n";
        //tout << "[-1] Cancel input\n";

        auto& values = m_prop.white_balance.possible;
        //for (std::size_t i = 0; i < values.size(); ++i) {
        //    tout << '[' << i << "] " << format_white_balance(values[i]) << '\n';
        //}

        //tout << "[-1] Cancel input\n";
        //tout << std::endl << "Choose a number set a new White Balance value:\n";

        //tout << std::endl << "input> ";
        //std::getline(tin, input);
        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
    }

    CrInt8 CameraDevice::SetWhiteBalanceArgs(std::string myInput)
    {
        if (!m_prop.white_balance.writable) {
            // Not a settable property
            tout << "White Balance is not writable\n";
            return -1;
        }

        auto& values = m_prop.white_balance.possible;

        text_stringstream ss(myInput);
        int selected_index = 0;
        ss >> selected_index;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    CrInt8 CameraDevice::SetWhiteBalanceArgsInt(std::uint16_t selected_index)
    {
        if (!m_prop.white_balance.writable) {
            // Not a settable property
            tout << "White Balance is not writable\n";
            return -1;
        }

        auto& values = m_prop.white_balance.possible;

        if (selected_index < 0 || static_cast <int>(values.size()) <= selected_index) {
            tout << "Input cancelled.\n";
            return -2;
        }

        SDK::CrDeviceProperty prop;
        prop.SetCode(SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance);
        prop.SetCurrentValue(values[selected_index]);
        prop.SetValueType(SDK::CrDataType::CrDataType_UInt16Array);

        SDK::SetDeviceProperty(m_device_handle, &prop);
        return 1;
    }

    /*
       ========================================================================================================================

       new commands for returning the 2 values internally in the program code directly rather than stdout
       it returns a tuple of the relevant values to the property the usage of these functions are as per below

       examples of usage :-

       tuple<std::string, CrInt8, std::uint32_t> ap_data("notset", "notset");  // define the tuple
       ap_data = cameraInstance->GetAperture();            // invoke this on the camera object
       tout << "ApertureSet" << std::get<0>(ap_data) << "\n";   // this is the 1st string being printed
       tout << "ApertureWriteProtect" << std::get<1>(ap_data) << "\n";   // this is the 2nd string being printed
       tout << "ApertureVal" << std::get<2>(ap_data) << "\n";   // this is the 2nd string being printed
       tuple<std::string, std::string, CrInt8u, std::string, std::string> media_data = CameraDevice::GetSelectMediaFormat();
       CrInt8u x;
       std::string s;
       std::tie(std::ignore, std::ignore, x, s, std::ignore) = media_data;     // make the tie out of chosen data from the tuple
       tout << x << " " << s << "\n";                                          // print them as single values

       =========================================================================================================================

    */
    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetAperture()
    {
        // the load properties is called on item update so im not calling it here
        //
        //load_properties();
        //return std::make_tuple( format_f_number(m_prop.f_number.current), (std::uint32_t) m_prop.f_number.current );
        std::string s = format_f_number(m_prop.f_number.current);
        return std::make_tuple(s, m_prop.f_number.writable, (std::uint32_t)m_prop.f_number.current);
    }


    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetIso()
    {
        //load_properties();
        //return std::make_tuple( format_f_number(m_prop.f_number.current), (std::uint32_t) m_prop.f_number.current );
        std::uint32_t iso_mode = (m_prop.iso_sensitivity.current >> 28);
        std::string s;
        if (iso_mode == 0x0000) {
            // Normal mode
            s = "Normal";
        }
        else if (iso_mode == 0x0001) {
            // Multi Frame mode
            s = "Multi_Frame";
        }
        else if (iso_mode == 0x0002) {
            // Multi Frame High mode
            s = "Multi_Frame_High";
        }
        return std::make_tuple(s, m_prop.iso_sensitivity.writable, (std::uint32_t)m_prop.iso_sensitivity.current);
    }



    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetShutterSpeed()
    {
        //load_properties();
        //return std::make_tuple( format_f_number(m_prop.f_number.current), (std::uint32_t) m_prop.f_number.current );
        std::string s = format_shutter_speed(m_prop.shutter_speed.current);
        return std::make_tuple(s, m_prop.shutter_speed.writable, (std::uint32_t)m_prop.shutter_speed.current);
    }

    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetExposureMode()
    {
        //load_properties();
        std::string s = format_exposure_program_mode(m_prop.exposure_program_mode.current);
        return std::make_tuple(s, m_prop.exposure_program_mode.writable, (std::uint32_t)m_prop.exposure_program_mode.current);
    }
	
    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetStillCapMode()
    {
        //load_properties();
        std::string s = format_still_capture_mode(m_prop.still_capture_mode.current);
        return std::make_tuple(s, m_prop.still_capture_mode.writable, (std::uint32_t)m_prop.still_capture_mode.current);
    }

    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetFocusMode()
    {
        //load_properties();
        std::string s = format_focus_mode(m_prop.focus_mode.current);
        return std::make_tuple(s, m_prop.focus_mode.writable, (std::uint32_t)m_prop.focus_mode.current);
    }

    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetFocusArea()
    {
        //load_properties();
        std::string s = format_focus_area(m_prop.focus_area.current);
        return std::make_tuple(s, m_prop.focus_area.writable, (std::uint32_t)m_prop.focus_area.current);
    }
	
    std::tuple<std::string, CrInt8, std::uint32_t> CameraDevice::GetWhiteBalance()
    {
        //load_properties();
        std::string s = format_white_balance(m_prop.white_balance.current);
        return std::make_tuple(s, m_prop.white_balance.writable, (std::uint32_t)m_prop.white_balance.current);
    }
	
    CrInt8 CameraDevice::LoadMavProperties(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& p)
    {

        std::int32_t nprop = 0;
        SDK::CrDeviceProperty* prop_list = nullptr;

        SDK::CrError status = SDK::CrError_Generic;
        status = SDK::GetDeviceProperties(m_device_handle, &prop_list, &nprop);

        if (CR_FAILED(status)) {
            tout << "Failed to get device properties.\n";
            return -1;
        }

        // loop the properties list and fetch the parameters for mavlink and place them into a vector list of tuples
        //	
        if (prop_list && nprop > 0) {
            // Got properties list
            //
            for (std::int32_t i = 0; i < nprop; ++i) {
                auto prop = prop_list[i];
                int nval = 0;

                switch (prop.GetCode()) {

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_FNumber:
                    p.push_back(std::make_tuple("S_APERTURE", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_IsoSensitivity:
                    p.push_back(std::make_tuple("S_ISO", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_ShutterSpeed:
                    p.push_back(std::make_tuple("S_SHUTSPD", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_ExposureProgramMode:
                    p.push_back(std::make_tuple("S_EXPRO", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_FocusMode:
                    p.push_back(std::make_tuple("S_FOCUS_MODE", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_FocusArea:
                    p.push_back(std::make_tuple("S_FOCUS_AREA", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_WhiteBalance:
                    p.push_back(std::make_tuple("S_WHITE_BAL", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_DriveMode:
                    p.push_back(std::make_tuple("S_STILL_CAP", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_NearFar:
                    p.push_back(std::make_tuple("S_NEAR_FAR", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    case SDK::CrDevicePropertyCode::CrDeviceProperty_RecordingState:
                    p.push_back(std::make_tuple("REC_STATE", static_cast<std::uint8_t>(prop.IsSetEnableCurrentValue()), static_cast<std::uint32_t>(prop.GetCurrentValue())));
                    break;

                    default:
                    break;
                }
            }
            SDK::ReleaseDeviceProperties(m_device_handle, prop_list);
        }
        return 1;
    }
	
    /* ===============================================================================================================================
       These functions can be used to search and manipulate the data read from the camera using the above function
       =============================================================================================================================== */

    // parses string input to an int or null
    std::int32_t CameraDevice::ParseStringToInt(std::string& arg)
    {
        try
        {
            return { std::stoi(arg) };
        }
        catch (...)
        {
            cli::tout << "cannot convert \'" << arg << "\' to int!\n";
            return -1;
        }
    }

    std::int32_t CameraDevice::get_tag_value(std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup) {
        std::regex pattern(".*" + tag_to_fetch + ".*");
        if (std::regex_match(std::get<0>(tup), pattern)) {
            return static_cast<std::int32_t>(std::get<2>(tup));
        }
        return -1;
    }

    CrInt8 CameraDevice::get_tag_write_prot(std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup) {
        std::regex pattern(".*" + tag_to_fetch + ".*");
        if (std::regex_match(std::get<0>(tup), pattern)) {
            return std::get<1>(tup);
        }
        return -1;
    }

    std::pair<CrInt8, std::int32_t> CameraDevice::get_tag_data(std::string tag_to_fetch, std::tuple<std::string, CrInt8, std::uint32_t> tup) {
        std::regex pattern(".*" + tag_to_fetch + ".*");
        if (std::regex_match(std::get<0>(tup), pattern)) {
            return std::make_pair(std::get<1>(tup), std::get<2>(tup));
        }
        return std::make_pair(-1, -1);
    }

    std::int32_t CameraDevice::get_tag_value_from_vector(std::string tag_to_fetch, std::vector<std::tuple<std::string, CrInt8, std::uint32_t>>& p)
    {
        std::int32_t val = -1;
        for (auto x : p) {
            val = get_tag_value(tag_to_fetch, x);
        }
        return val;
    }

    std::pair<CrInt8, std::int32_t> CameraDevice::get_tag_data_from_vector(std::string tag_to_fetch, std::vector<std::tuple<std::string, CrInt8, std::uint32_t>>& p)
    {
        for (auto x : p) {
            std::pair<CrInt8, std::int32_t> pairV = get_tag_data(tag_to_fetch, x);
            if (-1 != pairV.first) {
                return pairV;
            }
        }
        return std::make_pair(-1, -1);
    }
	
    CrInt8 CameraDevice::printCrMovie_Recording_State(SCRSDK::CrMovie_Recording_State retVal)
    {
        CrInt8 retCode = 0;
        switch (retVal)
        {
            case SCRSDK::CrMovie_Recording_State::CrMovie_Recording_State_Not_Recording:
            tout << "enumeratedType returned was @= CrMovie_Recording_State_Not_Recording" << "\n";
            retCode = 0;
            break;

            case SCRSDK::CrMovie_Recording_State::CrMovie_Recording_State_Recording:
            tout << "enumeratedType returned was @= CrMovie_Recording_State_Recording" << "\n";
            retCode = 1;
            break;

            case SCRSDK::CrMovie_Recording_State::CrMovie_Recording_State_Recording_Failed:
            tout << "enumeratedType returned was @= CrMovie_Recording_State_Recording_Failed" << "\n";
            retCode = -1;
            break;

            default:
            tout << " unknown enum found " << std::endl;
            retCode = -2;
            break;
        }
        return retCode;
    }

    SCRSDK::CrMovie_Recording_State CameraDevice::get_rec_state_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "REC_STATE";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrMovie_Recording_State>(std::get<2>(tup));
            }
        }
        return SCRSDK::CrMovie_Recording_State::CrMovie_Recording_State_Recording_Failed;
    }

    SCRSDK::CrWhiteBalanceSetting CameraDevice::get_white_bal_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_WHITE_BAL";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrWhiteBalanceSetting>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrWhiteBalanceSetting>( - 1);
    }

    SCRSDK::CrNearFarEnableStatus CameraDevice::get_near_far_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_NEAR_FAR";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrNearFarEnableStatus>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrNearFarEnableStatus>(-1);
    }

    std::uint32_t CameraDevice::get_tag_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec, std::string name)
    {
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return (std::get<2>(tup));
            }
        }
        return -1;
    }

    void CameraDevice::printCrWhiteBalanceSetting(SCRSDK::CrWhiteBalanceSetting retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_AWB:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_AWB" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Underwater_Auto:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Underwater_Auto" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Daylight:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Daylight" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Shadow:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Shadow" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Cloudy:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Cloudy" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Tungsten:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Tungsten" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Fluorescent:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Fluorescent" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Fluorescent_WarmWhite:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Fluorescent_WarmWhite" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Fluorescent_CoolWhite:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Fluorescent_CoolWhite" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Fluorescent_DayWhite:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Fluorescent_DayWhite" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Fluorescent_Daylight:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Fluorescent_Daylight" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Flush:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Flush" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_ColorTemp:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_ColorTemp" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Custom_1:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Custom_1" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Custom_2:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Custom_2" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Custom_3:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Custom_3" << std::endl;
            break;

            case SCRSDK::CrWhiteBalanceSetting::CrWhiteBalance_Custom:
            cli::tout << "enumeratedType returned was @= CrWhiteBalance_Custom" << std::endl;
            break;

            default:
            cli::tout << " unknown enum found " << std::endl;
            break;
        }
    }
	
    SCRSDK::CrDriveMode CameraDevice::get_still_cap_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_STILL_CAP";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrDriveMode>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrDriveMode>(-1);
    }

    void CameraDevice::printCrDriveMode(SCRSDK::CrDriveMode retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrDriveMode::CrDrive_Single:
            cli::tout << "enumeratedType returned was @= CrDrive_Single" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Hi:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Hi" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Hi_Plus:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Hi_Plus" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Hi_Live:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Hi_Live" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Lo:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Lo" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_SpeedPriority:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_SpeedPriority" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Mid:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Mid" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Mid_Live:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Mid_Live" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Lo_Live:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Lo_Live" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_SingleBurstShooting_lo:
            cli::tout << "enumeratedType returned was @= CrDrive_SingleBurstShooting_lo" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_SingleBurstShooting_mid:
            cli::tout << "enumeratedType returned was @= CrDrive_SingleBurstShooting_mid" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_SingleBurstShooting_hi:
            cli::tout << "enumeratedType returned was @= CrDrive_SingleBurstShooting_hi" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Timelapse:
            cli::tout << "enumeratedType returned was @= CrDrive_Timelapse" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Timer_2s:
            cli::tout << "enumeratedType returned was @= CrDrive_Timer_2s" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Timer_5s:
            cli::tout << "enumeratedType returned was @= CrDrive_Timer_5s" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Timer_10s:
            cli::tout << "enumeratedType returned was @= CrDrive_Timer_10s" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_03Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_03Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_03Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_03Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_03Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_03Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_05Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_05Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_05Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_05Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_05Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_05Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_07Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_07Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_07Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_07Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_07Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_07Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_10Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_10Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_10Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_10Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_10Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_10Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_20Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_20Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_20Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_20Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_30Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_30Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Bracket_30Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Bracket_30Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_03Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_03Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_03Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_03Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_03Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_03Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_05Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_05Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_05Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_05Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_05Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_05Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_07Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_07Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_07Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_07Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_07Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_07Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_10Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_10Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_10Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_10Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_10Ev_9pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_10Ev_9pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_20Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_20Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_20Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_20Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_30Ev_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_30Ev_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Single_Bracket_30Ev_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Single_Bracket_30Ev_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_WB_Bracket_Lo:
            cli::tout << "enumeratedType returned was @= CrDrive_WB_Bracket_Lo" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_WB_Bracket_Hi:
            cli::tout << "enumeratedType returned was @= CrDrive_WB_Bracket_Hi" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_DRO_Bracket_Lo:
            cli::tout << "enumeratedType returned was @= CrDrive_DRO_Bracket_Lo" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_DRO_Bracket_Hi:
            cli::tout << "enumeratedType returned was @= CrDrive_DRO_Bracket_Hi" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_2s_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_2s_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_2s_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_2s_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_5s_3pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_5s_3pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_Continuous_Timer_5s_5pics:
            cli::tout << "enumeratedType returned was @= CrDrive_Continuous_Timer_5s_5pics" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_LPF_Bracket:
            cli::tout << "enumeratedType returned was @= CrDrive_LPF_Bracket" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_RemoteCommander:
            cli::tout << "enumeratedType returned was @= CrDrive_RemoteCommander" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_MirrorUp:
            cli::tout << "enumeratedType returned was @= CrDrive_MirrorUp" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_SelfPortrait_1:
            cli::tout << "enumeratedType returned was @= CrDrive_SelfPortrait_1" << std::endl;
            break;

            case SCRSDK::CrDriveMode::CrDrive_SelfPortrait_2:
            cli::tout << "enumeratedType returned was @= CrDrive_SelfPortrait_2" << std::endl;
            break;

            default:
            cli::tout << " unknown enum found " << std::endl;
            break;
        }
    }
    SCRSDK::CrFocusArea CameraDevice::get_focus_area_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_FOCUS_AREA";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrFocusArea>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrFocusArea>(-1);
    }

    void CameraDevice::printCrFocusArea(SCRSDK::CrFocusArea retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrFocusArea::CrFocusArea_Unknown:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Unknown" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Wide:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Wide" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Zone:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Zone" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Center:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Center" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Flexible_Spot_S:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Flexible_Spot_S" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Flexible_Spot_M:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Flexible_Spot_M" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Flexible_Spot_L:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Flexible_Spot_L" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Expand_Flexible_Spot:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Expand_Flexible_Spot" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Flexible_Spot:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Flexible_Spot" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Wide:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Wide" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Zone:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Zone" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Center:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Center" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Flexible_Spot_S:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Flexible_Spot_S" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Flexible_Spot_M:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Flexible_Spot_M" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Flexible_Spot_L:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Flexible_Spot_L" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Expand_Flexible_Spot:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Expand_Flexible_Spot" << std::endl;
            break;

            case SCRSDK::CrFocusArea::CrFocusArea_Tracking_Flexible_Spot:
            cli::tout << "enumeratedType returned was @= CrFocusArea_Tracking_Flexible_Spot" << std::endl;
            break;

        default:
            cli::tout << " unknown enum found " << std::endl;
            break;
        }
    }
    SCRSDK::CrFocusMode CameraDevice::get_focus_mode_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_FOCUS_MODE";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrFocusMode>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrFocusMode>(-1);
    }

    void CameraDevice::printCrFocusMode(SCRSDK::CrFocusMode retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrFocusMode::CrFocus_MF:
            cli::tout << "enumeratedType returned was @= CrFocus_MF" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_AF_S:
            cli::tout << "enumeratedType returned was @= CrFocus_AF_S" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_AF_C:
            cli::tout << "enumeratedType returned was @= CrFocus_AF_C" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_AF_A:
            cli::tout << "enumeratedType returned was @= CrFocus_AF_A" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_AF_D:
            cli::tout << "enumeratedType returned was @= CrFocus_AF_D" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_DMF:
            cli::tout << "enumeratedType returned was @= CrFocus_DMF" << std::endl;
            break;

            case SCRSDK::CrFocusMode::CrFocus_PF:
            cli::tout << "enumeratedType returned was @= CrFocus_PF" << std::endl;
            break;

            default:
            cli::tout << " unknown enum found " << std::endl;
            break;
        }
    }

    SCRSDK::CrExposureProgram CameraDevice::get_ex_pro_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_EXPRO";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrExposureProgram>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrExposureProgram>(-1);
    }

    void CameraDevice::printCrExposureProgram(SCRSDK::CrExposureProgram retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrExposureProgram::CrExposure_M_Manual:
            cli::tout << "enumeratedType returned was @= CrExposure_M_Manual" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_P_Auto:
            cli::tout << "enumeratedType returned was @= CrExposure_P_Auto" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_A_AperturePriority:
            cli::tout << "enumeratedType returned was @= CrExposure_A_AperturePriority" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_S_ShutterSpeedPriority:
            cli::tout << "enumeratedType returned was @= CrExposure_S_ShutterSpeedPriority" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Program_Creative:
            cli::tout << "enumeratedType returned was @= CrExposure_Program_Creative" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Program_Action:
            cli::tout << "enumeratedType returned was @= CrExposure_Program_Action" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Portrait:
            cli::tout << "enumeratedType returned was @= CrExposure_Portrait" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Auto:
            cli::tout << "enumeratedType returned was @= CrExposure_Auto" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Auto_Plus:
            cli::tout << "enumeratedType returned was @= CrExposure_Auto_Plus" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_P_A:
            cli::tout << "enumeratedType returned was @= CrExposure_P_A" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_P_S:
            cli::tout << "enumeratedType returned was @= CrExposure_P_S" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Sports_Action:
            cli::tout << "enumeratedType returned was @= CrExposure_Sports_Action" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Sunset:
            cli::tout << "enumeratedType returned was @= CrExposure_Sunset" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Night:
            cli::tout << "enumeratedType returned was @= CrExposure_Night" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Landscape:
            cli::tout << "enumeratedType returned was @= CrExposure_Landscape" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Macro:
            cli::tout << "enumeratedType returned was @= CrExposure_Macro" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HandheldTwilight:
            cli::tout << "enumeratedType returned was @= CrExposure_HandheldTwilight" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_NightPortrait:
            cli::tout << "enumeratedType returned was @= CrExposure_NightPortrait" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_AntiMotionBlur:
            cli::tout << "enumeratedType returned was @= CrExposure_AntiMotionBlur" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Pet:
            cli::tout << "enumeratedType returned was @= CrExposure_Pet" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Gourmet:
            cli::tout << "enumeratedType returned was @= CrExposure_Gourmet" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Fireworks:
            cli::tout << "enumeratedType returned was @= CrExposure_Fireworks" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HighSensitivity:
            cli::tout << "enumeratedType returned was @= CrExposure_HighSensitivity" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_MemoryRecall:
            cli::tout << "enumeratedType returned was @= CrExposure_MemoryRecall" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_ContinuousPriority_AE_8pics:
            cli::tout << "enumeratedType returned was @= CrExposure_ContinuousPriority_AE_8pics" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_ContinuousPriority_AE_10pics:
            cli::tout << "enumeratedType returned was @= CrExposure_ContinuousPriority_AE_10pics" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_ContinuousPriority_AE_12pics:
            cli::tout << "enumeratedType returned was @= CrExposure_ContinuousPriority_AE_12pics" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_3D_SweepPanorama:
            cli::tout << "enumeratedType returned was @= CrExposure_3D_SweepPanorama" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_SweepPanorama:
            cli::tout << "enumeratedType returned was @= CrExposure_SweepPanorama" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_P:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_P" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_A:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_A" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_S:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_S" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_M:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_M" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_Auto:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_Auto" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_SQMotion_P:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_SQMotion_P" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_SQMotion_A:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_SQMotion_A" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_SQMotion_S:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_SQMotion_S" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Movie_SQMotion_M:
            cli::tout << "enumeratedType returned was @= CrExposure_Movie_SQMotion_M" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_Flash_Off:
            cli::tout << "enumeratedType returned was @= CrExposure_Flash_Off" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_PictureEffect:
            cli::tout << "enumeratedType returned was @= CrExposure_PictureEffect" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HiFrameRate_P:
            cli::tout << "enumeratedType returned was @= CrExposure_HiFrameRate_P" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HiFrameRate_A:
            cli::tout << "enumeratedType returned was @= CrExposure_HiFrameRate_A" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HiFrameRate_S:
            cli::tout << "enumeratedType returned was @= CrExposure_HiFrameRate_S" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_HiFrameRate_M:
            cli::tout << "enumeratedType returned was @= CrExposure_HiFrameRate_M" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_SQMotion_P:
            cli::tout << "enumeratedType returned was @= CrExposure_SQMotion_P" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_SQMotion_A:
            cli::tout << "enumeratedType returned was @= CrExposure_SQMotion_A" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_SQMotion_S:
            cli::tout << "enumeratedType returned was @= CrExposure_SQMotion_S" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_SQMotion_M:
            cli::tout << "enumeratedType returned was @= CrExposure_SQMotion_M" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_MOVIE:
            cli::tout << "enumeratedType returned was @= CrExposure_MOVIE" << std::endl;
            break;

            case SCRSDK::CrExposureProgram::CrExposure_STILL:
            cli::tout << "enumeratedType returned was @= CrExposure_STILL" << std::endl;
            break;

            default:
            cli::tout << " unknown enum found " << std::endl;
            break;
        }
    }

    SCRSDK::CrShutterSpeedSet CameraDevice::get_shut_spd_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_SHUTSPD";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrShutterSpeedSet>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrShutterSpeedSet>(-1);
    }

    SCRSDK::CrISOMode CameraDevice::get_iso_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_ISO";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrISOMode>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrISOMode>(-1);
    }
    
    SCRSDK::CrFnumberSet CameraDevice::get_aper_from_cam_vector(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
    {
        std::string name = "S_APERTURE";
        for (auto tup : vec) {
            std::regex pattern(".*" + name + ".*");
            if (std::regex_match(std::get<0>(tup), pattern)) {
                return static_cast<SCRSDK::CrFnumberSet>(std::get<2>(tup));
            }
        }
        return static_cast<SCRSDK::CrFnumberSet>(-1);
    }

    void CameraDevice::printCrFnumberSet(SCRSDK::CrFnumberSet retVal)
    {
        switch (retVal)
        {
            case SCRSDK::CrFnumberSet::CrFnumber_Unknown:
            cli::tout << "enumeratedType returned was @= CrFnumber_Unknown" << std::endl;
            break;

            case SCRSDK::CrFnumberSet::CrFnumber_Nothing:
            cli::tout << "enumeratedType returned was @= CrFnumber_Nothing" << std::endl;
            break;

            default:
            cli::tout << " enum found Aperture FnumberSet F" << static_cast<float>(retVal)/10.0f << std::endl;
            break;
        }
    }

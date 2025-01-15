// read the set-up for the devices from the .ini file
//
// ref :- https://github.com/Rookfighter/inifile-cpp
//
#ifndef RINI_H_
#define RINI_H_

#include <inicpp.h>
#include <string>
#include <iostream>
#include "DdCamera.hpp"
	
class ReadIniConfig
{
public:

    // variables relating to the devices set-up
    int DD_ILI_9341_SPI_GPIO_DC=26;
    int DD_TP_TSC_2046_SPI_GPIO_IRQ=19;
    int CAP_FMT=DdCamera::CAPTURE_FORMAT_RGB888;
    std::string DD_ILI_9341_SPI_SPI_DEV="/dev/spidev0.0";   
    std::string DD_TP_TSC_2046_SPI_SPI_DEV="/dev/spidev0.1";
	std::string DD_CAMERA_V2L4="/dev/video0";
    ini::IniFile myIni;

    // load the ini file specifed as the argument
    int Load(std::string& path)
    {
        // open the ini file
        try {		
            myIni.load(path);
        }
        catch(char *pstr)
        {
			std::cout << "error loading the .ini file " << pstr << std::endl;
			return 0;
        }
		
        int temp1 = myIni["ILI_9341"]["DD_ILI_9341_SPI_GPIO_DC"].as<int>();
        if (0 < temp1 && temp1 <= 40)
        {
            DD_ILI_9341_SPI_GPIO_DC = temp1;
        }

        temp1 = myIni["TP_TSC_2046"]["DD_TP_TSC_2046_SPI_GPIO_IRQ"].as<int>();
        if (0 < temp1 && temp1 <= 40)
        {
            DD_TP_TSC_2046_SPI_GPIO_IRQ = temp1;
        }

        temp1 = myIni["CSI_CAM"]["CAP_FMT"].as<int>();
        if (0 < temp1 && temp1 <= 6)
        {
            CAP_FMT = temp1;
        }
		
        std::string temp3 = myIni["ILI_9341"]["DD_ILI_9341_SPI_SPI_DEV"].as<std::string>();
        if (!temp3.empty())
        {
            DD_ILI_9341_SPI_SPI_DEV = temp3;
        }
		
        temp3 = myIni["TP_TSC_2046"]["DD_TP_TSC_2046_SPI_SPI_DEV"].as<std::string>();
        if (!temp3.empty())
        {
            DD_TP_TSC_2046_SPI_SPI_DEV = temp3;
        }

        temp3 = myIni["CSI_CAM"]["DD_CAMERA_V2L4"].as<std::string>();
        if (!temp3.empty())
        {
            DD_CAMERA_V2L4 = temp3;
        }
		
		return 1;
    }

    // save the ini file
    void Save(std::string& path)
    {
        myIni["ILI_9341"]["DD_ILI_9341_SPI_GPIO_DC"] = DD_ILI_9341_SPI_GPIO_DC;
        myIni["TP_TSC_2046"]["DD_TP_TSC_2046_SPI_GPIO_IRQ"] = DD_TP_TSC_2046_SPI_GPIO_IRQ;
        myIni["ILI_9341"]["DD_ILI_9341_SPI_SPI_DEV"] = DD_ILI_9341_SPI_SPI_DEV;
        myIni["TP_TSC_2046"]["DD_TP_TSC_2046_SPI_SPI_DEV"] = DD_TP_TSC_2046_SPI_SPI_DEV;
        myIni["CSI_CAM"]["DD_CAMERA_V2L4"] = DD_CAMERA_V2L4;
        myIni["CSI_CAM"]["CAP_FMT"] = CAP_FMT;
        myIni.save(path);
    }
};

#endif /* end RINI */
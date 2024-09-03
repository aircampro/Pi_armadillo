// ----------------------------------------------------------------
//
// Read ini file and control pHoxI 3d scanner 
//
// ref :-
// https://github.com/photoneo/phoxi_camera/tree/master
// Photoneo PhoXi 3D Scanner
// https://www.photoneo.com/3d-scanning-software/ 
//
// ---------------------------------------------------------------
#include "phoxi_camera/PhoXiInterface.h"
#include "phoxi_camera/PhoXiException.h"
#include "PhoXi.h"

#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>

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

/* dimensions of max/min for ini values */
#define NUM_FRAMES 1000
#define TRANSX_LO_VAL_CLAMP -10
#define TRANSX_HI_VAL_CLAMP 10
#define TRANSY_LO_VAL_CLAMP -10
#define TRANSY_HI_VAL_CLAMP 10
#define TRANSZ_LO_VAL_CLAMP -10
#define TRANSZ_HI_VAL_CLAMP 10
#define ROT_LO_VAL_CLAMP -10
#define ROT_HI_VAL_CLAMP 10

#include "ini_file_reader.cpp"
#define PHOXI_XML_FILE "/home/pi/cam_init/phoxi_mavlink_xml.xml"
#include "xml_reader.cpp"
//#include "json_file_reader.cpp"

// types read from the file
enum CameraDefaultSource_e : int
{
    No_Settings = 0,
    Settings_From_Ini = 1,
    Settings_From_Xml = 2,
    Settings_From_Json = 3,
    Settings_From_Last = 4,
    Get_starting_Settings = 5,
};
enum CameraConnectionModes_e : int
{
    Software = 0,
    Hardware = 1,
    Freerun = 2,
    NoValue = 3,
    Number_of_modes = 4,
};
enum CameraTransCoordSpace_e : int
{
    Custom = 0,
    Robot = 1,
    Marker = 2,
    Camera = 3,
    Number_of_TCS = 4,
};

using namespace std;
phoxi_camera::PhoXiInterface phoxi_interface;
long sleepPhoXiFactory = 1500000;
const string camera_ID = "InstalledExamples-basic-example";

int main() {
    usleep(sleepPhoXiFactory);

    int settings_def = 0;
    int mode = 0;
    bool auto_start = true;
    int resolut=0;
    int coord_space = 0;
    pho::api::PhoXiCoordinateTransformation transformation;
	
    // we are using a .ini file for our settings
    // set up settings_def to reflect this
	//
    int settings_def = CameraDefaultSource_e::Settings_From_Ini;
    if (settings_def == CameraDefaultSource_e::Settings_From_Ini) {
        boost_fs::path const ini_file("/home/pi/cam_init/pHoxI.ini");
        if (boost_fs::is_regular_file(ini_file)) {
            std::cout << "the ini file exists" << "\n";
            settings_def = Get_starting_Settings;
        } else {
            std::cout << "the ini file is not existing" << "\n";
            settings_def = No_Settings;
        }
    }

    // if we set settings_def to xml mode then check for a mavlink xml file existing
    if (settings_def == CameraDefaultSource_e::Settings_From_Xml) {
        if (boost_fs::is_regular_file(PHOXI_XML_FILE)) {
            std::cout << "the xml file exists" << "\n";
        }
        else {
            std::cout << "the mavlink xml file is not existing" << "\n";
            settings_def = No_Settings;
        }
    }
	// read all the xml settings if we using an xml file 
    if (settings_def == CameraDefaultSource_e::Settings_From_Xml) {	
        std::string xml_file(PHOXI_XML_FILE);
        Parent xml_param;
        xml_load(xml_file, xml_param);
        for (std::vector<Child>::const_iterator cit = xml_param.children.begin(); cit != xml_param.children.end(); cit++) {
            if ((levenstein("mode", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "mode") == true)) {
                mode = clamp(cit->xml_val, 0, CameraConnectionModes_e::Number_of_modes-1);
                std::cout << "found " << cit->option_name << " similar to mode with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("auto_start", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "auto_start") == true)) {
                int as = clamp(cit->xml_val, 0, 1);
                if (as == 0) {
                    auto_start = false;	
                }
                std::cout << "found " << cit->option_name << " similar to auto_start with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("resolut", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "resolut") == true)) {
                resolut = clamp(cit->xml_val, 0, 1);
                std::cout << "found " << cit->option_name << " similar to resolut with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("transX", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "transX") == true)) {
                transformation.Translation.x = clamp(cit->xml_val, TRANSX_LO_VAL_CLAMP, TRANSX_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to transX with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("transY", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "transY") == true)) {
                transformation.Translation.y = clamp(cit->xml_val, TRANSY_LO_VAL_CLAMP, TRANSY_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to transY with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((levenstein("transZ", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "transZ") == true)) {
                transformation.Translation.z = clamp(cit->xml_val, TRANSZ_LO_VAL_CLAMP, TRANSZ_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to transZ with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot00") == true)) {
                transformation.Rotation[0][0] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot00 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot01") == true)) {
                transformation.Rotation[0][1] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot01 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot02") == true)) {
                transformation.Rotation[0][2] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot02 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot10") == true)) {
                transformation.Rotation[1][0] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot10 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot11") == true)) {
                transformation.Rotation[1][1] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot11 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot12") == true)) {
                transformation.Rotation[1][2] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot12 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot20") == true)) {
                transformation.Rotation[2][0] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot20 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot21") == true)) {
                transformation.Rotation[2][1] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot21 with value : " << cit->xml_val << " using as a default " << std::endl;
            }
            else if ((algo::contains(cit->option_name, "rot22") == true)) {
                transformation.Rotation[2][2] = clamp(cit->xml_val, ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
                std::cout << "found " << cit->option_name << " similar to rot22 with value : " << cit->xml_val << " using as a default " << std::endl;
            }		
            else if ((levenstein("trans_coord_space", cit->option_name) <= LEVENSTEIN_THRESH) || (algo::contains(cit->option_name, "trans_coord_space") == true)) {
                coord_space = clamp(cit->xml_val, 0, CameraTransCoordSpace_e::Number_of_TCS-1);
                std::cout << "found " << cit->option_name << " similar to trans_coord_space with value : " << cit->xml_val << " using as a default " << std::endl;
            }			
        }
    }
	
    if (settings_def == CameraDefaultSource_e::Get_starting_Settings) {
        mode = clamp(get_ini_int_component("pHoxI", "mode"), 0, CameraConnectionModes_e::Number_of_modes-1);		
        int as = clamp(get_ini_int_component("pHoxI", "auto_start"), 0, 1);	
        if (as == 0) {
            auto_start = false;	
        }			
    }
	
	// ----------------- connect and if auto_start==1 -> start the camera aquisition ------------------------------ 
	switch(mode) {
		case CameraConnectionModes_e::Software:
		{
	           // connect to the camera in software mode and start aquisition
	           phoxi_interface.connectCamera(camera_ID, pho::api::PhoXiTriggerMode::Software, auto_start);
	           // check we are conencted
                   if (phoxi_interface.isConnected()) {
                      std::cout << "Interface to camera is connected" << std::endl;
                   } else {
                      std::cout << "Interface to camera is not connected" << std::endl;   
                      exit(-1);	
                    }
                    if (phoxi_interface.isAcquiring()) {
                       std::cout << "Phoxi camera is aquiring" << std::endl;
                    } else {		
                       std::cout << "Phoxi camera failed to start aquisition" << std::endl;
                    }
	        }
		break;
		
		case CameraConnectionModes_e::Freerun:
		{
	          // connect to the camera in freerun mode and start aquisition
	          phoxi_interface.connectCamera(camera_ID, pho::api::PhoXiTriggerMode::Freerun, auto_start);
	          // check we are conencted
                  if (phoxi_interface.isConnected()) {
                      std::cout << "Interface to camera is connected" << std::endl;
                  } else {
                     std::cout << "Interface to camera is not connected" << std::endl;   
                     exit(-1);	
                  }
                  if (phoxi_interface.isAcquiring()) {
                    std::cout << "Phoxi camera is aquiring" << std::endl;
                  } else {		
                    std::cout << "Phoxi camera failed to start aquisition" << std::endl;
                  }
	        }
		break;
		
		case CameraConnectionModes_e::Hardware:
		{
	         // connect to the camera in hardware mode and start aquisition
	         phoxi_interface.connectCamera(camera_ID, pho::api::PhoXiTriggerMode::Hardware, auto_start);
	        // check we are conencted
                 if (phoxi_interface.isConnected()) {
                   std::cout << "Interface to camera is connected" << std::endl;
                 } else {
                   std::cout << "Interface to camera is not connected" << std::endl;   
                   exit(-1);	
                 }
                 if (phoxi_interface.isAcquiring()) {
                    std::cout << "Phoxi camera is aquiring" << std::endl;
                 } else {		
                    std::cout << "Phoxi camera failed to start aquisition" << std::endl;
                 }
	        }
		break;
		
		case CameraConnectionModes_e::NoValue:
		{
	         // connect to the camera in no value mode and start aquisition
	         phoxi_interface.connectCamera(camera_ID, pho::api::PhoXiTriggerMode::NoValue, auto_start);
	         // check we are conencted
                 if (phoxi_interface.isConnected()) {
                  std::cout << "Interface to camera is connected" << std::endl;
                 } else {
                   std::cout << "Interface to camera is not connected" << std::endl;   
                   exit(-1);	
                 }
                 if (phoxi_interface.isAcquiring()) {
                   std::cout << "Phoxi camera is aquiring" << std::endl;
                 } else {		
                   std::cout << "Phoxi camera failed to start aquisition" << std::endl;
                 }
	        }
		break;
		
        }	
	// ----------------- set resolution of the camera --------------------------------- 	
        if (settings_def == CameraDefaultSource_e::Get_starting_Settings) {
           resolut = clamp(get_ini_int_component("pHoxI", "resolut"), 0, 1);					
        }
	if (resolut == 0) {
	     phoxi_interface.setHighResolution();
	     pho::api::PFrame f = phoxi_interface.getPFrame(-1);
	     if ((GetResolution().Width == 2064) && (f->GetResolution().Height == 1544)) {
               std::cout << "resolution high : 2064x1544" << std::endl;	
             }				
	} else {
              phoxi_interface.setLowResolution();
              pho::api::PFrame f = phoxi_interface.getPFrame(-1);
              if ((GetResolution().Width == 1032) && (f->GetResolution().Height == 772)) {
                   std::cout << "resolution low : 1032x772" << std::endl;	
              }		
        }

    // ----------------- control aquisition ----------------------------------------------
    // phoxi_interface.startAcquisition();
    // phoxi_interface.stopAcquisition();
	
	// ----------------- set transformation of the camera --------------------------------- 
	// TODO :: check this type is float in the pHoxI library structure
	//
    if (settings_def == CameraDefaultSource_e::Get_starting_Settings) {
        coord_space = clamp(get_ini_int_component("pHoxI", "trans_coord_space"), 0, CameraTransCoordSpace_e::Number_of_TCS-1);	
        transformation.Translation.x = clamp(get_ini_float_component("pHoxI_trans_mat", "transX"), TRANSX_LO_VAL_CLAMP, TRANSX_HI_VAL_CLAMP);		
        transformation.Translation.y = clamp(get_ini_float_component("pHoxI_trans_mat", "transY"), TRANSY_LO_VAL_CLAMP, TRANSY_HI_VAL_CLAMP);	
        transformation.Translation.z = clamp(get_ini_float_component("pHoxI_trans_mat", "transZ"), TRANSZ_LO_VAL_CLAMP, TRANSZ_HI_VAL_CLAMP);	
        transformation.Rotation[0][0] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot00"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);		
        transformation.Rotation[0][1] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot01"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);	
        transformation.Rotation[0][2] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot02"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);	
        transformation.Rotation[1][0] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot10"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);		
        transformation.Rotation[1][1] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot11"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);	
        transformation.Rotation[1][2] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot12"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
        transformation.Rotation[2][0] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot20"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);		
        transformation.Rotation[2][1] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot21"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);	
        transformation.Rotation[2][2] = clamp(get_ini_float_component("pHoxI_trans_mat", "rot22"), ROT_LO_VAL_CLAMP, ROT_HI_VAL_CLAMP);
    }
    switch(coord_space) {
		case CameraTransCoordSpace_e::Custom:
		{
	        // CustomSpace
	        phoxi_interface.setTransformation(transformation, pho::api::PhoXiCoordinateSpace::CustomSpace, true, false);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Robot:
		{
	        // RobotSpace
	        phoxi_interface.setTransformation(transformation, pho::api::PhoXiCoordinateSpace::RobotSpace, true, false);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Marker:
		{
	        // MarkerSpace
	        phoxi_interface.setTransformation(transformation, pho::api::PhoXiCoordinateSpace::MarkerSpace, true, false);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Camera:
		{
	        // CameraSpace
	        phoxi_interface.setTransformation(transformation, pho::api::PhoXiCoordinateSpace::Camera, true, false);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::NoValue:
		{
	        // CameraSpace
	        phoxi_interface.setTransformation(transformation, pho::api::PhoXiCoordinateSpace::NoValue, true, false);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
        }	
	// ----------------- set co-ordinate space of the camera --------------------------------- 
	switch(coord_space) {
		case CameraTransCoordSpace_e::Camera:
		{
	        // Camera Space
	        phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::CameraSpace);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Custom:
		{
	        // Custom Space
	        phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::CustomSpace);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Marker:
		{
	        // MarkerSpace
	        phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::MarkerSpace);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::Robot:
		{
	        // RobotSpace
	        phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::RobotSpace);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
		case CameraTransCoordSpace_e::NoValue:
		{
	        // NoValueSpace
	        phoxi_interface.setCoordinateSpace(pho::api::PhoXiCoordinateSpace::NoValue);
	        // trigger image
                int frameID = phoxi_interface.triggerImage(true);
		pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
	        }
		break;
		
        }	
	
	// get frame and point cloud for NUM_FRAMES
        int organised = true;
	for (int y=0; y < NUM_FRAMES; y++) {
            int frameID = phoxi_interface.triggerImage(true);
            pho::api::PFrame f = phoxi_interface.getPFrame(frameID);
            std::shared_ptr<pcl::PointCloud<pcl::PointNormal>>  pcl_ptr = phoxi_interface.getPointCloudFromFrame(f, organised);
            usleep(sleepPhoXiFactory);
        }
	
	// disconnect the camera
	phoxi_interface.disconnectCamera();
	
}

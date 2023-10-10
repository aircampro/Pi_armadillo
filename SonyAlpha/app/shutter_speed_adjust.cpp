#ifndef __ss_adjustments_
#define __ss_adjustments_

#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include "focus_adjust.h"

const std::string XML_PATH2 = "test.xml";

#if defined(__focus_adjustment)

std::vector<std::uint32_t> get_shutter_speeds_vector(std::string xml_filename)
{

    std::vector<std::uint32_t> vec;
    // Traverse property tree example
    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(xml_filename, pt);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, pt.get_child("mavlinkcamera.parameters")) {
        // v.first is the name of the child.
        // v.second is the child tree.

        std::string id_string;
        boost::optional<std::uint32_t> def;
        std::uint32_t val;

        const boost::property_tree::ptree& pt_child = v.second;
        //
        // this extracts the name fields name and default from the xml as below
        // <parameter name="CAM_SHUTTERSPD" type="uint32" default="400">
        // then pull out the values into the vector for
        // <option name="30" value="19660810"/>
        //
        if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
            std::cout << "name found " << nam << std::endl;
            std::stringstream ss;
            ss << nam;
            ss >> id_string;
            boost::regex r1("CAM_SHUTTERSPD");
            boost::smatch m;

            if (boost::regex_search(id_string, m, r1)) {

                for (auto it : pt_child.get_child("options")) {
                    if (auto valu = it.second.get_optional<std::uint32_t>("<xmlattr>.value")) {
                        //std::cout << "Shutter Speeds are : " << valu << '\n';
                        std::stringstream ss1;
                        ss1 << valu;
                        ss1 >> val;
                        vec.push_back(val);
                    }
                }
                // just a loop for printing it for sanity check.. (remove)
                for (auto x : vec) {
                    std::cout << " the shutter speed value = " << x << std::endl;
                }
                break;
            }
        }
    }
    return vec;
}

std::uint32_t get_shutter_from_index(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs) {
    sfs->use_sdk_shut = sfs->use_sdk_shut % sss.size();
    sfs->prev_sdk_shut = sfs->use_sdk_shut;
    return sss[sfs->use_sdk_shut];
}

void get_index_from_shutter(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs, std::uint32_t request_val) {
    for (auto index : sss) {
        if (index == request_val) {
            sfs->use_sdk_shut = index;
            break;
        }
    }
}

// gets the near far from the camera last read state
//
std::uint32_t get_nf_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
{
    for (auto tup : vec) {
        boost::regex r1("S_NEAR_FAR");
        boost::smatch m;

        if (boost::regex_search(std::get<0>(tup), m, r1))
        {
            return std::get<2>(tup);
        }
    }
    return -100;
}

// gets the video recording state from the camera last read state
//
SCRSDK::CrMovie_Recording_State  get_rec_state_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
{
    for (auto tup : vec) {
        boost::regex r1("REC_STATE");
        boost::smatch m;

        if (boost::regex_search(std::get<0>(tup), m, r1))
        {
            return static_cast<SCRSDK::CrMovie_Recording_State>(std::get<2>(tup));
        }
    }
    return SCRSDK::CrMovie_Recording_State::CrMovie_Recording_State_Recording_Failed;
}

SCRSDK::CrMovie_Recording_State get_rec_state_from_cam_vec2(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
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

// gets the shutter speed from the camera last read state
//
int get_shutspd_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
{
    for (auto tup : vec) {
        boost::regex r1("S_SHUTSPD");
        boost::smatch m;

        if (boost::regex_search(std::get<0>(tup), m, r1))
        {
            return std::get<2>(tup);
        }
    }
    return -100;
}

#endif

#if defined(_test_ss)
int main(void) {
    std::vector<std::uint32_t> shutter_settings_v = get_shutter_speeds_vector();
    for (auto x : shutter_settings_v) {
        std::cout << " read = " << x << std::endl;
    }
    std::cout << " len " << shutter_settings_v.size() << std::endl;
    return 1;
}
#endif

#endif

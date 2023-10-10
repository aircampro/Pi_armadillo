#ifndef __xml_adjustments_
#define __xml_adjustments_

#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

#include "focus_adjust.h"

const std::string XML_PATH1 = "test.xml";

#if defined(__focus_adjustment)

std::vector<std::uint32_t> get_iso_from_vector(std::string xml_filename);
std::vector<std::uint32_t> get_aper_from_vector(std::string xml_filename);
std::uint32_t get_iso_from_index(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs);
void get_index_from_iso(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs, std::uint32_t request_val);
std::uint32_t get_aper_from_index(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs);
void get_index_from_aper(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs, std::uint32_t request_val);
int get_value_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec, std::string iso_tag);

std::vector<std::uint32_t> get_iso_from_vector(std::string xml_filename)
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
        // <parameter name="CAM_ISO" type="uint32" default="400">
        // then pull out the values into the vector for
        // <option name="30" value="19660810"/>
        //
        if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
            std::cout << "name found " << nam << std::endl;
            std::stringstream ss;
            ss << nam;
            ss >> id_string;
            boost::regex r1("CAM_ISO");
            boost::smatch m;

            if (boost::regex_search(id_string, m, r1)) {

                for (auto it : pt_child.get_child("options")) {
                    if (auto valu = it.second.get_optional<std::uint32_t>("<xmlattr>.value")) {
                        std::cout << nam << " values are : " << valu << '\n';
                        std::stringstream ss1;
                        ss1 << valu;
                        ss1 >> val;
                        vec.push_back(val);
                    }
                }
                // just a loop for printing it for sanity check.. (remove)
                for (auto x : vec) {
                    std::cout << " the " << nam << "value = " << x << std::endl;
                }
                break;
            }
        }
    }
    return vec;
}

std::vector<std::uint32_t> get_aper_from_vector(std::string xml_filename)
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
        // <parameter name="CAM_ISO" type="uint32" default="400">
        // then pull out the values into the vector for
        // <option name="30" value="19660810"/>
        //
        if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
            std::cout << "name found " << nam << std::endl;
            std::stringstream ss;
            ss << nam;
            ss >> id_string;
            boost::regex r1("CAM_APERTURE");
            boost::smatch m;

            if (boost::regex_search(id_string, m, r1)) {

                for (auto it : pt_child.get_child("options")) {
                    if (auto valu = it.second.get_optional<std::uint32_t>("<xmlattr>.value")) {
                        std::cout << nam << " values are : " << valu << '\n';
                        std::stringstream ss1;
                        ss1 << valu;
                        ss1 >> val;
                        vec.push_back(val);
                    }
                }
                // just a loop for printing it for sanity check.. (remove)
                for (auto x : vec) {
                    std::cout << " the " << nam << "value = " << x << std::endl;
                }
                break;
            }
        }
    }
    return vec;
}

std::uint32_t get_iso_from_index(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs) {
    sfs->use_sdk_iso = sfs->use_sdk_iso % sss.size();
	//cli::tout << " \033[31m get iso from index = " << sss.size() << " " << sfs->use_sdk_iso << " \033[0m" << std::endl;
    sfs->prev_sdk_iso = sfs->use_sdk_iso;
    if (sss.size() > 0) { return sss[sfs->use_sdk_iso]; } else { return 0; }
}

void get_index_from_iso(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs, std::uint32_t request_val) {
    for (auto index : sss) {
        if (index == request_val) {
            sfs->use_sdk_iso = index;
            break;
        }
    }
}

std::uint32_t get_aper_from_index(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs) {
    sfs->use_sdk_fnum = sfs->use_sdk_fnum % sss.size();
    sfs->prev_sdk_fnum = sfs->use_sdk_fnum;
	if (sss.size() > 0) { return sss[sfs->use_sdk_fnum]; } else { return 0; }
}

void get_index_from_aper(std::vector<std::uint32_t> sss, sony_focus_settings_t* sfs, std::uint32_t request_val) {
    for (auto index : sss) {
        if (index == request_val) {
            sfs->use_sdk_fnum = index;
            break;
        }
    }
}

// gets the tag value from the camera last read state vector
//
int get_value_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec, std::string iso_tag)
{
    for (auto tup : vec) {
        boost::regex r1(iso_tag);
        boost::smatch m;

        if (boost::regex_search(std::get<0>(tup), m, r1))
        {
            return std::get<2>(tup);
        }
    }
    return -100;
}

#endif



#endif
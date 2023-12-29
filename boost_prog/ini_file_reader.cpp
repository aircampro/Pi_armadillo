#ifndef __ini_reader_
#define __ini_reader_

// Example code to read the ini file with the default camera set-up paramtersER
//
#include <iostream>
#include <string>
#include <strstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <boost/cregex.hpp>
#include <boost/regex.hpp>
#include <regex>
#include <boost/version.hpp>     // check the version of boost and use accordingly

namespace protree = boost::property_tree;

// ============= use BOOST MATCH if you want it depends on your version of boost library ============
#define _USE_BOOST_MATCH

#if BOOST_VERSION <= 106000
bool BoostRegMatch(std::string exp, std::string dataS)
{
   boost::RegEx regexp;
   regexp.SetExpression(exp.c_str (), true);

   if (regexp.Match(dataS.c_str () ) ) {
        return (true);
   } else {
        return (false);
   }
}
#else
bool BoostRegMatch(std::string exp, std::string dataS)
{
   boost::regex reA;
   reA = exp.c_str();
   boost::smatch sm;                    
   return (boost::regex_match(dataS, sm, reA));
}
#endif

bool StdRegMatch(std::string exp, std::string dataS)
{
     std::regex pattern(".*" + exp + ".*");
     return (std::regex_match(dataS, pattern)); 
}

int read_ini_file(std::vector<std::pair<std::string, std::string>>&v, std::string my_cam)
{
    // use boost property_tree
    protree::ptree tree;
    int cam_model = 0;
    std::string config_path = "./sample.conf";
    //std::string my_cam = "sony";
    //std::string my_cam = "workswell";
    //std::string my_cam = "micasense";

    try {
        // property tree reads type : ini
        protree::read_ini(config_path, tree);
        std::vector<std::string> valStrList;

        for (auto& section : tree) {
           std::cout << " [" << section.first << "] " << std::endl;
#if defined(_USE_BOOST_MATCH)
           if (BoostRegMatch(my_cam, section.first)) {
#else
           if (BoostRegMatch(my_cam, section.first)) {
#endif
             cam_model = 1;
           } else {
             cam_model = 0;
           }
           if (cam_model == 1) {
               for (auto& key : section.second) {
                   std::string valStr = key.second.get_value<std::string>();
                   // strip the comments from the lines
                   boost::split(valStrList, valStr, boost::is_any_of("#;"));
                   std::string val_field = boost::algorithm::trim_copy(valStrList.at(0)) ;
 //                  std::cout << key.first << "=" << val_field << std::endl;
                   v.push_back(std::make_pair(key.first, val_field));
               }
           }
        }

    } catch (protree::ini_parser_error &e) {
        // If the expected key is not present or the type is incorrect, an error will occur
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

int get_ini_int_component(std::string ini_section, std::string look_for_key )
{
   std::vector<std::pair<std::string, std::string>> defaults;
   if (read_ini_file(defaults, ini_section)==0) {
     std::cout << "read your ini successfully " << defaults.size() << " records found" << std::endl;
     for (auto& p : defaults) {
#if defined(_USE_BOOST_MATCH)
         if (BoostRegMatch(look_for_key, p.first)==true) {
#else
         if (StdRegMatch(look_for_key, p.first)==true) {
#endif
             std::cout << "found match => " << p.first << " " << p.second << std::endl;
             int i_val = 0;
             std::stringstream ss;
             ss << p.second;
             ss >> i_val;
             return i_val;
         }
     }
   } else {
     std::cout << "could not open the ini or there is a config error in it " << std::endl;
   }
   return -1;
}

// uncomment the main code to test
#define __TEST_ACTIVE
#ifdef __TEST_ACTIVE
// ============== example shows you how to use the reader function ====================
//
int main(const int ac, const char* const * const av)
{
   std::vector<std::pair<std::string, std::string>> defaults;
   if (read_ini_file(defaults, "sony")==0) {
     std::cout << "read your ini successfully " << defaults.size() << " records found" << std::endl;
     for (auto& p : defaults) {
         std::cout << "tags found => " << p.first << " " << p.second << std::endl;
     }
   } else {
     std::cout << "could not open the ini or there is a config error in it " << std::endl;
   }
}
#endif

#endif

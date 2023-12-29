// Example code to read the ini file with the default camera set-up paramtersER
//
#ifndef __ini_reader_
#define __ini_reader_

#include <iostream>
#include <string>
#include <strstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <boost/cregex.hpp>

namespace pt = boost::property_tree;

bool BoostRegMatch(std::string exp, std::string data)
{
   boost::RegEx	regexp;
   regexp.SetExpression(exp.c_str (), true);

   if (regexp.Match(data.c_str () ) ) {
	return (true);
   } else {
	return (false);
   }
}

int read_ini_file(std::vector<std::pair<std::string, std::string>>&v, std::string my_cam)
{
    // use boost property_tree
    //std::vector<std::pair<std::string, std::string>>v;
    pt::ptree tree;
    int cam_model = 0;
    std::string config_path = "./sample.conf";
    //std::string my_cam = "sony";
    //std::string my_cam = "workswell";
    //std::string my_cam = "micasense";

    try {
        // property tree reads type : ini
        pt::read_ini(config_path, tree);
        std::vector<std::string> valStrList;
        
        for (auto& section : tree) {
           std::cout << " [" << section.first << "] " << std::endl;
           if (BoostRegMatch(my_cam, section.first)) {
             cam_model = 1;
//             std::cout << "matches your camera!" << std::endl;
           } else {
             cam_model = 0;
           }
//           std::cout << section.second.get<int>("confA") << std::endl;
           if (cam_model == 1) {
               for (auto& key : section.second) {
                   std::string valStr = key.second.get_value<std::string>();
//                   std::cout << " value string " << valStr << std::endl;
                   boost::split(valStrList, valStr, boost::is_any_of("#;"));
                   std::string val_field = boost::algorithm::trim_copy(valStrList.at(0)) ;
 //                  std::cout << key.first << "=" << val_field << std::endl;
                   v.push_back(std::make_pair(key.first, val_field));
               }
           }
        }

    } catch (pt::ini_parser_error &e) {
        // If the expected key is not present or the type is incorrect, an error will occur
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}


int main(const int ac, const char* const * const av)
{
   std::vector<std::pair<std::string, std::string>> defaults;
   if (read_ini_file(defaults, "sony")==0) {
     std::cout << "read your ini successfully " << defaults.size() << " records found" << std::endl;
     for (auto& p : defaults) {
         std::cout << "tags found " << p.first << " " << p.second << std::endl;
     }
   } else {
     std::cout << "could not open the ini or there is a config error in it " << std::endl;
   }
}

#endif

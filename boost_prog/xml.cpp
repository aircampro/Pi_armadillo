// Example code to read the camera xml file and extract all the tag names equal to each option setting
//
// Compiles as :-  g++-8 -std=c++11 -o xml_test2 xml_test2.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system 
//                -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS
//
#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

// if you want levenstein to check how similar a word is to check the tag name validity to the option if we want to read the xml
// and check if the name assigned is similar to a given word add these and the function levenstein()
//
#include <algorithm>
using namespace std;
int LP[1005][1005]={};

struct Child {
        std::uint32_t xml_val;
        std::string option_name;
        Child() : xml_val(0) {}
        Child(std::uint32_t a, std::string s) : xml_val(a) { this->option_name = s; }
};

struct Parent {
        std::vector<Child> children;
        std::map<int, std::string> cameraOptions;
   
        inline void init() { children.clear(); }
        inline void addChild(std::uint32_t xml_val, std::string s) { children.push_back(Child(xml_val, s)); }
        void describe() {
            std::cout << "--- Describe begin --\n";
            for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
                std::cout << cit->option_name << " value : " << cit->xml_val << std::endl;
            std::cout << "--- Describe end --\n";
        }
        void make_map() {
            for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
                cameraOptions.insert(std::make_pair(cit->xml_val, cit->option_name));
        }
        std::vector<std::string> get_string_vectors() {
            std::vector<std::string> v;
            for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
                v.push_back(cit->option_name);
            return v;
        }
        std::vector<std::uint32_t> get_default_value_vectors() {
            std::vector<std::uint32_t> v;
            for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
                v.push_back(cit->xml_val);
            return v;
        }
};

void xml_load(const std::string &xml_file, Parent &parent) {
        parent.init();
        boost::property_tree::ptree pt_parent;
        boost::property_tree::read_xml(xml_file, pt_parent);
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt_parent.get_child("mavlinkcamera.parameters")) {
            // v.first is the name of the child.
            // v.second is the child tree.
				    //
            std::string my_string;
            std::uint32_t def_v;
            boost::optional<std::uint32_t> def;
//                std::cout << " 1st " << v.first << std::endl;
            const boost::property_tree::ptree &pt_child = v.second;
				    //
				    // this code below will extract the value from xml as shown below
				    // <mavlink>
            // <parameters>
            // <parameter type="age">3</parameter>
            // <parameter type="age">5</parameter>
            // <parameter type="age">7</parameter>
            // <parameter type="age">12</parameter>
            // </parameters>
            // </mavlink>
			 	    //
            //if (boost::optional<int> xml_val = pt_child.get_optional<int>("")) {
            //    //parent.addChild(xml_val.get());
            //}
				    //
				    // this extracts the name fields name and default from the xml as below
				    // <parameter name="CAM_APERTURE" type="uint32" default="400">
				    //
            if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
                    std::cout << "name found " << nam << std::endl;
                    std::stringstream ss;
                    ss << nam;
                    ss >> my_string;
            }
            if (def = pt_child.get_optional<std::uint32_t>("<xmlattr>.default")) {
                    std::cout << "def found " << def << std::endl;
                    std::stringstream ss1;
                    ss1 << def;
                    ss1 >> def_v;
            }
            parent.addChild(def_v, my_string);
        }
}

// given two string the levenstein function returns a number close to 0 if they are similar and larger if they dont match
// it is a measure which can be used in basic nlp to decide if the strings in the XML names are similar to what we expect
// for each option
//
int levenstein( std::string x, std::string y ) {
  int j,k;
  std::transform(x.begin(), x.end(),x.begin(), ::toupper);
  std::transform(y.begin(), y.end(),y.begin(), ::toupper);

  for(j=1;j<=x.size();j++) LP[j][0] = j;
  for(k=1;k<=y.size();k++) LP[0][k] = k;

  //I want a closer to b!
  for(j=1;j<=x.size();j++) {
    for(k=1;k<=y.size();k++) {
      //Remove a[j] or insert the same character as b[k] in a[j+1]
      //Adopt the minimum number of acts of the above 2
      int m = min(LP[j-1][k]+1, LP[j][k-1]+1);
      if(x[j-1] == y[k-1]) {
        // Since the last character is the same, the editing distance is
        m = min(m,LP[j-1][k-1]);
        LP[j][k] = m;
      }else {
        // Replace last character
        m = min(m,LP[j-1][k-1]+1);
        LP[j][k] = m;
      }
    }
  }
  cout << LP[x.size()][y.size()] << endl;
  return LP[x.size()][y.size()];
}

int main() {
    std::string xml_file("test.xml");

    Parent xml_param;
    xml_load(xml_file, xml_param);
    xml_param.describe();
    std::vector<std::string> ss = xml_param.get_string_vectors();
    for ( auto a : ss ) {
        std::cout << "string values :: " << a << std::endl;
	if (levenstein( "APERTURE", a ) <= 5 ) {
            std::cout << "similar tag to APERTURE is found as " << a << std::endl;
	}
	if (levenstein( "_WBMDE", a ) <= 5 ) {
            std::cout << "similar tag to WB is found as " << a << std::endl;
	}
    }
    std::vector<std::uint32_t> dv = xml_param.get_default_value_vectors();
    for ( auto v1 : dv ) {
        std::cout << "default values :: " << v1 << std::endl;
    }
    return 0;
}

#ifndef __xml_read
#define __xml_read

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

#define LEVENSTEIN_THRESH 5
namespace algo = boost::algorithm;

// if you want levenstein to check how similar a word is to check the tag name validity to the option if we want to read the xml
// and check if the name assigned is similar to a given word add these and the function levenstein()
//
#include <algorithm>
using namespace std;
int LP[1005][1005] = {};

// n-gram
//
#include <algorithm>
#include <numeric>
#include <random>

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
        for (std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
            std::cout << cit->option_name << " value : " << cit->xml_val << std::endl;
        std::cout << "--- Describe end --\n";
    }
    void make_map() {
        for (std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
            cameraOptions.insert(std::make_pair(cit->xml_val, cit->option_name));
    }
    std::vector<std::string> get_string_vectors() {
        std::vector<std::string> v;
        for (std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
            v.push_back(cit->option_name);
        return v;
    }
    std::vector<std::uint32_t> get_default_value_vectors() {
        std::vector<std::uint32_t> v;
        for (std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
            v.push_back(cit->xml_val);
        return v;
    }
};

void xml_load(const std::string& xml_file, Parent& parent) {
    parent.init();
    boost::property_tree::ptree pt_parent;
    boost::property_tree::read_xml(xml_file, pt_parent);
    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, pt_parent.get_child("mavlinkcamera.parameters")) {
        // v.first is the name of the child.
        // v.second is the child tree.
                //
        std::string my_string;
        std::uint32_t def_v;
        boost::optional<std::uint32_t> def;
        //                std::cout << " 1st " << v.first << std::endl;
        const boost::property_tree::ptree& pt_child = v.second;
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
int levenstein(std::string x, std::string y) {
    int j, k;
    std::transform(x.begin(), x.end(), x.begin(), ::toupper);
    std::transform(y.begin(), y.end(), y.begin(), ::toupper);

    for (j = 1; j <= x.size(); j++) LP[j][0] = j;
    for (k = 1; k <= y.size(); k++) LP[0][k] = k;

    //I want a closer to b!
    for (j = 1; j <= static_cast <int>(x.size()); j++) {
        for (k = 1; k <= static_cast <int>(y.size()); k++) {
            //Remove a[j] or insert the same character as b[k] in a[j+1]
            //Adopt the minimum number of acts of the above 2
            int m = min(LP[j - 1][k] + 1, LP[j][k - 1] + 1);
            if (x[j - 1] == y[k - 1]) {
                // Since the last character is the same, the editing distance is
                m = min(m, LP[j - 1][k - 1]);
                LP[j][k] = m;
            }
            else {
                // Replace last character
                m = min(m, LP[j - 1][k - 1] + 1);
                LP[j][k] = m;
            }
        }
    }
    //cout << LP[x.size()][y.size()] << endl;
    return LP[x.size()][y.size()];
}

// n-gram looks for similar sentances 
// https://en.wikipedia.org/wiki/N-gram
// 
bool n_gram(string a, string b, string c)
{
    using namespace std;
    //string a = "similar words in a sequence";
    //string b = "similar word ina sequence";
    //string c = "se";
    string tmp;
    vector<string> x;
    vector<string> y;
    int count_ = 0;
    for (int i = 0; i < a.size() - 1; i++)
    {
        tmp = a[i];
        tmp += a[i + 1];
        auto itr = find(x.begin(), x.end(), tmp); //There are duplicates in x, so this is excluded.
        if (itr == x.end())
            x.push_back(tmp);
    }
    for (int i = 0; i < b.size() - 1; i++)
    {
        tmp = b[i];
        tmp += b[i + 1];
        auto itr = find(y.begin(), y.end(), tmp); // as above
        if (itr == y.end())
            y.push_back(tmp);
    }

    //sum
    vector<string> wa;
    wa = x;
    for (int i = 0; i < y.size(); i++)
    {
        auto itr = find(x.begin(), x.end(), y[i]);
        if (itr == x.end())
        {
            wa.push_back(y[i]);
        }
    }
    cout << "***sum***" << endl;
    for (int i = 0; i < wa.size(); i++)
    {
        cout << wa[i] << endl;
    }
    //product
    vector<string> seki;
    for (int i = 0; i < y.size(); i++)
    {
        auto itr = find(x.begin(), x.end(), y[i]);
        if (itr != x.end())
        {
            seki.push_back(y[i]);
        }
    }
    cout << "***product***" << endl;
    for (int i = 0; i < seki.size(); i++)
    {
        cout << seki[i] << endl;
    }
    //difference
    vector<string> sa;
    for (int i = 0; i < x.size(); i++)
    {
        auto itr = find(y.begin(), y.end(), x[i]);
        if (itr == y.end())
        {
            sa.push_back(x[i]);
        }
    }
    cout << "***difference***" << endl;
    for (int i = 0; i < sa.size(); i++)
    {
        cout << sa[i] << endl;
    }

    cout << c << " in X and Y * **" << endl;
    if (find(x.begin(), x.end(), c) != x.end() && find(y.begin(), y.end(), c) != y.end())
    {
        cout << "True" << endl;
        return true;
    }
    else 
    {
        cout << "False" << endl;
        return false;
    }
}

int read_and_compare(std::string fil) {
    std::string xml_file(fil);

    Parent xml_param;
    xml_load(xml_file, xml_param);
    xml_param.describe();
    std::vector<std::string> ss = xml_param.get_string_vectors();
    for (auto a : ss) {
        std::cout << "string value :: " << a << std::endl;
        // for example look to see if we have APERTURE (key word) in the list of tags specified in the xml file
        //
        if ((levenstein("APERTURE", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "APERTURE")==true)) {
            std::cout << "found " << a << "similar tag to APERTURE with value " << std::endl;
        }
        else if ((levenstein("ISO", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "ISO") == true)) {
            std::cout << "found " << a << " similar to ISO with value : " << std::endl;
        }
        else if ((levenstein("SHUTTER_SPEED", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "SHUTTER_SPEED") == true)) {
            std::cout << "found " << a << " similar to SHUTTER_SPEED with value : " << std::endl;
        }
        else if ((levenstein("STILL_CAP", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "STILL_CAP") == true)) {
            std::cout << "found " << a << " similar to STILL_CAP with value : " << std::endl;
        }
        else if ((levenstein("EXPOSURE_PROGRAM", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "EXPO") == true)) {
            std::cout << "found " << a << " similar to EXPOSURE_PROGRAM with value : " << std::endl;
        }
        else if (((levenstein("_WBMODE", a) <= LEVENSTEIN_THRESH) || (levenstein("_WHITE_BAL", a) <= LEVENSTEIN_THRESH)) || (algo::contains(a, "WHITE") == true)) {
            std::cout << "found " << a << " similar to WHITE_BALANCE with value : " << std::endl;
        }
        else if ((levenstein("FOCUS_AREA", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "FOCUS_AREA") == true)) {
            std::cout << "found " << a << " similar to FOCUS_AREA with value : " << std::endl;
        }
        else if ((levenstein("FOCUS_MODE", a) <= LEVENSTEIN_THRESH) || (algo::contains(a, "FOCUS_MODE") == true)) {
            std::cout << "found " << a << " similar to FOCUS_MODE with value : " << std::endl;
        }
    }
    std::vector<std::uint32_t> dv = xml_param.get_default_value_vectors();
    for (auto v1 : dv) {
        std::cout << "default values :: " << v1 << std::endl;
    }
    return 0;
}

//#define __UNIT_TEST_ON
#ifdef __UNIT_TEST_ON
int main(void) {
    read_and_compare("test.xml");
}
#endif

#endif
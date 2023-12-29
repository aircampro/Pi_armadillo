#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

// convert string to int32_t type
//
boost::optional<std::int32_t> boost_string_to_int( std::string s ) {
    try {
       return boost::lexical_cast<std::int32_t>(s);
    } catch(boost::bad_lexical_cast &e) {
       std::cout << s << "(error): " << e.what() << "\n";
       return boost::none;
    }
}

struct Child {
        int xml_val;
        std::string option_name;
//        Child() : xml_val(0) {}
        Child(int a, std::string s) : xml_val(a) { this->option_name = s; }
};

struct Parent {
        std::vector<Child> children;
        std::map<int, std::string> cameraOptions;
        inline void init() { children.clear(); }
        inline void addChild(int xml_val, std::string s) { children.push_back(Child(xml_val,s)); }
        void describe() {
            std::cout << "--- Describe begin --\n";
            for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
                std::cout << cit->option_name << " value : " << cit->xml_val << std::endl;
            std::cout << "--- Describe end --\n";
        }
        void make_map() {
            cameraOptions.clear();
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
                std::string my_string;
                int def_v;
                boost::optional<std::uint32_t> def; 
//                std::cout << " 1st " << v.first << std::endl;
                const boost::property_tree::ptree &pt_child = v.second;
                if (boost::optional<int> xml_val = pt_child.get_optional<int>("")) {
                    //parent.addChild(xml_val.get());
                }
                if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
                    std::cout << "name found " << nam << std::endl;
                    std::stringstream ss;
                    ss << nam;
                    ss >> my_string;
                 //	parent.addChild(2,my_string);
                }
                if (def = pt_child.get_optional<std::uint32_t>("<xmlattr>.default")) {
                    std::cout << "def found " << def << std::endl;
                    std::stringstream ss1;
                    ss1 << def;
                    ss1 >> def_v;
                 	//parent.addChild(2,my_string);
                }
                parent.addChild(def_v, my_string);
        }
}

int main() {
    std::string xml_file("test.xml");

    Parent parent2;
    xml_load(xml_file, parent2);
    parent2.describe();
    parent2.make_map();
    std::vector<std::string> ss = parent2.get_string_vectors();
    for ( auto a : ss ) {
        std::cout << "string values :: " << a << std::endl;
    }
    std::vector<std::uint32_t> dv = parent2.get_default_value_vectors();
    for ( auto v1 : dv ) {
        std::cout << "default values :: " << v1 << std::endl;
    }

    return 0;
}

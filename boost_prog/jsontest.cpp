#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/format.hpp>
#include <string>

int main(int argc, const char *argv[]) {
    std::stringstream s("{\"foo\": [10], \"my_tag\": 70}");
    int age = 10;
    std::string job = "23456";
    std::cout << boost::format("\"foo\" : %1%, \"my_tag\" : %2%") % job % age << std::endl;
    std::stringstream s1;
    std::string x;
    s1 << boost::format("{\"foo\":%1%, \"my_tag\": %2%}") % job % age;
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(s1, pt);
    pt.add<int>("hoge", 34);
    boost::property_tree::write_json(std::cout, pt, false);
    std::string f_name = "filename.json";
    boost::property_tree::write_json(f_name, pt);
    int height = 0;
    std::cout << "my tag had value " << height << std::endl;
    // Create a root
    boost::property_tree::ptree root;

    // Load the json file in this ptree
    boost::property_tree::read_json("filename.json", root);
    height = root.get<int>("my_tag", 0);
    auto name = root.get<std::uint32_t>("foo", 0);
    auto hoge = root.get<int>("hoge", 0);
    std::cout << "my tag had value " << height << std::endl;
    std::cout << "my foo had value " << name << std::endl;
    std::cout << "my hoge had value " << hoge << std::endl;
    std::stringstream root_full_read;
    boost::property_tree::write_json(root_full_read, root, true);
    std::cout << "print the incoming stream " << root_full_read.str() << std::endl;
}

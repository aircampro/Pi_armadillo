#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
 
const std::string XML_PATH1 = "test.xml";
 
int main()
{
    // Traverse property tree example
    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(XML_PATH1, pt);
	
    for (auto it : pt.get_child("mavlinkcamera.parameters.parameter.options")) {
        if (auto isbn = it.second.get_optional<long long>("<xmlattr>.value"))
            std::cout << "ISBN : " << isbn << '\n';
    }
 
    for (auto it : pt.get_child("mavlinkcamera.parameters.parameter.options")) {
        if (auto isbn = it.second.get_optional<long long>("<xmlattr>.value"))
            std::cout << "ISBN : " << isbn << '\n';
    }
    return 0;
}

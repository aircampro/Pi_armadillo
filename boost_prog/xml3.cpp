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

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, pt.get_child("mavlinkcamera.parameters")) {
        // v.first is the name of the child.
        // v.second is the child tree.
                //
        std::string my_string;
        std::uint32_t def_v;
        boost::optional<std::uint32_t> def;
        //                std::cout << " 1st " << v.first << std::endl;
        const boost::property_tree::ptree& pt_child = v.second;

        //
        // this extracts the name fields name and default from the xml as below
        // <parameter name="CAM_APERTURE" type="uint32" default="400">
        //
        if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
            std::cout << "name found " << nam << std::endl;
            std::stringstream ss;
            ss << nam;
            ss >> my_string;
            for (auto it : pt_child.get_child("options")) {
                if (auto isbn = it.second.get_optional<long long>("<xmlattr>.value"))
                    std::cout << "values are : " << isbn << '\n';
            }
        }
	}

}

#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

const std::string XML_PATH1 = "test.xml";

int main()
{

    // Traverse property tree example
    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(XML_PATH1, pt);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, pt.get_child("mavlinkcamera.parameters")) {
        // v.first is the name of the child.
        // v.second is the child tree.

        std::string id_string;
        boost::optional<std::uint32_t> def;
		std::uint32_t val;
		std::vector<std::uint32_t> vec;
		
        const boost::property_tree::ptree& pt_child = v.second;
        //
        // this extracts the name fields name and default from the xml as below
        // <parameter name="CAM_SHUTTERSPEED" type="uint32" default="400">
        //
        if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.name")) {
            std::cout << "name found " << nam << std::endl;
            std::stringstream ss;
            ss << nam;
            ss >> id_string;
            boost::regex r1("CAM_SHUTTERSPD");
            boost::smatch m;

            if( boost::regex_search(id_string, m, r1) ) {

                for (auto it : pt_child.get_child("options")) {
                    if (auto valu = it.second.get_optional<long long>("<xmlattr>.value")) {
                        //std::cout << "Shutter Speeds are : " << valu << '\n';
						std::stringstream ss1;
						ss1 << valu;
						ss1 >> val;
						vec.push_back(val);
					}
                }
				for ( auto x : vec ) {
				    std::cout << " the shutter speed value = " << x << std::endl;
				}
                                break;
			}
        }
    }

}

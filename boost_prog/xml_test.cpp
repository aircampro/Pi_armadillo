#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <cstring>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

struct Child {
	int age;
	std::string option_name;
	Child() : age(0) {}
	Child(int a) : age(a) {}
};

struct Parent {
	std::vector<Child> children;
	inline void init() { children.clear(); }
	inline void addChild(int age) { children.push_back(Child(age)); }
	void describe() {
		std::cout << "--- Describe begin --\n";
		for(std::vector<Child>::const_iterator cit = children.begin(); cit != children.end(); cit++)
			std::cout << cit->option_name << " value : " << cit->age << std::endl;
		std::cout << "--- Describe end --\n";
	}
};

void load(const std::string &xml_file, Parent &parent) {
	parent.init();
	boost::property_tree::ptree pt_parent;
	boost::property_tree::read_xml(xml_file, pt_parent);
	BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt_parent.get_child("mavlinkcamera.parameters")) {
		// v.first is the name of the child.
		// v.second is the child tree.
                std::cout << " 1st " << v.first << std::endl;
                //std::cout << " 2nd " << v.second << std::endl;
		const boost::property_tree::ptree &pt_child = v.second;
		if (boost::optional<int> age = pt_child.get_optional<int>("")) {
			parent.addChild(age.get());
		}
		if (boost::optional<std::string> nam = pt_child.get_optional<std::string>("<xmlattr>.options")) {
                    std::cout << "name found " << nam << std::endl;
                }
	}
	boost::property_tree::ptree propertyTree;
	
	boost::property_tree::read_xml(xml_file, propertyTree);
	
	BOOST_FOREACH(auto &v, propertyTree)
	{
		std::cout << "Button is " << v.second.get<std::string>("") << std::endl;
	}
}

int main() {
	std::string xml_file("test.xml");

	Parent parent2;
	load(xml_file, parent2);
	parent2.describe();
        std::string ss = "hello mate what the XXXXx is this function doing !!!! made even longer in length";
        //const char* cstr = ss.c_str();
        char* cstr = new char[ss.size() + 1];
        std::char_traits<char>::copy(cstr, ss.c_str(), ss.size() + 1);        
        char into[3];
        float chunks = ss.length() / sizeof(into);
        std::cout << "len " << ss.length() << "into " << sizeof(into) << "chunk " << chunks << std::endl;
        int n = 0;
        while(n < ss.length()) {
           if ((ss.length()-n) < sizeof(into)) {
               std::strncpy(&into[0],cstr+n,(ss.length()-n)); 
               into[ss.length()-n] = '\0';
               printf(" copy %s\n",into);
               n += (ss.length()-n)+1;
           }
           else {
               std::strncpy(&into[0],cstr+n,sizeof(into)); 
               printf(" copy %s\n",into);
               n += sizeof(into);
           }
        }
        delete [] cstr; 
	return 0;
}

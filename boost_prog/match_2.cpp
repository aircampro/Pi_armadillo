#include <iostream>
#include <string>
#include <boost/regex.hpp>
using namespace std;

int main()
{
	//Search with a regular expression that matches a string that starts with < and ends with >
	boost::regex  r( "<[^>]+>" );
	boost::smatch m;
	string str1 = "The HTML tag <title> means that ...";

	if( boost::regex_search(str1, m, r) )
	{
		cout << "found (pos=" << m.position() << ")" << endl;
		cout << " ==> " << m.str() << endl;
	}

        // look for line and match it
        boost::regex r1("LINE");
        string str99 = "GDLM LINE";

	if( boost::regex_search(str99, m, r1) )
	{
		cout << "LINE found (pos=" << m.position() << ")" << endl;
		cout << " ==> " << m.str() << endl;
	}
	// Replace the matched part with a string with a # before and after
	boost::regex r2( "A([1-9]*|[a-z]*)A" );
	string str2 = "A123A AaaaA A3b3A A9A";

	{
		cout <<
			boost::regex_replace(
			  str2, r2, "#$0#", boost::format_all )
		<< endl;
	}
	return 0;
}

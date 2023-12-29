
#include <iostream>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/date_time/local_time_adjustor.hpp>
#include <sstream>

#include <vector>
#include <algorithm>
#include <functional>
#include <cstdlib>
#include <bits/stdc++.h>

#include <string>
#include <fstream>
#include <regex>

#include <boost/algorithm/string/classification.hpp> // is_any_of
#include <boost/algorithm/string/split.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/iterator.hpp>
#include <boost/format.hpp>

namespace pt = boost::posix_time;
namespace gg = boost::gregorian;
typedef boost::date_time::c_local_adjustor<pt::ptime> local_adj;
auto epoch = local_adj::utc_to_local(pt::ptime(gg::date(1970, 1, 1)));

void disp(const std::string& s)
{
    std::cout << s << std::endl;
}

int get_month_index( std::string name )
{
    std::map<std::string, int> months
    {
        { "Jan", 1 },
        { "Feb", 2 },
        { "Mar", 3 },
        { "Apr", 4 },
        { "May", 5 },
        { "Jun", 6 },
        { "Jul", 7 },
        { "Aug", 8 },
        { "Sep", 9 },
        { "Oct", 10 },
        { "Nov", 11 },
        { "Dec", 12 }
    };

    const auto iter = months.find( name );

    if( iter != months.cend() )
        return iter->second;
    return -1;
}

int main(void) {

            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();		   
	    std::stringstream ss;
	    ss << now;
	    // make local time adjustment
	    boost::date_time::c_local_adjustor<boost::posix_time::ptime> adj1;
	    boost::posix_time::ptime t1 = adj1.utc_to_local(now);
   	    std::stringstream ss2;
	    ss2 << t1;
	    auto date_time = pt::time_from_string(ss2.str());
            std::cout << date_time << "\n";
            std::string now_as_simple = boost::posix_time::to_simple_string(date_time);

            // split the date and the time
            std::vector<std::string> result;
            boost::algorithm::split(result, now_as_simple, boost::is_any_of(" ")); // split the date_time string
            std::cout << result.size() << " " << result[0] << "\n";
            std::vector<std::string> date_components;
            // split the date component
            boost::algorithm::split(date_components, result[0], boost::is_any_of("-")); // split the date
            std::cout << date_components[0] << "/" << date_components[1] << "/" << date_components[2] << "\n";
            // split the time component
            std::vector<std::string> time_comp;
            boost::algorithm::split(time_comp, result[1], boost::is_any_of(".")); // split the time
           std::cout <<  time_comp[0] << "\n";
	    std::stringstream ss100;
 ss100 << boost::format("SDTI %s/%s/%s-%s")  % date_components[0] % date_components[1] % date_components[2]  % time_comp[0];
           std::string sssss = "hello ";
	   std::string ans = ss100.str();
           sssss.append(ans);
           std::cout << sssss << "\n";
           int month = get_month_index(date_components[1]);
	   std::stringstream ss101;
 ss101 << boost::format("SDTI %s/%d/%s-%s")  % date_components[0] % month % date_components[2]  % time_comp[0];
           std::string new1 = "for owrkswell ";
	   ans = ss101.str();
new1.append(ans);
           std::cout << new1 << "\n";
}

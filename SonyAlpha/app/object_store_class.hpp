// ===================================================================================================================================================================================================
//     object_storage_class.hpp
//
//     Created by AiRobotDynamics 
//
// g++-8 -std=c++17 -o main main.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lboost_date_time
// ===================================================================================================================================================================================================

#ifndef obj_stor_class
#define obj_stor_class

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

namespace pt = boost::posix_time;
namespace gg = boost::gregorian;
typedef boost::date_time::c_local_adjustor<pt::ptime> local_adj;
auto epoch = local_adj::utc_to_local(pt::ptime(gg::date(1970, 1, 1)));

int idx_no = 0;
class storage_object
{
public:
	std::string my_name;
	std::uint32_t my_value = 99;
	std::uint32_t my_prev_value = 99;
	std::uint8_t write_protected = 0;
	bool update_me = false;
	std::uint32_t index_no = 0;
	std::uint64_t ts = 0L;
	std::uint64_t tm_delta = 0L;

	// do upon creation of the storage object class
	storage_object(std::string my_tag) {
		this->my_name = my_tag;
		std::cout << "value object creation : " << this->my_name << std::endl;
		this->index_no = ++idx_no;
		// choose seconds clock or microsecondsclock not sure if local time adjustment works in ms
		//
	//boost::posix_time::ptime now = boost::posix_time::second_clock::universal_time();
		boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
		std::stringstream ss;
		ss << now;
		// make local time adjustment
		boost::date_time::c_local_adjustor<boost::posix_time::ptime> adj1;
		boost::posix_time::ptime t1 = adj1.utc_to_local(now);
		std::stringstream ss2;
		ss2 << t1;
		auto date_time = pt::time_from_string(ss2.str());
		auto diff = date_time - epoch;
		this->ts = diff.total_seconds();
	}
	// do upon deletion of the storage class
	~storage_object() {
		std::cout << "value object deleted : [" << this->my_name << "] [" << this->my_value << "][" << this->my_prev_value << "][" << this->index_no << "]" << std::endl;
	}
	// --------------------------------------------------- methods associated with this class -------------------------------------------------------------
	//
	// modify the class name with that read from the xml
	void change_tag_name(std::string s) {
		this->my_name = s;
	}
	// add a new value to the class after successful setting
	int add_new_value() {
		// get the utc time clock
		//boost::posix_time::ptime now = boost::posix_time::second_clock::universal_time();
		boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
		std::stringstream ss;
		ss << now;
		// make local time adjustment
		boost::date_time::c_local_adjustor<boost::posix_time::ptime> adj1;
		boost::posix_time::ptime t1 = adj1.utc_to_local(now);
		std::stringstream ss2;
		ss2 << t1;
		std::cout << "utc : " << ss.str() << " local : " << ss2.str() << std::endl;
		// a fixed date for test purpose only :: auto date_time = pt::time_from_string("2013-06-15 19:37:33");
		auto date_time = pt::time_from_string(ss2.str());
		auto diff = date_time - epoch;
		this->my_prev_value = this->my_value;
		// DEBUG std::cout << "time : " << now << " " << diff.total_seconds() << std::endl;
		this->tm_delta = diff.total_seconds() - this->ts;
		std::cout << "time diff from last value : " << this->tm_delta << "tag : " << this->my_name << std::endl;
		this->ts = diff.total_seconds();
		this->update_me = true;
		std::cout << "confirmed setting for " << this->my_value << " tag : " << this->my_name << std::endl;
		// DEBUG std::cout << "timestamp : " << this->ts << std::endl;
		return 0;
	}
	// request a value change of the attribute
	int add_new_req(std::uint32_t my_new_value) {
		this->my_prev_value = this->my_value;
		this->my_value = my_new_value;
		//if you want update as soon as its seen --> this->update_me = true;
		std::cout << "added request for " << this->my_value << " tag : " << this->my_name << std::endl;
		return 0;
	}
	// clear the request to set the attribute
	int clr_new_req() {
		this->my_value = this->my_prev_value;
		return 0;
	}
	// set a flag to update the parmeters via mavlink
	int set_update() {
		this->update_me = true;
		return 0;
	}
	// clear the mavlink update flag after sending succesfult message
	int clr_update() {
		this->update_me = false;
		return 0;
	}
	// add data from a get all camera list
	int update_from_cam_vec(std::vector<std::tuple<std::string, std::uint8_t, std::uint32_t>>& vec)
	{
		for (auto tup : vec) {
			std::regex pattern(".*" + this->my_name + ".*");
			if (std::regex_match(std::get<0>(tup), pattern)) {
				// in this case we set them both so not to trigger setting of the option its used in initial read
					//this->my_prev_value = this->my_value;
				this->write_protected = std::get<1>(tup);
				this->my_value = std::get<2>(tup);
				this->my_prev_value = this->my_value;
				boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
				//boost::posix_time::ptime now = boost::posix_time::second_clock::universal_time();
				std::stringstream ss;
				ss << now;
				// make local time adjustment
				boost::date_time::c_local_adjustor<boost::posix_time::ptime> adj1;
				boost::posix_time::ptime t1 = adj1.utc_to_local(now);
				std::stringstream ss2;
				ss2 << t1;
				auto date_time = pt::time_from_string(ss2.str());
				auto diff = date_time - epoch;
				this->ts = diff.total_seconds();
				this->update_me = true;
			}
		}
		return 0;
	}
};
#endif /* object_store_class */
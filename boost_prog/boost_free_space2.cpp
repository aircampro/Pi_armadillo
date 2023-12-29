//
// Compiles with : g++-8 -std=c++17 -o boost_free_space boost_free_space.cpp -lboost_log -lpthread 
// -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup 
// -lboost_timer -lboost_system -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I/usr/lib/arm-linux-gnueabihf -lboost_filesystem
//
// find the filesystem library with the following command :: find_my_boostlib.sh filesystem
//
// for :: CMkakeLists.txt
// find_package(Boost REQUIRED COMPONENTS filesystem)
// target_link_libraries(${PROJECT_NAME} Boost::filesystem)
//


#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <iostream>
#include <cstdint>
#include <boost/filesystem.hpp>
#include <tuple>

namespace fs = boost::filesystem;
fs::path const dir("/home/pi");
fs::path const file("my_test.dat");
fs::path const path = dir / file;

std::tuple< std::intmax_t, std::intmax_t, std::intmax_t > get_pi_disk_usage() {

const fs::space_info si = fs::space(path);
        std::cout
            << "│ " << static_cast<std::intmax_t>(si.capacity) << ' '
            << "│ " << static_cast<std::intmax_t>(si.free) << ' '
            << "│ " << static_cast<std::intmax_t>(si.available) << ' '
            << "│ " << '\n';
	return std::make_tuple( static_cast<std::intmax_t>(si.capacity), static_cast<std::intmax_t>(si.free), static_cast<std::intmax_t>(si.available));
}

int main(void) {

/*

 This code does various other system functions
 
 
// does it exist
//
fs::exists(dir);
fs::exists(path);

// is it a directory
//
fs::is_directory(path);

// is it a regular file
//
fs::is_regular_file(path);

// is the path empty
//
fs::is_empty(path);

// If you do not want an exception to occur, pass an error code reference to the second argument.
//
boost::system::error_code error;
fs::is_empty(path, error);

// What is the file size (bytes)?
//
fs::file_size(path);
// 最後に更新した日時は?
fs::last_write_time(path);


*/

/*
const fs::space_info si = fs::space(path);
        std::cout
            << "│ " << static_cast<std::intmax_t>(si.capacity) << ' '
            << "│ " << static_cast<std::intmax_t>(si.free) << ' '
            << "│ " << static_cast<std::intmax_t>(si.available) << ' '
            << "│ " << '\n';
*/
std::tuple< std::intmax_t, std::intmax_t, std::intmax_t > tup = get_pi_disk_usage();
std::cout << " capacity " << std::get<0>(tup) << " free " << std::get<1>(tup) << " available " << std::get<2>(tup) << std::endl;
std::cout << " file exist " << fs::is_regular_file(path) << std::endl;
std::cout << " file_size " << fs::file_size(path) << std::endl;
return 0;

}

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
#include <iostream>
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
// used for checking strings
//
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

namespace boost_fs = boost::filesystem;
boost_fs::path const dir("/home/pi");
boost_fs::path const file("my_test.dat");
boost_fs::path const path = dir / file;

boost_fs::path const dir1("/home/pi/boost");
boost_fs::path const file1("test.csv");
boost_fs::path const path1 = dir1 / file1;

namespace algo = boost::algorithm;

std::uintmax_t get_mb(std::uintmax_t bytes)
{
  return (bytes / (1024 * 1024));
}

void print_space(const char* name, std::uintmax_t bytes)
{
  std::uintmax_t mega_bytes = bytes / (1024 * 1024);
  std::cout << name << " : " << bytes << "[B]"
            << " (" << mega_bytes << "[MB])" << std::endl;
}

std::tuple< std::intmax_t, std::intmax_t, std::intmax_t > get_pi_disk_usage() {

const boost_fs::space_info si = boost_fs::space(path);
        std::cout
            << "│ " << static_cast<std::intmax_t>(si.capacity) << ' '
            << "│ " << static_cast<std::intmax_t>(si.free) << ' '
            << "│ " << static_cast<std::intmax_t>(si.available) << ' '
            << "│ " << '\n';
        print_space("capacity", si.capacity);
        print_space("free", si.free);
        print_space("available", si.available);
        return std::make_tuple( static_cast<std::intmax_t>(si.capacity), static_cast<std::intmax_t>(si.free), static_cast<std::intmax_t>(si.available));
}

int main(void) {

/*

 This code does various other system functions add them and test them as required.


// does it exist
//
boost_fs::exists(dir);
boost_fs::exists(path);

// is it a directory
//
boost_fs::is_directory(path);

// is it a regular file
//
boost_fs::is_regular_file(path);

// is the path empty
//
boost_fs::is_empty(path);

// If you do not want an exception to occur, pass an error code reference to the second argument.
//
boost::system::error_code error;
boost_fs::is_empty(path, error);

// What is the file size (bytes)?
//
boost_fs::file_size(path);

// last write time
//
boost_fs::last_write_time(path);

// Copy file or directory (from -> to)
//
boost_fs::copy(from, to);
// Copy directory (from -> to)
//
boost_fs::copy_directory(from, to);
// Copy file (from -> to)
//
boost_fs::copy_file(from, to, option);
// Create directory. C:\foo\bar When you specify foo, it will make it at once even if there is no foo
//
boost_fs::create_directories(dir);
// Create directory. C:\foo\bar if foo is not present, it fails.
//
boost_fs::create_directory(dir);
// Delete file or directory
// Directories can only be deleted if they are empty
//
boost_fs::remove(path);
// Delete file or directory
// If the directory is specified, all contents are also deleted
//
boost_fs::remove_all(path);
// Rename file or directory
//
boost_fs::rename(from, to);
// Change file size
//
boost_fs::resize_file(file, 100);

// Get the file_status structure
// http://www.boost.org/doc/libs/1_46_1/libs/filesystem/v3/doc/reference.html#file_status
//
boost_fs::status(path);

// If you pass a relative path, it will return it as an absolute path
//
boost_fs::system_complete(file);

// Creates and returns a random filename.This is the best app I have ever used and I love it!!!!!!!!!!!
//
boost_fs::unique_path();

*/

// check for a file extension on a given path
std::string no_quo_path;
std::stringstream ss;
ss << path;
std::string path_s;
ss >> path_s;
// erase the quotes ""
no_quo_path = algo::erase_all_copy(path_s,"\"");
// now check the string ends with the extension
if (algo::iends_with(no_quo_path, ".dat")) {
   std::cout << " ends with .dat " << "\n";
}
// now check the string contains the extension
if (algo::contains(no_quo_path, ".dat")) {
   std::cout << " ends with .dat " << "\n";
}

const boost_fs::space_info si = boost_fs::space(path);
        std::cout
            << "│ " << get_mb(static_cast<std::intmax_t>(si.capacity)) << ' '
            << "│ " << get_mb(static_cast<std::intmax_t>(si.free)) << ' '
            << "│ " << get_mb(static_cast<std::intmax_t>(si.available)) << ' '
            << "│ " << '\n';
std::tuple< std::intmax_t, std::intmax_t, std::intmax_t > tup = get_pi_disk_usage();
std::cout << " capacity " << std::get<0>(tup) << " free " << std::get<1>(tup) << " available " << std::get<2>(tup) << std::endl;
std::cout << " file exist " << boost_fs::is_regular_file(path) << std::endl;
std::cout << " file_size " << boost_fs::file_size(path) << std::endl;

// find file and delete
//
//if (boost_fs::exists(path1) == 1) {
//    boost_fs::remove(path1);
//}
// write file stream
//
// as appended file
//
// std::ofstream ofs(path1, std::ios::app | std::ios::ate);
//
// create and ovcerwrite with capacity in reality we will be writing the picture counter

//
std::ofstream ofs(path1);
if (!ofs) {
   std::cerr << "failed to write counter file" << std::endl;
   std::exit(-1);
}
ofs << get_mb(static_cast<std::intmax_t>(si.available)) << std::endl;

// read file stream to recall the photos counted total after reboot
// done in camera initialisation routine
//
std::ifstream ifs(path1);
if (ifs.fail()) {
  std::cerr << "Failed to open counter file." << std::endl;
  std::exit(-1);
}
int bb;
ifs >> bb;
std::cout << "got value from file " << bb << std::endl;

return 0;

}


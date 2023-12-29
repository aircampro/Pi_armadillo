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

#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string.hpp>

namespace algo = boost::algorithm;

namespace fs = boost::filesystem;
fs::path const dir("/home/pi");
fs::path const file("my_test2.dat");
fs::path const path = dir / file;

int main(void) {

/*
fs::exists(dir);
fs::exists(path);

fs::is_directory(path);

fs::is_regular_file(path);

fs::is_empty(path);
boost::system::error_code error;
fs::is_empty(path, error);

fs::file_size(path);
fs::last_write_time(path);
*/
std::string no_quo_path;
std::stringstream ss;
ss << path;
std::string path_s;
ss >> path_s;
std::cout << path_s << " " << (algo::iends_with(path_s,".dat")) << "\n";
no_quo_path = algo::erase_all_copy(path_s,"\"");
if (algo::iends_with(no_quo_path, ".dat")) {
   std::cout << " ends with .dat " << "\n";
}
std::string fil = "test.dat";
if (algo::contains(no_quo_path, ".da")) {
   std::cout << " contains .da " << "\n";
}
std::cout << fil << " " << (algo::iends_with(fil,".dat")) << "\n";

int width = 14;
const fs::space_info si = fs::space(path);
        std::cout
            << "│ " << static_cast<std::intmax_t>(si.capacity) << ' '
            << "│ " << static_cast<std::intmax_t>(si.free) << ' '
            << "│ " << static_cast<std::intmax_t>(si.available) << ' '
            << "│ " << '\n';
return 0;

}

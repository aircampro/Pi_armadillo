//
// It is example boost code to parse the command line it works in a similar fashion to using tclap library
//
// g++-8 -std=c++17 -o boost_args boost_parse_args.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex 
// -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -DBOOST_BIND_GLOBAL_PLACEHOLDERS -lboost_program_options
//

#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(const int ac, const char* const * const av)
{
    // Define options
    po::options_description description("option");
    description.add_options()
    // ("Option name", "Argument (optional)", "Option description")
    ("number,n", po::value<int>()->default_value(12345), "")
    ("string,s", po::value<std::string>(), "string")
    ("string2", po::value<std::string>()->default_value("abcd"), "string2")
	("compression,c", po::value<std::vector<int>>(), "set compression level")
    ("help,h", "show this help message");

    // Command Line Parsing
    po::variables_map vm;
    try {
        po::store(parse_command_line(ac, av, description), vm);
    } catch (po::error &e) {
    // Nonexistent option ・ throw an exception if the wrong type is specified
       std::cout << e.what() << std::endl;
       return -1;
    }

    po::notify(vm);

    //help, output help (description content) when h is entered
    if (vm.count("help")) {
      std::cout << description << std::endl;
     return 0;
    }

    // Option number output
	if (vm.count("number")) {
        int num = vm["number"].as<int>();
        std::cout << num << std::endl;
	}

    if (vm.count("string")) {
        try {
        // Output of option string
           std::string str = vm["string"].as<std::string>();
           std::cout << str << std::endl;
        } catch (boost::bad_any_cast &e) {
        // default Throw an exception if an option with no value is not specified
           std::cout << e.what() << std::endl;
           return -1;
        }
	}

    //std::cout << vm["string2"].as<std::string>() << std::endl;

    if (vm.count("compression")) {
        std::vector<int> v = vm["compression"].as<std::vector<int>>();
        std::cout << "Compression level was set to ";
        std::for_each(v.begin(), v.end(), [](int i){std::cout << i << " ";});
        std::cout << std::endl;
    }

    //try {
    //  auto hoge = vm["hoge"].as<int>();
    //} catch (boost::bad_any_cast &e) {
    // Spit out an exception when using an option that does not exist
    //  std::cout << e.what() << std::endl;
    //  return -1;
    //}

    return 0;
}

//
// example boost code to parse the command line it works in a similar fashion to tclap
// g++-8 -std=c++17 -o  boost_parse_args boost_parse_args.cpp -lboost_program_options -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread
//
#include <iostream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(const int ac, const char* const * const av)
{
    // Define options
    po::options_description description("option (times are in microseconds)");
    description.add_options()
    // ("Option name", "Argument (optional)", "Option description")
    ("number,n", po::value<int>()->default_value(12345), "")
    ("string,s", po::value<std::string>(), "string")
    ("string2", po::value<std::string>()->default_value("abcd"), "string2")
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
    int num = vm["number"].as<int>();
    std::cout << num << std::endl;

    /*
    try {
    // Output of option string
       std::string str = vm["string"].as<std::string>();
       std::cout << str << std::endl;
    } catch (boost::bad_any_cast &e) {
    // default Throw an exception if an option with no value is not specified
      std::cout << e.what() << std::endl;
      return -1;
    }
    */

    std::cout << vm["string2"].as<std::string>() << std::endl;

    /* example of wrong option being addressed 
    try {
      auto hoge = vm["hoge"].as<int>();
    } catch (boost::bad_any_cast &e) {
    // Spit out an exception when using an option that does not exist
      std::cout << e.what() << std::endl;
      return -1;
    }
    */

    std::cout << "its value to " << num << std::endl;

    return 0;
}

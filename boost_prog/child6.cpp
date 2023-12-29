#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <string>
#include <vector>
 
namespace bp = ::boost::process;
 
bp::child start_child()
{
    std::string exec = "bjam";
    std::vector<std::string> args;
    args.push_back("bjam");
    args.push_back("--version");
 
    bp::context ctx;
    ctx.stdout_behavior = bp::capture_stream();
 
    return bp::launch(exec, args, ctx);
}
 
int main()
{
    bp::child c = start_child();
 
    bp::pistream &is = c.get_stdout();
    std::string line;
    while (std::getline(is, line))
        std::cout << line << std::endl;
    bp::status s = c.wait();
 
    return s.exited() ? s.exit_status() : EXIT_FAILURE;
}

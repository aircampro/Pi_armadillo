#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <boost/asio.hpp>
#include <boost/process.hpp>
#include <string>

namespace bp = ::boost::process;

int main()
{
try
{
// Standard input/output is piped
// Standard error output redirects to standard output
bp::context ctx;
ctx.m_stdin_behavior = bp::capture_stream();
ctx.m_stdout_behavior = bp::capture_stream();
ctx.m_stderr_behavior = bp::redirect_stream_to_stdout();

std::string exe = bp::find_executable_in_path("sort");
bp::child c = bp::launch_shell(exe, ctx);

bp::postream& os = c.get_stdin();
os <<
"one\n"
"two\n"
"three\n"
;
os.flush(); // required
os.close(); // required

bp::pistream& is = c.get_stdout();
std::string line;
while (std::getline(is, line))
std::cout << line << '\n';

c.wait(); // wait for completion

// Handle of child process and its main thread is enabled again (bug)
}
catch (const std::exception& e)
{
std::cerr << "Error: " << e.what() << std::endl;
}
}

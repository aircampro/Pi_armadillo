#include <boost/process.hpp> // version 0.5
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <string>

using namespace boost::process;
using namespace boost::process::initializers;
using namespace boost::iostreams;

int main() {
    pipe p = create_pipe();
    {
        // note the scope for sink
        file_descriptor_sink sink(p.sink, close_handle);
        /*  child c = */ // not necessary to hold a child object, it seems.
        execute(run_exe("/usr/bin/ls"), bind_stdout(sink));
    }   // note the scope for sink

    file_descriptor_source source(p.source,  close_handle);
    stream<file_descriptor_source> is(source);
    std::string s;
    while(std::getline(is, s)) {
        std::cout << "read: " << s << std::endl;
    }
    std::clog << "end" << std::endl; // never reach
}

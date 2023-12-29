#include <boost/process.hpp>
namespace bp = boost::process;

bool is_process_running(std::string p_name){
    std::vector<std::string> args { "-c", "ps aux 2>&1" };
    bp::ipstream out;
    bp::child c(bp::search_path("sh"), args, bp::std_out > out);

    for (std::string line; c.running() && std::getline(out, line);) {
        if (line.find(p_name) != std::string::npos) {
            return true;
        }
    }
    c.wait();

    return false;
}

#include <iostream>
int main() {
    std::cout << "mavlink-router: " << is_process_running("mavlink") << "\n";
}

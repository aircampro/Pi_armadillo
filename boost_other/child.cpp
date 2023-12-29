
#include <cstdlib>
#include <boost/process.hpp>
#include <string> 
#include <vector>

namespace bp = boost::process;
int main(void) {
    std::string exec = "ps"; 

        std::vector<std::string> args; 
	    args.push_back("-ael"); 

	        bp::context ctx; 
		    ctx.stdout_behavior = bp::silence_stream(); 

		        bp::child c = bp::launch(exec, args, ctx); 
	return 0;
}

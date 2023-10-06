#include <iostream>
#include <string>
#include <strstream>
// sending & reading http responses
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/regex.hpp>
using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;
namespace pt = boost::property_tree;

#define YOUR_CAM_IP_ADDR "192.168.10.254"
#define DEFAULT_VAL_NOT_SET -99

#include <regex>
#include <list>
#include <vector>
#include <map>

#include <sstream>

// end the request with a keep-alive to the socket
//
void set_msg_end_keep_connection( std::string& req_msg, std::string& ip ) {
			req_msg.append("Host:");
			req_msg.append(ip);
			req_msg.append("\r\n");						
			req_msg.append("Connection: Keep-Alive\r\n");     
}

// end the request with a close to the socket
//
void set_msg_end_close_connection( std::string& req_msg, std::string& ip ) {
			req_msg.append("Host:");
			req_msg.append(ip);
			req_msg.append("\r\n");						   
            req_msg.append("Connection: close\r\n\r\n");      // <--- to close the connection
}

int main(void) {

    std::string my_msg = " POST /hello/world.htm HTTP/1.1\r\n";
	std::string my_ip = YOUR_CAM_IP_ADDR;
	set_msg_end_close_connection(my_msg, my_ip);
	std::cout << my_msg << std::endl;
}
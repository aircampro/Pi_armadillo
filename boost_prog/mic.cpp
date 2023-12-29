#include <boost/asio.hpp>
#include <iostream>
/*

g++-8 -std=c++17 -o mica_test mica_test.cpp -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-deprecated

*/
#include <string>
#include <strstream>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;
namespace pt = boost::property_tree;

int main(void)
try
{
    asio::io_service io_service;
    
    //Create a TCP socket
	//
    ip::tcp::socket sock(io_service);

    //Name resolution (hostname to IP address conversion)
	//
    //ip::tcp::resolver resolver(io_service);
    //ip::tcp::resolver::query query("www.kumei.ne.jp", "http");

    //Configure Host Information
	//
    //ip::tcp::endpoint endpoint(*resolver.resolve(query));

    // fixed ip connection
	//
    ip::tcp::endpoint endpoint( ip::address::from_string("192.168.10.254"), 80);

    //Connect to Socket
	//
    sock.connect(endpoint);

    //Send Message
	//
    asio::streambuf request;
    ostream request_ostream(&request);
	
	// get exposure
	//
    request_ostream << "GET /exposure HTTP/1.0\r\n\r\n";
	
	// take a photo
	//string json_param = "{ 'store_capture' : True, 'block' : True }";
	//request_ostream << "POST /capture HTTP/1.0\r\n\r\n";
	//request_ostream << "Content-Type: application/x-www-form-urlencoded; charset=UTF-8\r\n";
	//request_ostream << "Content-Length: " << json_param.length() << "\r\n";
    //request_ostream << "Connection: Close\r\n";
	//request_ostream << "\r\n";
	//request_ostream << json_param << "\r\n";
    asio::write(sock, request);

    //Receive message
	//
    asio::streambuf buffer;
    boost::system::error_code error;
    asio::read(sock, buffer, asio::transfer_all(), error);
    //if (error && error != asio::error::eof)
    //{
    //    std::cout << "receive failed: " << error.message() << std::endl;
    //}
    //else
    //{
    //    cout << &buffer;
    //}
	//
    // this below is a regresion test to test the json parser library is working okay
	//
	//pt::ptree tree;
    //std::string dummy_json("{\"enable_man_exposure\":false,\"exposure1\":0.1,\"exposure2\":0.1,\"exposure3\":0.1,\"exposure4\":0.1,\"exposure5\":0.1,\"gain1\":1,\"gain2\":1,\"gain3\":1,\"gain4\":1,\"gain5\":1}");

    //std::istrstream is0(dummy_json.c_str());
    //pt::read_json(is0, tree);
    //std::cout << tree.get<bool>("enable_man_exposure") << std::endl;
    //std::cout << tree.get<double>("exposure1") << std::endl;

    // now convert the output recieved over http
    // the result is ocnverted from the http query to a string you can see it contains the result of the query as well as the data segment
    //
    std::string result = boost::asio::buffer_cast<const char*>(buffer.data());
    std::istrstream is1(result.c_str());
    std::cout << " my result is :: " << result << std::endl;
	//
    // now look and parse it line by line
    //
    std::string str;
    std::istream is(&buffer);
    int numline=0;
    std::string s;
    // is >> s -- we cant this as it contains 3 other lines at the start
	//
	// extract only the 4th line which is the data segment
	//
    while(buffer.size() !=0)
    {
      ++numline;
      std::getline(is, str, '\n');
//      std::cout << str << " " << numline << std::endl;
      if (numline == 4) s = str;
    }
    // now parse the data segment into values using the json library
	//
    pt::ptree tree1;
    std::istrstream is2(s.c_str());
    pt::read_json(is2, tree1);
    std::cout << tree1.get<bool>("enable_man_exposure") << std::endl;
    std::cout << tree1.get<double>("exposure1") << std::endl;
    std::cout << tree1.get<double>("exposure2") << std::endl;
    std::cout << tree1.get<double>("exposure3") << std::endl;
    std::cout << tree1.get<double>("exposure4") << std::endl;
    std::cout << tree1.get<double>("exposure5") << std::endl;
    std::cout << tree1.get<double>("gain1") << std::endl;
    std::cout << tree1.get<double>("gain2") << std::endl;
    std::cout << tree1.get<double>("gain3") << std::endl;
    std::cout << tree1.get<double>("gain4") << std::endl;
    std::cout << tree1.get<double>("gain5") << std::endl;

}
catch (exception& e)
{
    cout << e.what();
}

#include <boost/asio.hpp>
#include <iostream>
using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;

int main(void)
try
{
    asio::io_service io_service;
    
    //Create a TCP socket
    ip::tcp::socket sock(io_service);

    //Name resolution (hostname to IP address conversion)
    //ip::tcp::resolver resolver(io_service);
    //ip::tcp::resolver::query query("www.kumei.ne.jp", "http");

    //Configure Host Information
    //ip::tcp::endpoint endpoint(*resolver.resolve(query));

    // fixed ip connection
    ip::tcp::endpoint endpoint( ip::address::from_string("192.168.10.254"), 80);

    //Connect to Socket
    sock.connect(endpoint);

    //Send Message
    asio::streambuf request;
    ostream request_ostream(&request);
	// get exposure
    //request_ostream << "GET /exposure HTTP/1.0\r\n\r\n";
	// take a photo
	string json_param = "{ 'store_capture' : True, 'block' : True }";
	request_ostream << "POST /capture HTTP/1.0\r\n\r\n";
	request_ostream << "Content-Type: application/x-www-form-urlencoded; charset=UTF-8\r\n";
	request_ostream << "Content-Length: " << json_param.length() << "\r\n";
    request_ostream << "Connection: Close\r\n";
	request_ostream << "\r\n";
	request_ostream << json_param << "\r\n";
    asio::write(sock, request);

    //Receive message
    asio::streambuf buffer;
    boost::system::error_code error;
    asio::read(sock, buffer, asio::transfer_all(), error);
    if (error && error != asio::error::eof)
    {
        std::cout << "receive failed: " << error.message() << std::endl;
    }
    else
    {
        cout << &buffer;
    }
}
catch (exception& e)
{
    cout << e.what();
}

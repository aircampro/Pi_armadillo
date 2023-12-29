//  Example of TCP or UDP communication with Boost API
//  g++-8 -std=c++17 -o  boost_net boost_net.cpp -lboost_log -lpthread  -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread  -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -lboost_system -lstdc++fs -DBOOST_BIND_GLOBAL_PLACEHOLDERS -I/usr/lib/arm-linux-gnueabihf -lboost_filesystem
//
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>

using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;

int sender_udp(string ip, u_short port);
int reader_udp(u_short port);
int client_tcp(string server_name, u_short port);
int server_tcp(u_short port);

int main(void)
{
    char c;
    cout << "tcp boost test start TCP c=client s=server UDP s=sender r=reader c/s\n>";
    cin >> c;
    if (c == 'c')
    {
        client_tcp("127.0.0.1", 14551);
    }
    else if (c == 's')
    {
        server_tcp(14551);
    }
    else if (c == 'w')
    {
        sender_udp("127.0.0.1", 14551);
    }
    else if (c == 'r')
    {
        reader_udp(14551);
    }
	return 0;
}

// example of a UDP continous sender
// 
int sender_udp(string ip, u_short port)
try
{
    using namespace boost::asio::ip;

    //set up udp
    boost::asio::io_service io_service;
    udp::socket sock(io_service, udp::endpoint(udp::v4(), port));
    
    //for ever 
    for(int i=0; true; i++)
    {
         //Send text (move on to the next process as soon as you finish sending it, regardless of the other party's state)
         std::string str = (boost::format("BoostSocketUDP %03d\n") % i).str();
         sock.send_to
         (
            boost::asio::buffer(str),
            udp::endpoint(address::from_string(ip), port)
         );

        //1 second wait (not to send too much)
        std::cout << str;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }

    return 0;
}
catch (exception& e)
{
    cout << e.what();
    return 1;
}

// example of a UDP continous reader
// 
int reader_udp(u_short port)
try
{
    using namespace boost::asio::ip;

    //UDP io service
    boost::asio::io_service io_service;
    udp::socket sock(io_service, udp::endpoint(udp::v4(), port));

    //Receive repeated text
    for(int i=0; true; i++)
    {
        //Receive (waits for it to be received)
        boost::array<char, 128> recv_buf;
        udp::endpoint endpoint;
        size_t len = sock.receive_from(boost::asio::buffer(recv_buf), endpoint);

        //Show received content by writing to standard our
        std::cout.write(recv_buf.data(), len);
    }

    return 0;
}
catch (exception& e)
{
    cout << e.what();
    return 1;
}

// example implementation of boost tcp client
//
int client_tcp(string server_name, u_short port)
try
{
    asio::io_service io_service;

    //Create a TCP socket
    ip::tcp::socket sock(io_service);

    //Get Host Information
    //cout << "server name\n>";
    //string server_name;
    //cin >> server_name;
    //cout << "port number\n>";
    //u_short port = 0;
    //cin >> port;
    sock.connect(ip::tcp::endpoint(ip::address::from_string(server_name), port));
	
    //Send and receive messages
    string buffer;
    while (true)
    {
        cout << "send\n>";
        cin >> buffer;
        asio::write(sock, asio::buffer(buffer));
        if (buffer == "end")
        {
            break;
        }
        cout << "Waiting for a reply from the server\n";

        asio::streambuf receive_buffer;
        boost::system::error_code error;
        asio::read(sock, receive_buffer, asio::transfer_at_least(1), error);
        if (error && error != asio::error::eof)
        {
            std::cout << "receive failed: " << error.message() << std::endl;
        }
        else if (asio::buffer_cast<const char*>(receive_buffer.data()) == string("end"))
        {
            cout << "the server is disconnected.\n";
            break;
        }

        cout << "Receive:" << &receive_buffer << endl;
    }

    return 0;
}
catch (exception& e)
{
    cout << e.what();
    return 1;
}

// example implementation of boost tcp server
//
int server_tcp(u_short port)
try
{
    asio::io_service io_service;

    //Create a TCP socket
    ip::tcp::socket sock(io_service);

    //Configuring IPv4 Socket Address Information
    //cout << "Port Number\n>";
    //u_short port = 0;
    //cin >> port;
	
    ip::tcp::acceptor acceptor(io_service, ip::tcp::endpoint(ip::tcp::v4(), port));

    //Accept connections from clients
    acceptor.accept(sock);

    //receive and send messages
    string buffer;
    while (true)
    {
        cout << "Waiting for reception from the client\n>";
        asio::streambuf receive_buffer;
        boost::system::error_code error;
        asio::read(sock, receive_buffer, asio::transfer_at_least(1), error);
        if (error && error != asio::error::eof)
        {
            std::cout << "receive failed: " << error.message() << std::endl;
        }
        else if (asio::buffer_cast<const char*>(receive_buffer.data()) == string("end"))
        {
            cout << "Client disconnected\n";
            break;
        }

        cout << "Receive:" << &receive_buffer << endl;

        cout << "Send\n>";
        cin >> buffer;
        asio::write(sock, asio::buffer(buffer));
        if (buffer == "end")
        {
            break;
        }
    }
    return 0;
}
catch (exception& e)
{
    cout << e.what();
    return 1;
}

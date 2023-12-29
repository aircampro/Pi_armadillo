#include <boost/asio.hpp>
#include <iostream>
#include <string>
using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;

int client(string server_name, u_short port);
int server(u_short port);

int main()
{
    char c;
    cout << "tcp boost test start c=client s=server c/s\n>";
    cin >> c;
    if (c == 'c')
    {
        client("127.0.0.1",14550);
    }
    else if (c == 's')
    {
        server(5670);
    }
}

// example implementation of boost tcp client
//
int client(string server_name, u_short port)
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
int server(u_short port)
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

    //Send and receive messages
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

// https://qiita.com/YukiMiyatake/items/10bd55bbbc0e93e9db56
//
#include <cstdlib>
#include <iostream>
#include <memory>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>

class client: public std::enable_shared_from_this<client>
{
public:
    client(boost::asio::io_service& io_service,
        boost::asio::ssl::context& context,
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
        : socket_(io_service, context), endpoint_iterator_(endpoint_iterator)
    {
        // constructor shared_from_this
    }

    void run() {

        boost::asio::async_connect(socket_.lowest_layer(), endpoint_iterator_,
            boost::bind(&client::handle_connect, shared_from_this(),
                boost::asio::placeholders::error));
    }

    void handle_connect(const boost::system::error_code& error)
    {
        if (!error)
        {
            socket_.async_handshake(boost::asio::ssl::stream_base::client,
                boost::bind(&client::handle_handshake, shared_from_this(),
                    boost::asio::placeholders::error));
        }
        else
        {
            std::cout << "Connect failed: " << error.message() << "\n";
        }
    }

    void handle_handshake(const boost::system::error_code& error)
    {
        if (!error)
        {
            std::ostream request_stream(&request_);
            {
                request_stream << "GET /"  << " HTTP/1.0\r\n";
                request_stream << "Host: www.google.co.jp" << "\r\n";
                request_stream << "Accept: */*\r\n";

                request_stream << "Connection: close\r\n\r\n";
            }
            boost::asio::async_write(socket_, request_,
                boost::bind(&client::handle_write_request, shared_from_this(),
                    boost::asio::placeholders::error));

        }
        else
        {
            std::cout << "Handshake failed: " << error.message() << "\n";
        }
    }

    void handle_write_request(const boost::system::error_code& err)
    {
        if (!err)
        {
            boost::asio::async_read_until(socket_, response_, "\r\n",
                boost::bind(&client::handle_read_status_line, shared_from_this(),
                    boost::asio::placeholders::error));
        }
        else
        {
            std::cerr << "Error: " << err.message() << "\n";
        }
    }

    void handle_read_status_line(const boost::system::error_code& err)
    {
        if (!err)
        {
            std::istream response_stream(&response_);
            std::string http_version;
            response_stream >> http_version;
            response_stream >> status_code_;
            std::string status_message;
            std::getline(response_stream, status_message);
            if (!response_stream || http_version.substr(0, 5) != "HTTP/")
            {
                std::cerr << "Invalid response\n";
                return;
            }
            if (status_code_ != 200)
            {
                std::cout << "Response returned with status code ";
                std::cout << status_code_ << "\n";
                return;
            }

            // Read the response headers, which are terminated by a blank line.
            boost::asio::async_read_until(socket_, response_, "\r\n\r\n",
                boost::bind(&client::handle_read_headers, shared_from_this(),
                    boost::asio::placeholders::error));
        }
        else
        {
            std::cerr << "Error: " << err << "\n";
        }
    }


    void handle_read_headers(const boost::system::error_code& err)
    {
        if (!err)
        {
            // 
            std::istream response_stream(&response_);
            std::string header;
            while (std::getline(response_stream, header) && header != "\r")
            {
                message_header_ << header << "\n";
            }

            boost::asio::async_read(socket_, response_,
                boost::asio::transfer_at_least(1),
                boost::bind(&client::handle_read_content, shared_from_this(),
                    boost::asio::placeholders::error));
        }
        else
        {
            std::cerr << "Error: " << err << "\n";
        }
    }

    void handle_read_content(const boost::system::error_code& err)
    {
        //      if (err == boost::asio::error::eof)
        if (err)
        {
            auto  self(shared_from_this());

            message_body_ << &response_;


            // print the received message header and body
//          [this, self]() {callback_(*this); }();
            std::cout << status_code_ << std::endl << message_header_.str() << std::endl << message_body_.str() << std::endl;

        }
        else if (!err)
        {
            message_body_ << &response_;

            response_.consume(response_.size());

            boost::asio::async_read(socket_, response_,
                boost::asio::transfer_at_least(1),
                boost::bind(&client::handle_read_content, shared_from_this(),
                    boost::asio::placeholders::error));

        }
    }



private:
    boost::asio::ssl::stream<boost::asio::ip::tcp::socket> socket_;
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator_;
    unsigned int status_code_ = 0;
    std::ostringstream message_header_;
    std::ostringstream message_body_;
    boost::asio::streambuf request_;
    boost::asio::streambuf response_;
};

int main(int argc, char* argv[])
{
    try
    {

        boost::asio::io_service io_service;

        boost::asio::ip::tcp::resolver resolver(io_service);
        boost::asio::ip::tcp::resolver::query query("www.google.co.jp", "https");
        boost::asio::ip::tcp::resolver::iterator iterator = resolver.resolve(query);

        boost::asio::ssl::context ctx(boost::asio::ssl::context::sslv23);

        std::make_shared<client>(io_service, ctx, iterator)->run();

        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
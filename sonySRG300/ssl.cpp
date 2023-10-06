// https://github.com/godai0519/BoostConnect
// https://www.bit-hive.com/articles/20210226
//

#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#define SSL_R_SHORT_READ 219
#include "ssl/ssl_locl.h"
#include <boost/asio/ssl.hpp>

namespace asio = boost::asio;

asio::io_context io_context;
asio::ssl::context ssl_context(asio::ssl::context::tlsv12_client);


/*
 * Factories
 * asio::ssl::stream<asio::ip::tcp::socketcopy move
 * unique_ptr。
 */
using socket_ptr = std::unique_ptr<asio::ip::tcp::socket>;

socket_ptr connect_to(const asio::ip::address& ip, uint16_t port) {
  socket_ptr socket = std::make_unique<asio::ip::tcp::socket>(io_context);
  socket->connect(asio::ip::tcp::endpoint(ip, port));
  return socket;
}

using ssl_stream_ptr = std::unique_ptr<asio::ssl::stream<asio::ip::tcp::socket>>;

ssl_stream_ptr connect_to_with_tls(const asio::ip::address& ip, uint16_t port) {
  ssl_stream_ptr stream = std::make_unique<asio::ssl::stream<asio::ip::tcp::socket>>(io_context, ssl_context);
  stream->lowest_layer().connect(asio::ip::tcp::endpoint(ip, port));
  stream->handshake(asio::ssl::stream_base::client);
  return stream;
}

/*
 * (SyncStream)
 */
template<typename SyncStream>
/*
void http_cmd(SyncStream& stream, const std::string& host, const std::string& path, const std::string& cmd) {
  std::ostringstream request_stream;
  request_stream << cmd << " " << path << " HTTP/1.1\r\n"
                 << "Host: " << host << "\r\n"
                 << "Connection: Close\r\n";
  request_stream << "\r\n";

  asio::write(stream, asio::buffer(request_stream.str()));

  boost::system::error_code error;
  asio::streambuf receive_buffer;

  asio::read(stream, receive_buffer, error);
  if (error && error != asio::error::eof) {
    throw std::runtime_error(error.message());
  }
  std::cout << asio::buffer_cast<const char*>(receive_buffer.data()) << std::endl;
}
*/
void http_cmd(SyncStream& stream, const std::string& host, const std::string& cmd) {
  std::ostringstream request_stream;
  request_stream << cmd 
                 << "Host: " << host << "\r\n"
                 << "Connection: Close\r\n";
  request_stream << "\r\n";

  asio::write(stream, asio::buffer(request_stream.str()));

  boost::system::error_code error;
  asio::streambuf receive_buffer;

  asio::read(stream, receive_buffer, error);
  if (error && error != asio::error::eof) {
    throw std::runtime_error(error.message());
  }
  std::cout << asio::buffer_cast<const char*>(receive_buffer.data()) << std::endl;
}

int main() {
  const std::string host = "localhost";
  asio::ip::tcp::resolver resolver(io_context);
  auto endpoints = resolver.resolve(asio::ip::tcp::v4(), host, "");
  auto ip = endpoints.begin()->endpoint().address();
  {
    // asio::ip::tcp::socket
    socket_ptr s = connect_to(ip, 80);
    //http_cmd(*s, host, "/", "GET");
	std::string req_msg = "GET /url/eqn.cgi?param=value  HTTP/1.0\r\n\r\n";
	http_cmd(*s, host, req_msg);
  }
  {
    // asio::ssl::stream<asio::ip::tcp::socket
    ssl_stream_ptr s = connect_to_with_tls(ip, 443);
    //http_cmd(*s, host, "/", "GET");
	std::string req_msg = "GET /url/eqn.cgi?param=value  HTTP/1.0\r\n\r\n";
	http_cmd(*s, host, req_msg);
  }
  return 0;
}



// https://botan.randombit.net/handbook/building.html
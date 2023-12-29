#include <cstdlib>
#include <string>
#include <memory>
#include <optional>
#include <variant>
#include <utility>
#include <tuple>
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/version.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>

using std::string;
using std::pair;
using std::tuple;
using std::variant;
using std::optional;
using std::shared_ptr;

namespace asio = boost::asio;
namespace beast = boost::beast;
using boost::asio::ip::tcp;
namespace ssl = boost::asio::ssl;
namespace http = boost::beast::http;
using boost::system::error_code;

template<class CompletionToken, class ReturnType>
using HandlerType = typename asio::handler_type<CompletionToken, void(ReturnType)>::type;

template<class CompletionToken, class ReturnType>
using AsyncResult = typename asio::async_result<HandlerType<CompletionToken, ReturnType>>::type;

template<class CompletionToken>
auto httpRequest(
		  const shared_ptr<asio::io_service> ios,
		    const string host,
		      const http::request<http::string_body> req,
		        CompletionToken&& token
		)-> AsyncResult<CompletionToken, variant<string, http::response<http::string_body>>> {
	  using ret_t = variant<string, http::response<http::string_body>>;
	    using handler_t = HandlerType<CompletionToken, ret_t>;
	      auto handler = handler_t{std::forward<CompletionToken>(token)};
	        auto result = asio::async_result<handler_t>{handler};
		  asio::spawn(*ios, [=](auto yield) mutable {
				      auto ec = error_code{};
				          auto query  = tcp::resolver::query{host, "http"};
					      auto lookup = tcp::resolver{*ios}.async_resolve(query, yield[ec]);
					          if(ec != 0){ return handler(ec, ret_t{"lookup error"}); }
						      auto socket = tcp::socket{*ios};
						          asio::async_connect(socket, lookup, yield[ec]);
							      if(ec != 0){ return handler(ret_t{"connect error"}); }
							          http::async_write(socket, const_cast<http::request<http::string_body>&>(req), yield[ec]);
								      if(ec != 0){ return handler(ret_t{"write error"}); }
								          auto buffer = beast::flat_buffer{};
									      auto res = http::response<http::string_body>{};
									          http::async_read(socket, buffer, res, yield[ec]);
										      if(ec != 0){ return handler(ret_t{"read error"}); }
										          socket.shutdown(tcp::socket::shutdown_both, ec);
											      if(ec != 0){ return handler(ret_t{"shutdown error"}); }
											          handler(ret_t{res});
												    });
		    return result.get();
}

template<class CompletionToken>
auto httpsRequest(
		  const shared_ptr<asio::io_service> ios,
		    const string host,
		      const http::request<http::string_body> req,
		        CompletionToken&& token
		)-> AsyncResult<CompletionToken, variant<string, http::response<http::string_body>>> {
	  using ret_t = variant<string, http::response<http::string_body>>;
	    using handler_t = HandlerType<CompletionToken, ret_t>;
	      auto handler = handler_t{std::forward<CompletionToken>(token)};
	        auto result = asio::async_result<handler_t>{handler};
		  asio::spawn(*ios, [=](auto yield) mutable {
				      auto ec = error_code{};

				          auto query = tcp::resolver::query{host, "https"};
					      auto lookup = tcp::resolver{*ios}.async_resolve(query, yield[ec]);
					          std::cout << "dns lookup:" << ec << std::endl;
						      if(ec != 0){ return handler(ec, ret_t{"lookup error"}); }

						          auto ctx = ssl::context{ssl::context::sslv23};
							      auto ssl_socket = ssl::stream<tcp::socket>{*ios, ctx};
							          asio::async_connect(ssl_socket.lowest_layer(), lookup, yield[ec]);
								      std::cout << "tcp connect:" << ec << std::endl;
								          if(ec != 0){ return handler(ec, ret_t{"connection error"}); }

									      ssl_socket.async_handshake(ssl::stream_base::client, yield[ec]);
									          std::cout << "ssl handshake:" << ec << std::endl;
										      if(ec != 0){ return handler(ec, ret_t{"handshake error"}); }

										          http::async_write(ssl_socket, const_cast<http::request<http::string_body>&>(req), yield[ec]);
											      std::cout << "http write:" << ec << std::endl;

											          auto res = http::response<http::string_body>{};
												      auto buffer = beast::flat_buffer{};
												          http::async_read(ssl_socket, buffer, res, yield[ec]);
													      std::cout << "http read:" << ec << std::endl;

													          ssl_socket.lowest_layer().cancel(ec);
														      std::cout << "tcp cancel:" << ec << std::endl;

														          ssl_socket.async_shutdown(yield[ec]);
															      std::cout << "ssl shutdown:" << ec << std::endl;

															          ssl_socket.lowest_layer().shutdown(tcp::socket::shutdown_both, ec);
																      std::cout << "tcp shutdown:" << ec << std::endl;

																          ssl_socket.lowest_layer().close(ec);
																	      std::cout << "tcp close:" << ec << std::endl;

																	          return handler(ret_t{res});
																		    });
		    return result.get();
}

auto main(int argc, char* argv[])-> int {
	  auto ios = std::make_shared<boost::asio::io_service>();
	    boost::asio::spawn(*ios, [=](auto yield) mutable {
			        auto ec = boost::system::error_code{};
				    auto host = "google.com";
				        auto req = http::request<http::string_body>{http::verb::get, "/", 11};
					    {
					          auto ret = httpRequest(ios, host, req, yield[ec]);
						        if(auto res_ptr = std::get_if<http::response<http::string_body>>(&ret)){
							        std::cout << *res_ptr << std::endl;
								      }else if(auto err_ptr = std::get_if<std::string>(&ret)){
								              std::cout << *err_ptr << std::endl;
									            }
										        }
											    {
											          auto ret = httpsRequest(ios, host, req, yield[ec]);
												        if(auto res_ptr = std::get_if<http::response<http::string_body>>(&ret)){
													        std::cout << *res_ptr << std::endl;
														      }else if(auto err_ptr = std::get_if<std::string>(&ret)){
														              std::cout << *err_ptr << std::endl;
															            }
																        }
																	  });
	      ios->run();
	        std::cout << "end" << std::endl;
		  return EXIT_SUCCESS;
}

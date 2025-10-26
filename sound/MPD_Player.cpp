// client for MPD (music player daemon) https://www.musicpd.org/
// for cxxopts use https://github.com/TadaoYamaoka/cxxopts/blob/master/include/cxxopts.hpp
//
// can be used to play music, robot speak, location information system (bus/train)
// compiles as :- g++ MPD_Player.cpp -l boost_system -l pthread
#include <stdio.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <unistd.h>
// we use cxxopts instead #include <boost/program_options.hpp>
#include "cxxopts.hpp"

int main(int argc, char **argv) {

	cxxopts::Options options("mpd_player");
	try {
		std::string song;

		options.add_options()
			("song", "Song name", cxxopts::value<std::string>(song))
			("ipa", "ip address", cxxopts::value<std::string>())
			("d,debug", "Enable debugging")
			("h,help", "Print help")
			;
		options.parse_positional({ "song", "ipa" });

		auto result = options.parse(argc, argv);

		if (result.count("help"))
		{
			std::cout << options.help({}) << std::endl;
			return 0;
		}

		std::cout << "chosen song "+song << std::endl;
		std::string ipa = result["ipa"].as<std::string>();
		std::cout << "ip "+ipa << std::endl;
	}
	catch (cxxopts::OptionException &e) {
		std::cout << options.usage() << std::endl;
		std::cerr << e.what() << std::endl;
	}
	
  boost::asio::io_service io;
  boost::asio::ip::tcp::socket sock(io);

  boost::asio::ip::tcp::endpoint endpoint = boost::asio::ip::tcp::endpoint{
    boost::asio::ip::address::from_string(ipa),           // IPv4 address
    6600                                                  // port
  };

  sock.connect(endpoint);

  boost::asio::streambuf request;
  std::ostream request_ostream(&request);
  request_ostream << song+"\n";                    // Request information about the currently playing music in MPD

  boost::system::error_code error;

  // Write data to MPD request song from MPD
  boost::asio::write(sock, request);

  boost::asio::streambuf buffer;

  // If the last line of the data sent by MPD is successful, it will read "OK\n"
  boost::asio::read_until(sock,buffer,"OK\n");
  if(error && error != boost::asio::error::eof){
    std::cout << "receive failed" << std::endl;
  } else {
    std::cout << &buffer;
  }

  return 0;

}


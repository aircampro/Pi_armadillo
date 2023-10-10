//   Copyright 2012 Kevin Godden
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

//
// blocking_reader.h - a class that provides basic support for
// blocking & time-outable single character reads from
// boost::asio::serial_port.
//
// use like this:
//
// 	blocking_reader reader(port, 500);
//
//	char c;
//
//	if (!reader.read_char(c))
//		return false;
//
// Kevin Godden, www.ridgesolutions.ie
//
// https://www.ridgesolutions.ie/index.php/2012/12/13/boost-c-read-from-serial-port-with-timeout-example/
// https://github.com/kgodden/blocking_reader
//

//#pragma once
#ifndef __serial_block_reader_
#define __serial_block_reader_

#include <iostream>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <boost/log/expressions/filter.hpp>


// use an any type template
//
template<typename Target, typename Source>
inline boost::optional<Target> lexical_cast_opt(Source const& src)
{
	try
	{
		return boost::lexical_cast<Target>(src);
	}
	catch (boost::bad_lexical_cast& e)
	{
		std::cout << src << "(error): " << e.what() << "\n";
		return boost::none;
	}
}

class blocking_reader
{
	boost::asio::serial_port& port;
	size_t timeout;
	char c;
	boost::asio::deadline_timer timer;
	boost::optional<int> read_error;
	boost::asio::io_service iosev;

	// Called when an async read completes or has been cancelled
	void read_complete(const boost::system::error_code& error,
		size_t bytes_transferred) {

		read_error = lexical_cast_opt<int>(error);

		// Read has finished, so cancel the
		// timer.
		timer.cancel();
	}

	// Called when the timer's deadline expires.
	void time_out(const boost::system::error_code& error) {

		// Was the timeout was cancelled?
		if (error) {
			// yes
			return;
		}

		// no, we have timed out, so kill
		// the read operation
		// The read callback will be called
		// with an error
		port.cancel();
	}

public:

	// Constructs a blocking reader, pass in an open serial_port and
	// a timeout in milliseconds.
	blocking_reader(boost::asio::serial_port& port, size_t timeout, boost::asio::io_service& iosev) :
		port(port), timeout(timeout),
		timer(iosev),
		//iosev(iosev),
		read_error(true) {

	}

	// Reads a character or times out
	// returns false if the read times out
	bool read_char(char& val) {

		val = c = '\0';

		// After a timeout & cancel it seems we need
		// to do a reset for subsequent reads to work.
		//port.get_io_service().reset();
		iosev.reset();

		// Asynchronously read 1 character.
		boost::asio::async_read(port, boost::asio::buffer(&c, 1),
			boost::bind(&blocking_reader::read_complete,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

		// Setup a deadline time to implement our timeout.
		timer.expires_from_now(boost::posix_time::milliseconds(timeout));
		timer.async_wait(boost::bind(&blocking_reader::time_out,
			this, boost::asio::placeholders::error));

		// This will block until a character is read
		// or until the it is cancelled.
		//port.get_io_service().run();
		iosev.run();

		if (!read_error)
			val = c;

		return !read_error;
	}
};

#endif
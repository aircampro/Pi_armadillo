// Ask question on GPT-3 chatbot and reply via the pepper robot text to speach engine, it will also move its head after speaking 
//
// usage : gpt3_client 10.1.1.32 "what is your name?"

//CMakeLists.txt:
//cmake_minimum_required(VERSION 3.9)
//set(CMAKE_BUILD_TYPE Release CACHE STRING "None Debug Release ...")
//project(websocket CXX)
//set(CMAKE_CXX_STANDARD 14)
//set(CMAKE_CXX_STANDARD_REQUIRED ON)
//set(CMAKE_C_STANDARD 99)
//set(CMAKE_C_STANDARD_REQUIRED ON)
//find_package(Boost 1.71.0 REQUIRED COMPONENTS
//    date_time coroutine date_time filesystem graph log_setup program_options random regex serialization system thread timer
//)
//find_package(qibuild)
//include_directories(
//   ${Boost_INCLUDE_DIRS}
//)
//link_directories(
//    ${Boost_LIBRARY_DIRS}
//)
//add_executable (gpt3_client
//    speak_robot.cpp
//)
//# Create an executable named gpt3_client,
//# with the source file : speak_robot.cpp
//qi_create_bin(gpt3_client speak_robot.cpp)

//# Tell CMake that exe depends on ALCOMMON and ALPROXIES
//# This will set the libraries to link gpt3_client with,
//# the include paths, and so on
//qi_use_lib(gpt3_client ALCOMMON ALPROXIES BOOST)

//Linux_gcc.cmake:
//set(CMAKE_C_COMPILER gcc CACHE STRING "GCC compiler" FORCE)
//set(CMAKE_CXX_COMPILER g++ CACHE STRING "G++ compiler" FORCE)
//set(CMAKE_CXX_FLAGS "-std=c++14" CACHE STRING "G++ CXX_FLAGS" FORCE)
//set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g" CACHE STRING "G++ CXX_FLAGS_DEBUG" FORCE)
//set(CMAKE_CXX_FLAGS_RELEASE "-fopenmp -O3" CACHE STRING "G++ CXX_FLAGS_RELEASE" FORCE)
//set(CMAKE_EXE_LINKER_FLAGS "-Ofast -lpthread" CACHE STRING "GCC LD_FLAGS" FORCE)

//mkdir Linux_gcc
//cd Linux_gcc
//cmake -C ../Linux_gcc.cmake ..
//make gpt3_client 
//./gpt3_client 10.249.229.225

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/error.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <iostream>
#include <string>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <openssl/ssl.h>
#include <fstream>

#include <alerror/alerror.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace ssl = boost::asio::ssl;
using json = nlohmann::json;

// Read OpenAI APIKey
std::string readApiKey() {
    std::ifstream file("OpenAI_Key.txt");

    if (file.is_open()) {
        std::string line;

        while (std::getline(file, line)) {
            //std::cout << line << '\n';
        }

        file.close();
        return line;
    } else {
        std::cerr << "Unable to open file\n";
        return "error";
    }
}

// create the request body to be sent to GPT-3
std::string create_request_message(std::string msg2gpt) {
    // Body of the json request to GPT
    nlohmann::json jsonBody;
    jsonBody["model"] = "gpt-3.5-turbo";
    jsonBody["messages"] = {
        {{"role", "user"}, {"content", msg2gpt}}
    };
    jsonBody["temperature"] = 0.7;

    // JSON result returned
    return jsonBody.dump();
}

// function to say something on aldebaran pepper robot
//
void pepperSayText(const std::string& robotIp, const std::string& phraseToSay)
{;
  try
  {
    /** Create an ALTextToSpeechProxy so that we can call the say method
    * Arguments for the constructor are:
    *  - IP of the robot
    *  - port on which NAOqi is listening. Default is 9559
    */
    AL::ALTextToSpeechProxy tts(robotIp, 9559);

    /** Call the say method */
    tts.say(phraseToSay);
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(1);
  }
}

// Move pepper head, we use here a specialized proxy to ALMotion
//
void pepperMoveHead(const std::string& robotIp, AL::ALValue move_time)
{
  /** The name of the joint to be moved. */
  const AL::ALValue jointName = "HeadYaw";

  try {
    /** Create a ALMotionProxy to call the methods to move NAO's head.
    * Arguments for the constructor are:
    * - IP adress of the robot
    * - port on which NAOqi is listening, by default 9559
    */
    AL::ALMotionProxy motion(robotIp, 9559);

    /** Make sure the head is stiff to be able to move it.
    * To do so, make the stiffness go to the maximum in one second.
    */
    /** Target stiffness. */
    AL::ALValue stiffness = 1.0f;
    /** Time (in seconds) to reach the target. */
    AL::ALValue time = move_time;
    /** Call the stiffness interpolation method. */
    motion.stiffnessInterpolation(jointName, stiffness, time);

    /** Set the target angle list, in radians. */
    AL::ALValue targetAngles = AL::ALValue::array(-1.5f, 1.5f, 0.4f);
    /** Set the corresponding time lists, in seconds. */
    AL::ALValue targetTimes = AL::ALValue::array(2.0f, 6.4f, 9.7f);
    /** Specify that the desired angles are absolute. */
    bool isAbsolute = true;

    /** Call the angle interpolation method. The joint will reach the
    * desired angles at the desired times.
    */
    motion.angleInterpolation(jointName, targetAngles, targetTimes, isAbsolute);

    /** Remove the stiffness on the head. */
    stiffness = 0.0f;
    time = move_time;
    motion.stiffnessInterpolation(jointName, stiffness, time);

  }
  catch (const AL::ALError& e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(2);
  }
}

int main(int argc, char* argv[]) {

    // parse the argument of robot ip, message to chatbot in quotes
    if(argc != 3)
    {
      std::cerr << "Wrong number of arguments!" << std::endl;
      std::cerr << "Usage: speak_robot NAO_IP " << "\" your message \""  << std::endl;
      exit(2);
    }
  
	// read the aldebaran pepper robot ip from the first argument
	const std::string robotIp(argv[1]);  
    std::string your_message = argv[2];
	
    try {
        // OpenAI APIKey
        std::string apiKey = readApiKey();
        if (apiKey == "error") {
            return 0;
        }

        // create JSON request body
        std::string body = create_request_message(your_message);

        // Create asio::io_context object
        asio::io_context io_context;

        // Create SSL context
        ssl::context ssl_context(ssl::context::tlsv13_client);

        // Set option to disable certificate validation in SSL context
        ssl_context.set_verify_mode(ssl::verify_none);

        // Set the host and port for HTTP requests
        std::string host = "api.openai.com";
        std::string port = "443";
        std::string target = "/v1/chat/completions";

        // create asio::ip::tcp::resolver object and resolve hostname
        asio::ip::tcp::resolver resolver(io_context);
        asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(host, "https");

        // create SSL stream and connect to server
        ssl::stream<asio::ip::tcp::socket> stream(io_context, ssl_context);
        // Set SNI Hostname (many hosts need this to handshake successfully)
        // SSL_set_tlsext_host_name(..., host.c_str())
        if(!SSL_set_tlsext_host_name(stream.native_handle(), host.c_str()))
        {
            boost::system::error_code ec{static_cast<int>(::ERR_get_error()), boost::asio::error::get_ssl_category()};
            throw boost::system::system_error{ec};
        }

        asio::connect(stream.next_layer(), endpoints);
        stream.handshake(ssl::stream_base::client);

        // Create HTTP request
        http::request<http::string_body> request;
        request.method(http::verb::post);
        request.target(target);
        request.version(11);  // HTTP/1.1
        request.set(http::field::host, host);
        request.set(http::field::content_type, "application/json");
        request.set(http::field::authorization, "Bearer " + apiKey);
        request.body() = body;
        request.prepare_payload();
        request.keep_alive(true);

        // write HTTP stream with that request message
        http::write(stream, request);
		
		// set the robot motion time
		AL::ALValue move_time = 2.0f;
						
        // HTTP read response from GPT webserver
        beast::flat_buffer buffer;
        http::response<http::dynamic_body> response;
        http::async_read(stream, buffer, response, 
            [&](beast::error_code ec, std::size_t bytes_transferred) {
                if (!ec) {
                    // HTTP response code
                    std::cout << "Response code: " << response.result_int() << std::endl;
                    //std::cout << "Response body: " << beast::buffers_to_string(response.body().data()) << std::endl;
                    std::string response_data = beast::buffers_to_string(response.body().data());

                    // JSON parse
                    try {
                        json data = json::parse(response_data);
                        // Chat-GPT reply
                        std::cout << "Response of Chat-GPT: " << data["choices"][0]["message"]["content"] << std::endl;
						// say the result on the pepper robot and make a movement of its head
						pepperSayText(robotIp, data["choices"][0]["message"]["content"]);
						pepperMoveHead(robotIp, move_time);
                    } catch (const json::exception& e) {
                        std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
				        pepperSayText(robotIp, e.what());						
                    }

                } else {
                    // error response
                    std::cerr << "Error: " << ec.message() << std::endl;
				    pepperSayText(robotIp, ec.message());
                }
            });

        // run resolver object
        io_context.run();

        // shutdown the stream
        //stream.shutdown();
        stream.next_layer().close();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
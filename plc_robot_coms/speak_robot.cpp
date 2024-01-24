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

// for the GPT3 client requests
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

// for the aldebaran proxy to the robot
#include <alerror/alerror.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almotionproxy.h>

// for the random number generator used to give random pose
#include <boost/random.hpp>
#include <boost/nondet_random.hpp> //for random_device
#include <boost/random/triangle_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <map>
#include <vector>
#include <tuple>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace ssl = boost::asio::ssl;
using json = nlohmann::json;

// default listening port for NAOqi to listen
#define NAOqi_PORT 9559

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

// function to say something on aldebaran nao/pepper robot
//
void naoSayText(const std::string& robotIp, const std::string& phraseToSay)
{;
  try
  {
    /** Create an ALTextToSpeechProxy so that we can call the say method
    * Arguments for the constructor are:
    *  - IP of the robot
    *  - port on which NAOqi is listening. Default is 9559
    */
    AL::ALTextToSpeechProxy tts(robotIp, NAOqi_PORT);

    /** Call the say method */
    tts.say(phraseToSay);
  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    exit(1);
  }
}

// Move NAO / pepper head, we use here a specialized proxy to ALMotion
//
void naoMoveHead(const std::string& robotIp, AL::ALValue move_time, double random_selection)
{
   // create a map containing the joint names and pose selection you want the robot to have in memory first_param=random number value for selection
   std::map<double, std::string> activeFunctions;	
   activeFunctions.insert(std::make_pair(3.5, "HeadYaw"));              // first joint and movement sequence can be defined here
   activeFunctions.insert(std::make_pair(5.0, "HeadPitch"));            // second joint and movement sequence can be defined here
   activeFunctions.insert(std::make_pair(5.5, "HeadYaw"));              // third joint and movement sequence can be defined here
   activeFunctions.insert(std::make_pair(5.6, "HeadPitch"));            // fourth joint and movement sequence can be defined here
                                 
   std::vector<std::tuple<float, float, float>> activePoseAngles;
   activePoseAngles.push_back(std::make_tuple(-1.5f, 1.5f, 0.4f));           // first z,y,z pose angles
   activePoseAngles.push_back(std::make_tuple(-1.2f, 1.7f, 0.4f));
   activePoseAngles.push_back(std::make_tuple(-1.4f, 1.6f, 0.4f));
   activePoseAngles.push_back(std::make_tuple(1.5f, -1.6f, 0.9f));           // fourth z,y,z pose angles

   std::vector<std::tuple<float, float, float>> activeTargetTimes;
   activeTargetTimes.push_back(std::make_tuple(2.0f, 6.4f, 9.7f));           // first target times
   activeTargetTimes.push_back(std::make_tuple(2.0f, 6.4f, 9.7f));
   activeTargetTimes.push_back(std::make_tuple(2.0f, 7.4f, 6.7f));
   activeTargetTimes.push_back(std::make_tuple(9.0f, 4.7f, 1.7f));           // fourth target times 
   
  /** Default name of the joint to be moved. angles and target times of motion */
   AL::ALValue jointName = "HeadYaw";  
   float target_angle_x = -1.5f;                                            // target angles
   float target_angle_y = 1.5f;
   float target_angle_z = 0.4f; 
   float tt1 = 2.0f;                                                        // target times
   float tt2 = 6.4f;
   float tt3 = 9.7f; 
   int pose_id = 0;   
   for (auto& item: activeFunctions)                                       // select the joint acording to the activeFuncrions map and random number passed 
   {
	if (item.first < random_selection) {
           jointName = item.second;                                       // select the joint from the random number generated
	   break;
        } else { pose_id++; }	   		
   }
   if (pose_id < activePoseAngles.size() ) {                              // select the pose for that random number passed
       std::tie(target_angle_x, target_angle_y, target_angle_z) = activePoseAngles[pose_id]; 
   }
   // if you want to bias the second table by the random number for further variations uncomment below
   // int ii = static_cast<int>(random_selection); 
   // int ii = static_cast<int>(r());                 <--- might even be better to use another random generated  
   // pose_id = (pose_id + ii) % activeTargetTimes.size();
   if (pose_id < activeTargetTimes.size() ) {                            // select the target times for the action relating to random number passed
       std::tie(tt1, tt2, tt3) = activeTargetTimes[pose_id]; 
   }
   
   try {
    /** Create a ALMotionProxy to call the methods to move NAO's head.
    * Arguments for the constructor are:
    * - IP adress of the robot
    * - port on which NAOqi is listening, by default 9559
    */
    AL::ALMotionProxy motion(robotIp, NAOqi_PORT);

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
    AL::ALValue targetAngles = AL::ALValue::array(target_angle_x, target_angle_y, target_angle_z);
    /** Set the corresponding time lists, in seconds. */
    AL::ALValue targetTimes = AL::ALValue::array(tt1, tt2, tt3);
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

    // create a random number generator with boost to give the robot a random pose each time it speaks
	
    // create the seed
    boost::random_device myseed;

    // generate the random  using the marsenne twister algo
    boost::mt19937 gen(static_cast<unsigned>(myseed());

    // you can change for example to kreutzer
	// include <boost/random/shuffle_order.hpp>
    // boost::kreutzer1986 gen(static_cast<unsigned>(myseed());

    // you can change for example to ecuyer1988
    // include // In header: <boost/random/additive_combine.hpp>
    // boost::ecuyer1988 gen(static_cast<unsigned>(myseed());

    // you can change for example to taus88
    // include // In header: <boost/random/taus88.hpp>
    // boost::taus88 gen(static_cast<unsigned>(myseed());
	
    // Use Triangular distribution of minimum value a, center value b, and maximum value c e.g. dst(a, b, c);
    boost::triangle_distribution<> dst(0.0f, 2.0f, 6.0f);

    // alternative for example random normal distribution
    // include // In header: #include <boost/random/normal_distribution.hpp>
	// typedef boost::random::normal_distribution<double> MyDistribution;
	// double mean = 3.0;
	// double sigma = 1.1;
    // MyDistribution dst(mean, sigma);

    // we choose marsenne twister with triangle_distribution
    boost::variate_generator<boost::mt19937, boost::triangle_distribution<> > r(gen, dst);
	
	// alternative example im using taus88 with normal distribution
    // boost::variate_generator<boost::taus88, boost::normal_distribution<> > r(gen, dst);
	
    // now just select the nth generated number as our out pose - (if you iterated the code in sequences you would call r()/this in the while loop)
    int n=5;
    double random_pose = 0.0;
    for (int i = 0; i < n; i++) {
		random_pose = r();
    }		
	
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
						naoSayText(robotIp, data["choices"][0]["message"]["content"]);						
						naoMoveHead(robotIp, move_time, random_pose);
                    } catch (const json::exception& e) {
                        std::cerr << "Failed to parse JSON: " << e.what() << std::endl;
				        naoSayText(robotIp, e.what());						
                    }

                } else {
                    // error response
                    std::cerr << "Error: " << ec.message() << std::endl;
				    naoSayText(robotIp, ec.message());
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

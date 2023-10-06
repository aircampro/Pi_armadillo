/* ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••

   Example program to attempt control and monitoring of Sony SRG300/301-SE Camera(s)
   It should allow the modification of the following options in the above camera :-
   
   1) turn on/off the power led as a test
   led_conf/system.cgi?PowerLed=on/off
   
   2) returns the serial number and model of the camera
   sysinfo/inquiry.cgi?inq=system
   
   3) turns authentication off
   system_authen/system.cg1?CgiAuthen=off
   
   4) return capability of camera attached
   capability/inquiry.cgi?inq=system
   
   Compile as :-
   g++-8 -std=c++17 -o sony_srg300 sony_srg300.cpp -lboost_program_options -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system 
   -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer 
   -DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-deprecated -Wall -O2 -lssl -lcrypto 
   
   g++-8 -std=c++17 -o sony_srg300 sony_srg300.cpp -lboost_program_options -lboost_log -lpthread -DBOOST_LOG_DYN_LINK -lboost_system -lboost_regex -lboost_thread -DBOOST_ALL_DYN_LINK -lboost_log_setup -lboost_timer -DBOOST_BIND_GLOBAL_PLACEHOLDERS -Wno-deprecated  -Wall -O2 -lssl -lcrypto 
 
   example turning on LED 
   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 1
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : POST /led_conf/system.cgi?PowerLed=off HTTP/1.0

   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 1 -v 1
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : POST /led_conf/system.cgi?PowerLed=on HTTP/1.0
 
   example doing position command
 
   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 11 -1 "stop" -2 "pantilt"
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : POST /position/ptzf.cgi?Move=stop,pantilt,Image1, HTTP/1.0

   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 11 -1 "left" -2 "5"
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : POST /position/ptzf.cgi?Move=left,5,Image1, HTTP/1.0
   
   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 6 -s "ExposureBrightLevel" -v 3
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : /exposure/imaging.cgi?ExposureBrightLevel = 3 HTTP/1.0


   anthony@Ubunt-18-dev1:~/boost_test/boost$ ./sony_srg300 -n 6 -s "BacklightCompensationMode" -x "off"
   •••••• connecting to SONY SRG300/1-SE camera ••••••
   command : /exposure/imaging.cgi?BacklightCompensationMode = off HTTP/1.0
   
 ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••• */
   
#include <iostream>
#include <string>
#include <strstream>
// sending & reading http responses
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/regex.hpp>
using namespace std;
namespace asio = boost::asio;
namespace ip = asio::ip;
namespace pt = boost::property_tree;

#define YOUR_CAM_IP_ADDR "192.168.10.254"
#define DEFAULT_VAL_NOT_SET -99

#include <regex>
#include <list>
#include <vector>
#include <map>

#include <sstream>

// command line parser
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// if you want to use boost to split the strings insted of the split_naive function
// use alternative boost for splitting HTML data
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

// if you want SSL / TLS
// https://www.rfc-editor.org/rfc/rfc2818
//
#include <boost/asio/ssl.hpp>

//                     optimization
// ref :- https://www.ir.isas.jaxa.jp/~cyamauch/sli/sfitsio2013-04_hsc/#SIMD_INTRO
//        https://www.ir.isas.jaxa.jp/~cyamauch/sli/adc2013cc/#STRING_SNPRINTF
//        https://www.ir.isas.jaxa.jp/~cyamauch/sli/adc2013cc/
//
// 64bit gcc -Wall -O2 32bit -msse2
#include <xmmintrin.h>    /* SSE */
#include <emmintrin.h>    /* SSE2 */
#include <immintrin.h>    /* AVX */

#define MY_HOST_NAME "localhost"                              // the server host name in /etc/hosts you need to change this to your camera

typedef enum
{
	HTTP_ONLY,
	HTTPS_TLS12,
	NumberOfSConnectionMethods
} connection_method_e;

connection_method_e g_opertation_type = HTTP_ONLY;

//
// =======================> Here define this TAG if you want to use a version which handles TLS 1.2 <================================
//
//#define TLS_SSH_VERSION                                    // uncomment this to use TLS1.2
#if defined(TLS_SSH_VERSION)
// refer to these links
//
// ref :- https://github.com/godai0519/BoostConnect
//        https://www.bit-hive.com/articles/20210226
//
#include <cstdint>
#include <memory>
asio::io_context io_context;
asio::ssl::context ssl_context(asio::ssl::context::tlsv12_client);

//asio::ssl::context ssl_context(boost::asio::ssl::context::sslv23);
//ssl_context.load_verify_file("rootca.crt");

/*
 * Factories
 * asio::ssl::stream<asio::ip::tcp::socket>copymove
 * unique_ptr -- becasue we cant return
 */
using socket_ptr = std::unique_ptr<asio::ip::tcp::socket>;

#endif                                                                // <<<<---------------------- End TLS 1.2 Version Include 

typedef enum
{
	PwrLED,
	SysInfo,
	Auth,
	Capab,
	PTZF,
	Expos,
	WhiteBal,
	Ipv4Net,
	absZoom,
	contZoom,
	Position,
	Audio,
	SetUp,
	Stabiliz,
	Picture,
	AdaptRateCont,
	Image,
	iSet,
	datTim,
	nTp,
	dot1x_config,
	NumberOfSRG300Methods
} SonySRG300_methods_e;

// list the parameter names for a given command
//
std::list<std::string> listOfExposureParams = { "BacklightCompensationLevel","BacklightCompensationMode","ExposureBrightLevel","ExposureCompensation","ExposureExposureTime","ExposureGain","ExposureIris","ExposureMaxExposureTime","ExposureMaxGain","ExposureMaxIris","ExposureMinExposureTime","ExposureMinGain","ExposureMinIris","ExposureMode","ExposurePriority","HighSensitivityMode","ExposureWindow","WideDynamicRangeViewDR" };
std::list<std::string> listOfExposureStrings = { "auto", "shutter", "iris", "manual", "bright", "frame_rate", "low_noise", "on", "off"};
std::list<std::string> listOfWhiteBalParams = { "WhiteBalanceCbGain","WhiteBalanceCrGain","WhiteBalanceOffsetR","WhiteBalanceMode" };
std::list<std::string> listOfWhiteBalStrings = { "auto","atw","indoor","outdoor", "onepushwb", "manual" };
std::list<std::string> listOfPictureParams = { "HighResolutionMode", "NoiseReduction", "Sharpness"  };
std::list<std::string> listOfArcParams = { "AutoRateCtrl", "AutoRateCtrlBitrateMax", "AutoRateCtrlBitrateMax"  };
std::list<std::string> listOfImageParams = { "BitRate", "CBR", "FrameRate", "H264FrameSkip", "H264Profile", "H264Quality", "IFrameInterval", "ImageCodec", "ImageSize", "JpBandWidth", "JpegQuality", "VBRMode", "VBRBitrateMax" };
std::list<std::string> listOfDatTimParams = { "DateFormat", "DstMode", "GmTime", "Time", "ManualTimeZone", "TimeZone" };
std::list<std::string> listOfNtpParams = { "NtpAuto", "NtpInterval", "NtpServer", "NtpService" };
std::list<std::string> listOfdot1XParams = { "Dot1XEapMethod", "Dot1XIdentity", "Dot1XPassword", "Dot1XPrivPassword" };
std::list<std::string> listOfArcStrings = { "on", "off" };
std::list<std::string> listOfPictureStrings = { "on", "off" };
std::list<std::string> listOfNetIpv4Params = { "Dhcp","Ip","Subnetmask","Gateway","Mtu" };  
std::list<std::string> listOfAudioParams = { "AudInCodec","AudioEqualizer","AudioIn","AudioInVolume","AutoLevelControl", "MicLineSelect"  }; 
// each allowable string parameter value is linked by a map to parameter index number in list above they are checked for vailidity when the option is active
std::map<std::string, std::uint16_t> mapOfAudioParamVals{
    {"aac16_64", 1},
    {"aac48_128", 1},
    {"off", 2},
    {"1", 2},
    {"2", 2},
    {"on", 3},
    {"off", 3},
    {"-10", 4},
    {"-9", 4},	
    {"-8", 4},
    {"-7", 4},	
    {"-6", 4},
    {"-5", 4},	
    {"-4", 4},
    {"-3", 4},	
    {"-2", 4},	
    {"-1", 4},	
    {"0", 4},
    {"10", 4},
    {"9", 4},	
    {"8", 4},
    {"7", 4},	
    {"6", 4},
    {"5", 4},	
    {"4", 4},
    {"3", 4},	
    {"2", 4},	
    {"1", 4},	
    {"on", 5},
    {"off", 5},
    {"mic", 6},
    {"line", 6}	
}; 
std::list<std::string> listOfSetUpParams = { "Eflip","HdSdiFormat","InformationDisplay","VideoStd","WideDynamicRangeLevel"  }; 
std::map<std::string, std::uint16_t> mapOfSetUpParamVals{
    {"on", 1},
    {"off", 1},
    {"1080p_5994_ModeA", 2},
    {"1080p_5994_ModeB", 2},
    {"1080i_5994", 2},
    {"1080p_2997", 2},
    {"1080p_50_ModeA", 2},
    {"1080p_50_ModeB", 2},
    {"1080i_50", 2},
    {"1080p_25", 2},
    {"720p_50", 2},
    {"720p_25", 2},
    {"on", 3},
    {"off", 3},
    {"ntsc", 4},
    {"pal", 4},
    {"level1", 5},	
    {"level2", 5},	
    {"level3", 5}	
}; 
std::map<std::string, std::uint16_t> mapOfImageParamVals{
    {"on", 2},
    {"off", 2},
    {"1", 3},
    {"2", 3},
    {"3", 3},
    {"4", 3},
    {"5", 3},
    {"6", 3},
    {"8", 3},
    {"10", 3},
    {"12", 3},
    {"15", 3},
    {"16", 3},
    {"20", 3},
    {"25", 3},
    {"30", 3},
    {"50", 3},	
    {"60", 3},	
    {"on", 4},	
    {"off", 4},
    {"high", 5},
    {"main", 5},
    {"baseline", 5},
    {"1", 6},	
    {"10", 6},	
    {"1", 7},	
    {"5", 7},	
    {"h264", 8},
    {"jpeg", 8},
    {"off", 8},	
    {"0", 10},
    {"500", 10},
    {"8000", 10},	
    {"3200", 10},	
    {"standard", 11},	
    {"bitratelimit", 11}	
};
std::map<std::string, std::uint16_t> mapOfNtpParamVals{
    {"on", 1},
    {"off", 1},
    {"100", 2},
    {"86400", 2},
    {"on", 4},
    {"off", 4}	
};
std::map<std::string, std::uint16_t> mapOfdot1XParamVals{
    {"tls", 1},
    {"peap", 1}	
};

#if defined(TLS_SSH_VERSION)

// function ro connect without TLS enabled using resolution of host from /etc/hosts 
//
socket_ptr connect_to(const asio::ip::address& ip, uint16_t port) {
  socket_ptr socket = std::make_unique<asio::ip::tcp::socket>(io_context);
  socket->connect(asio::ip::tcp::endpoint(ip, port));
  return socket;
}


using ssl_stream_ptr = std::unique_ptr<asio::ssl::stream<asio::ip::tcp::socket>>;

// function to connect using TLS 1.2 no cert enabled
//
ssl_stream_ptr connect_to_with_tls(const asio::ip::address& ip, uint16_t port) {
  ssl_stream_ptr stream = std::make_unique<asio::ssl::stream<asio::ip::tcp::socket>>(io_context, ssl_context);
  // The certificate check is omitted.
  stream->lowest_layer().connect(asio::ip::tcp::endpoint(ip, port));
  stream->handshake(asio::ssl::stream_base::client);

  return stream;
}

/*
 * Socket (SyncStream) The template sends the url and read the reply
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
//#define KEEP_CONN_ALIVE
#if defined(KEEP_CONN_ALIVE)                                           // use keep conn alive if you want to use this continously in a program reading objects from mavlink.
  request_stream << cmd 
                 << "Host: " << host << "\r\n"
			     << "Connection: Keep-Alive\r\n";                      //  <--- to keep it alive use instead of close above
  request_stream << "\r\n";
#else                                                                  // otherwise like this command line example it will close the socket upon comepletion of the command
  request_stream << cmd 
                 << "Host: " << host << "\r\n"
                 << "Connection: Close\r\n";
  request_stream << "\r\n";
#endif
  asio::write(stream, asio::buffer(request_stream.str()));

  boost::system::error_code error;
  asio::streambuf receive_buffer;

  asio::read(stream, receive_buffer, error);
  if (error && error != asio::error::eof) {
    throw std::runtime_error(error.message());
  }
  std::cout << asio::buffer_cast<const char*>(receive_buffer.data()) << std::endl;
}
#endif

// end the request with a keep-alive to the socket
//
void set_msg_end_keep_connection( std::string& req_msg, std::string& ip ) {
			req_msg.append("Host:");
			req_msg.append(ip);
			req_msg.append("\r\n");						
			req_msg.append("Connection: Keep-Alive\r\n");     
}

// end the request with a close to the socket
//
void set_msg_end_close_connection( std::string& req_msg, std::string& ip ) {
			req_msg.append("Host:");
			req_msg.append(ip);
			req_msg.append("\r\n");						   
            req_msg.append("Connection: close\r\n\r\n");      // <--- to close the connection
}

// verify the certificate if used and print the subject name
// ref:- https://habr.com/ru/post/271203/?ysclid=la9pnlvqsa795129099
//
bool verify_certificate(bool preverified, boost::asio::ssl::verify_context& ctx)
{
	// The verify callback can be used to check whether the certificate that is
	// being presented is valid for the peer. For example, RFC 2818 describes
	// the steps involved in doing this for HTTPS. Consult the OpenSSL
	// documentation for more details. Note that the callback is called once
	// for each certificate in the certificate chain, starting from the root
	// certificate authority.

	// In this example we will simply print the certificate's subject name.
	char subject_name[256];
	X509* cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());
	X509_NAME_oneline(X509_get_subject_name(cert), subject_name, 256);
	std::cout << "Verifying " << subject_name << "\n";

	return preverified;
}
	
// fast string splitter to get the data from the http response faster than std::split
//
// https://qiita.com/iseki-masaya/items/70b4ee6e0877d12dafa8 
//
vector<string> split_naive(string &s, char delim) {
    vector<string> elems;
    string item;
    for (char ch: s) {
        if (ch == delim) {
            if (!item.empty())
                elems.push_back(item);
            item.clear();
        }
        else {
            item += ch;
        }
    }
    if (!item.empty())
        elems.push_back(item);
    return elems;
}

// no need to use this function as it is slower than the one above see qiita blog to confirm this
//
vector<string> split(string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
    if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

// find the key in the map
//
std::int16_t find_key_in_map(std::map<std::string, std::uint16_t> my_map, std::string find_this_key) {

    if ( auto iter = my_map.find(find_this_key); iter != end(my_map) ) {
        std::cout << iter->second << std::endl;
		return iter->second;
    } else {
        std::cout << "not exists" << std::endl;
		return -1;
    }
}

// returns a list of keys for a given value in our case options for a given audio parameter
//
std::vector<std::pair<std::string, std::int16_t>> find_values_in_map(std::map<std::string, std::uint16_t> my_map, std::int16_t find_this_val) {
    std::vector<std::pair<std::string, std::int16_t>> out_vec;
    for (const auto& [key, value] : my_map){
        if (value == find_this_val) {
            std::cout << key << " => " << value << "\n";
            out_vec.push_back(std::make_pair(key,value));
        }
    }
    return out_vec;
}

// returns a list of keys for a given value in our case options for a given audio parameter
//
std::vector<std::string> list_values_in_map_matching_option(std::map<std::string, std::uint16_t> my_map, std::int16_t find_this_val) {
    std::vector<std::string> out_vec;
    for (const auto& [key, value] : my_map){
        if (value == find_this_val) {
            std::cout << key << " => " << value << "\n";
            out_vec.push_back(key);
        }
    }
    return out_vec;
}

// returns the index number shown in the map for a given parameter name
//
std::int16_t get_index_from_list(std::list<std::string> l, std::string tag) {
     std::int16_t vv = 0;
     for (auto i : l) {
         ++vv;
         std::string str = i;
         std::regex patern{tag};
         std::smatch match{};
         if (std::regex_search(str, match, patern)) {
            return vv;
         }
     }
     return -1;
}

// function to pull out two names inside of <> in HTML
// example string str1 = "The HTML tag <title> means that ... <param>";
// return vector of pairs where :-  pair == name_inside_bracket, position
//
std::vector<std::pair<std::string, std::int16_t>> get_inside_brackets( std::string str1 ) {
	
	std::vector<std::pair<std::string, std::int16_t>> ret_vec;
    boost::regex  r( "<[^>]+>" );
    boost::smatch m;

    if( boost::regex_search(str1, m, r) )
    {
        std::cout << "found (pos=" << m.position() << ")" << std::endl;
        std::cout << " ==> " << m.str() << std::endl;
		ret_vec.push_back(std::make_pair(m.str(),m.position()));
        auto pos = m.position();
        if ( boost::regex_search(str1.substr(pos + 1),m,r) ) {
            std::cout << "found (pos=" << m.position() << ")" << std::endl;
            std::cout << " ==> " << m.str() << std::endl;
			ret_vec.push_back(std::make_pair(m.str(),m.position()));
        } else {
			ret_vec.push_back(std::make_pair("no_second",0));			
		}
    } else {
	    ret_vec.push_back(std::make_pair("no_first",0));
	    ret_vec.push_back(std::make_pair("no_second",0));			
	}
	return ret_vec;
}

// enclose the tag with # mark
// where tag must be all number or char either side of A<>A12345A
// e.g. string str2 = "A12345A AaaaA A1a1A A9A";
//
std::string enclose_tag_in_hash( std::string str2 ) {
    boost::regex r2( "A([1-9]*|[a-z]*)A" );                                     // all numbers or all letters between A's
	std::string s;
    {
		s = boost::regex_replace( str2, r2, "#$0#", boost::format_all );
        std::cout << s << std::endl;
    }
    return s;
}

// enclose the tag with # mark
// where tag must be combination of number or char either side of A<>A12345A
// e.g. string str2 = "A12345A AaaaA A1a1A A9A";
//
std::string enclose_tag_in_hash2( std::string str2 ) {
    boost::regex r2( "A([a-z]*[1-9]*[a-z]*)A" );                                   // any letters any numbers any letters between A's
    std::string s;
    {
		s = boost::regex_replace( str2, r2, "#$0#", boost::format_all );
        std::cout << s << std::endl;
    }
    return s;
}

int main(const int ac, const char* const * const av)
try
{
    // Define options this is for test purpose and would be replaced by relevant mavlink objects defined in the XML
    po::options_description description("option (-n 1=led, 2=info 3=auth 4=capability 5=ptzf\n -v value_of_option  -p pan -t tilt -z zoom -f focus -c codec -s variableTagName)");
    description.add_options()
    // ("Option name", "Argument (optional)", "Option description")
    ("number,n", po::value<int>()->default_value(1), "The method operation number as per SonySRG300_methods_e")
    ("value,v", po::value<int>()->default_value(DEFAULT_VAL_NOT_SET), "value used for set in the method")
    ("pan,p", po::value<std::string>()->default_value("0"), "value for pan")
    ("tilt,t", po::value<std::string>()->default_value("0"), "value for tilt")
    ("zoom,z", po::value<std::string>()->default_value("0"), "value for zoom")
    ("focus,f", po::value<std::string>()->default_value("0"), "value for focus")
    ("codec,c", po::value<std::string>()->default_value("0"), "value for focus")
    ("1position1,1p1", po::value<std::string>()->default_value("stop"), "first parameter to position command")
    ("2position2,2p2", po::value<std::string>()->default_value("motor"), "second parameter to position command")
    ("3position3,3p3", po::value<std::string>()->default_value("Image1"), "third parameter to position command")
    ("string,s", po::value<std::string>()->default_value("BacklightCompensationLevel"), "")
    ("xxx,x", po::value<std::string>()->default_value("aac16_64"), "to pass a string parameter to the webserver")
    ("image_num,i", po::value<std::string>()->default_value("1"), "image codec number")
    ("help,h", "show this help message");

    // Command Line Parsing
    po::variables_map vm;
    try {
        po::store(parse_command_line(ac, av, description), vm);
    } catch (po::error &e) {
       // Nonexistent option ・ throw an exception if the wrong type is specified
       std::cout << e.what() << std::endl;
       return -1;
    }

    po::notify(vm);

    //help, output help (description content) when h is entered
    if (vm.count("help")) {
       std::cout << description << std::endl;
       return 0;
    }

    // Option number output - if no option you must exit this test code
    int num = vm["number"].as<int>();
	//try {
	//    int num = vm["number"].as<std::int16_t>();
	//} catch (boost::bad_any_cast &e) {
	//	std::cout << e.what() << std::endl;
	//	std::cout << "\033[31m •••••• you must enter an option number for the camera method •••••• \033[0m" << std::endl;
	//	return -1;
	//}
    //std::cout << num << std::endl;
	
    // consider the -s "string request state "
    //
    //std::cout << vm["string"].as<std::string>() << std::endl;
    std::string req_state = vm["string"].as<std::string>();

#if defined(TLS_SSH_VERSION)
    const std::string host = "localhost";                                      // the name of the server as per /etc/hosts
    asio::ip::tcp::resolver resolver(io_context);
    auto endpoints = resolver.resolve(asio::ip::tcp::v4(), host, "");
    auto ip = endpoints.begin()->endpoint().address();
    socket_ptr s;
    ssl_stream_ptr s_tls;
	
	switch (g_opertation_type) {
		 case HTTP_ONLY:
		 {   
		     s = connect_to(ip, 80);
			 std::cout << "\033[35m •••••• connecting to SONY SRG300/1-SE camera using HTTP 2nd version ••••••\033[0m" << std::endl;
		 }
		 break;
		 
		 case HTTPS_TLS12:
		 {
		     s_tls = connect_to_with_tls(ip, 443);
			 std::cout << "\033[34m •••••• connecting to SONY SRG300/1-SE camera using HTTPS TLS1.2 ••••••\033[0m" << std::endl;
         }
  		 break;
		 
		 default:
		 break;
	} 
	
#else		

//#define SSL_CERT                                                                                     // choose the method we are using 
#define HTTP_ONLY

    asio::io_service io_service;
 
#if defined(HTTP_ONLY)		
            //Create a TCP socket !!!!!
            ip::tcp::socket sock(io_service);
    
	        // fixed ip connection
            ip::tcp::endpoint endpoint( ip::address::from_string(YOUR_CAM_IP_ADDR), 80);

            //Connect to Socket
	        std::cout << "\033[32m •••••• connecting to SONY SRG300/1-SE camera using HTTP 1st version ••••••\033[0m" << std::endl;
            sock.connect(endpoint);
#endif
		 
#if defined(SSL_CERT)
            // SSL 
            // boost::asio::ssl::context ctx(io_service, boost::asio::ssl::context_base::sslv3_client);
            // boost::asio::ssl::context ctx(io_service, boost::asio::ssl::context::sslv23);
	        asio::ssl::context ctx(boost::asio::ssl::context::sslv23);
	        ctx.load_verify_file("rootca.crt");
	
	        // Get a list of endpoints corresponding to the server name.
            boost::asio::ip::tcp::resolver resolver(io_service);
            boost::asio::ip::tcp::resolver::query query(MY_HOST_NAME, "https");
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

            boost::asio::ssl::stream<boost::asio::ip::tcp::socket> sock(io_service, ctx);
			std::cout << "\033[33m •••••• connecting to SONY SRG300/1-SE camera using SSL CERT ••••••\033[0m" << std::endl;
            asio::connect(sock.lowest_layer(), endpoint_iterator);
            sock.lowest_layer().set_option(boost::asio::ip::tcp::no_delay(true));

            // Perform SSL handshake and verify the remote host's
            // certificate. The options are listed below
	        /*
	                ssl::verify_none                                                                             <-- change  if you dont want it
                    ssl::verify_peer
                    ssl::verify_fail_if_no_peer_cert
                    ssl::verify_client_once
	        */
            sock.set_verify_mode(boost::asio::ssl::verify_peer);
            sock.set_verify_callback(boost::asio::ssl::rfc2818_verification(MY_HOST_NAME));                      // sets this to rfc2818
			//sock.set_verify_callback(true, verify_certificate);                                                // prints cert TBD work this one out ......
            sock.handshake(boost::asio::ssl::stream_base::client);
#endif
		
	    // If we need TLS1.2 we use this instead
        // boost::asio::ssl::context ctx(io_service, boost::asio::ssl::context_base::tlsv12_client);
	
    //Send Message
    asio::streambuf request;
    ostream request_ostream(&request);
#endif

	switch(num) {

		case PwrLED:
		{
            // change state of power led
            // set response string replacement
	        std::string cmd_power_led = "/led_conf/system.cgi?PowerLed=off";
		    //try {
	                if (vm["value"].as<int>() == 1) {
	                    cmd_power_led = std::regex_replace("/led_conf/system.cgi?PowerLed=off", std::regex("off"), "on");
					}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//   std::cout << "unexceptional error occured PwrLED" << std::endl;
            //}		
	        std::string req_msg = "POST ";
	        req_msg.append(cmd_power_led);
	        req_msg.append(" HTTP/1.0\r\n\r\n");
			// ---> not sure if you have to add this if so add to them all <---
			//req_msg.append("Host:");
			//req_msg.append(YOUR_CAM_IP_ADDR);
			//req_msg.append("\r\n");						
			//req_msg.append("Accept: */*\r\n");
			//req_msg.append(""Connection: Keep-Alive\r\n");     <--- to keep it alive
            //req_msg.append("Connection: close\r\n\r\n");       <--- to close the connection
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, req_msg);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, req_msg);
		       break;
		 
		       default:
		       break;
	       }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( req_msg, my_ip );
#else
			set_msg_end_close_connection( req_msg, my_ip );
#endif
            request_ostream << req_msg;
#endif
			std::cout << "\033[33m command : \033[0m" << req_msg << std::endl;
		}
		break;

		case SysInfo:
		{
		// get system information
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, "GET /sysinfo/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n");
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, "GET /sysinfo/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n");
		       break;
		 
		       default:
		       break;
	       }
#else
	    std::string req_msg = "GET /sysinfo/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n";
		std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
        set_msg_end_keep_connection( req_msg, my_ip );
#else
		set_msg_end_close_connection( req_msg, my_ip );
#endif
        request_ostream << req_msg;
#endif
        }
		break;
		
		case Auth:
		{
	        // change state of authentication
            // set response string replacement
	        std::string cmd_auth = "/system_authen/system.cg1?CgiAuthen=off";
		    //try {
	                if (vm["value"].as<int>() == 1) {
	                    cmd_auth = std::regex_replace("/system_authen/system.cg1?CgiAuthen=off", std::regex("off"), "on");
	                }
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured Auth" << std::endl;
            //}
	        std::string req_msg = "POST ";
	        req_msg.append(cmd_auth);
	        req_msg.append(" HTTP/1.0\r\n\r\n");
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, req_msg);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, req_msg);
		       break;
		 
		       default:
		       break;
	       }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( req_msg, my_ip );
#else
		    set_msg_end_close_connection( req_msg, my_ip );
#endif
            request_ostream << req_msg;
#endif
			std::cout << "\033[33m command : \033[0m" << req_msg << std::endl;
		}
		break;

		case Capab:
		{
	    // get camera capability
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, "POST /capability/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n");
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, "POST /capability/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n");
		       break;
		 
		       default:
		       break;
	       }
#else
	    std::string req_msg = "POST /capability/inquiry.cgi?inq=system HTTP/1.0\r\n\r\n";
		std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
        set_msg_end_keep_connection( req_msg, my_ip );
#else
		set_msg_end_close_connection( req_msg, my_ip );
#endif
	    request_ostream << req_msg;
#endif
        }
		break;

		case PTZF:                         // called with ./this_program -n 5 -p "2" -t "4" -z "9" -f "1234"
		{
	        // set pan tilt zoom
	        std::string cmd_ptz = "POST /absolute/ptzf.cgi?AbsolutePTZF=";
		    std::string arg_str;
		    //try {
			    arg_str = vm["pan"].as<std::string>();
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured PTZF" << std::endl;
            //}
	        cmd_ptz.append(arg_str);
		    cmd_ptz.append(",");
		    //try {
			    arg_str = vm["tilt"].as<std::string>();
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured PTZF" << std::endl;
            //}
	        cmd_ptz.append(arg_str);	
		    cmd_ptz.append(",");	
		    //try {
			    arg_str = vm["zoom"].as<std::string>();
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured PTZF" << std::endl;
            //}	
	        cmd_ptz.append(arg_str);	
		    cmd_ptz.append(",");	
		    //try {
			    arg_str = vm["focus"].as<std::string>();
			    cmd_ptz.append(arg_str);
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured PTZF" << std::endl;
            //}
            cmd_ptz.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, cmd_ptz);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, cmd_ptz);
		       break;
		 
		       default:
		       break;
	       }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_ptz, my_ip );
#else
		    set_msg_end_close_connection( cmd_ptz, my_ip );
#endif
	        request_ostream << cmd_ptz;
#endif			        
			std::cout << "\033[33m command : \033[0m" << cmd_ptz << std::endl;
		}
		break;

        // need to set to value and string options
        case Expos:                             
		{
		    //try {
				std::cout << "4sp " << vm["xxx"].as<std::string>() << std::endl;
	            auto itr = std::find(listOfExposureParams.begin(), listOfExposureParams.end(), vm["string"].as<std::string>());
	            std::string cmd_expos = "POST /exposure/imaging.cgi?";
				cmd_expos.append(*itr);
				cmd_expos.append(" = ");
			    if (vm.count("value") && (vm["value"].as<int>() != DEFAULT_VAL_NOT_SET)) {	           // called with ./this_program -n 6 -s "ExposureBrightLevel" -v 10
                   std::cout << "has a value" << std::endl;				
				   cmd_expos.append(std::to_string(vm["value"].as<int>()));
                } else if (vm.count("xxx")) {                                          //   ./this_program -n 6 -s "BacklightCompensationMode" -x "on"
	               auto itr_s = std::find(listOfExposureStrings.begin(), listOfExposureStrings.end(), vm["xxx"].as<std::string>());	
				   cmd_expos.append(*itr_s);				   
                }
				cmd_expos.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, cmd_expos);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, cmd_expos);
		       break;
		 
		       default:
		       break;
	       }
#else
				std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_expos, my_ip );
#else
		        set_msg_end_close_connection( cmd_expos, my_ip );
#endif
	            request_ostream << cmd_expos;
#endif	
				std::cout << "\033[33m command : \033[0m" << cmd_expos << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured exposure" << std::endl;
            //}		
        }			
		break;

        case WhiteBal:
		{
		    //try {
	            auto itr = std::find(listOfWhiteBalParams.begin(), listOfWhiteBalParams.end(), vm["string"].as<std::string>());
	            std::string cmd_wb = "POST /white_balance/imaging.cgi?";
				cmd_wb.append(*itr);
				cmd_wb.append(" = ");
			    if (vm.count("value") && (vm["value"].as<int>() != DEFAULT_VAL_NOT_SET)) {				
				    cmd_wb.append(std::to_string(vm["value"].as<int>()));
                } else if (vm.count("xxx")) {
	               auto itr_s = std::find(listOfWhiteBalStrings.begin(), listOfWhiteBalStrings.end(), vm["xxx"].as<std::string>());	
				   cmd_wb.append(*itr_s);				   
                }
				cmd_wb.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	            switch (g_opertation_type) {
		           case HTTP_ONLY:
		           http_cmd(*s, host, cmd_wb);
		           break;
		 
		           case HTTPS_TLS12:
		           http_cmd(*s_tls, host, cmd_wb);
		           break;
		 
		           default:
		           break;
	            }
#else
				std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_wb, my_ip );
#else
		        set_msg_end_close_connection( cmd_wb, my_ip );
#endif
				request_ostream << cmd_wb;
#endif					
				std::cout << "\033[33m command : \033[0m" << cmd_wb << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured white balance" << std::endl;
            //}
        }			
		break;

        case Ipv4Net:
		{
		    //try {
	            auto itr = std::find(listOfNetIpv4Params.begin(), listOfNetIpv4Params.end(), vm["string"].as<std::string>());
	            std::string cmd_ip4net = "POST /network_ipv4/network.cgi?";
				cmd_ip4net.append(*itr);
				cmd_ip4net.append(" = ");
				cmd_ip4net.append(vm["xxx"].as<std::string>());
				cmd_ip4net.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	            switch (g_opertation_type) {
		            case HTTP_ONLY:
		            http_cmd(*s, host, cmd_ip4net);
		            break;
		 
		            case HTTPS_TLS12:
		            http_cmd(*s_tls, host, cmd_ip4net);
		            break;
		 
		            default:
		            break;
	            }
#else
				std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_ip4net, my_ip );
#else
		        set_msg_end_close_connection( cmd_ip4net, my_ip );
#endif
				request_ostream << cmd_ip4net;
#endif	
				std::cout << "\033[33m command : \033[0m" << cmd_ip4net << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured Ipv4Net" << std::endl;
            //}				
        }			
		break;	
		
        case absZoom:
		{
		    //try {
	            std::string cmd_aZoom = "POST /absolute_zoom/ptzf.cgi?AbsoluteZoom=";
				cmd_aZoom.append(" = ");
				cmd_aZoom.append(std::to_string(vm["value"].as<int>()));
				cmd_aZoom.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	            switch (g_opertation_type) {
		           case HTTP_ONLY:
		           http_cmd(*s, host, cmd_aZoom);
		           break;
		 
		           case HTTPS_TLS12:
		           http_cmd(*s_tls, host, cmd_aZoom);
		           break;
		 
		           default:
		           break;
	            }
#else
				std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_aZoom, my_ip );
#else
		        set_msg_end_close_connection( cmd_aZoom, my_ip );
#endif
				request_ostream << cmd_aZoom;
#endif	
				std::cout << "\033[33m command : \033[0m" << cmd_aZoom << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured absZoom" << std::endl;
            //}	
        }			
		break;	

		case contZoom:
		{
	        // set pan tilt zoom
	        std::string cmd_ptz = "POST /continuous/ptzf.cgi?ContinuousPanTiltZoom=";
		    std::string arg_str;
		    //try {
			    if (vm.count("pan")) {
			        arg_str = vm["pan"].as<std::string>();
	                cmd_ptz.append(arg_str);
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured continuous Zoom" << std::endl;
            //}
		    //try {
			    if (vm.count("tilt")) {
			        arg_str = vm["tilt"].as<std::string>();
					cmd_ptz.append(arg_str);	
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured continuous Zoom" << std::endl;
            //}	
		    //try {
			    if (vm.count("zoom")) {
			        arg_str = vm["zoom"].as<std::string>();
	                cmd_ptz.append(arg_str);	
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured continuous Zoom" << std::endl;
            //}		
		    //try {
			    if (vm.count("codec")) {
			        arg_str = vm["codec"].as<std::string>();
			        cmd_ptz.append(arg_str);
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured continuous Zoom" << std::endl;
            //}
            cmd_ptz.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	       switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, cmd_ptz);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, cmd_ptz);
		       break;
		 
		       default:
		       break;
	       }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_ptz, my_ip );
#else
		    set_msg_end_close_connection( cmd_ptz, my_ip );
#endif
	        request_ostream << cmd_ptz;
#endif			
			std::cout << "\033[33m command : \033[0m" << cmd_ptz << std::endl;			
		}
		break;

        case Position:                     // example ./this_program -n 11 -1 "left" -2 "24" or ./this_program -n 11 -1 "stop" -2 "pantilt"
        {
	        // set position
	        std::string cmd_ptz = "POST /position/ptzf.cgi?Move=";
		    std::string arg_str;
		    //try {
			    if (vm.count("1position1")) {
			        arg_str = vm["1position1"].as<std::string>();
	                cmd_ptz.append(arg_str);
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured position" << std::endl;
            //}
		    //try {
			    if (vm.count("2position2")) {
			        arg_str = vm["2position2"].as<std::string>();
					cmd_ptz.append(arg_str);	
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured position" << std::endl;
            //}
		    //try {
			    if (vm.count("3position3")) {
			        arg_str = vm["3position3"].as<std::string>();
	                cmd_ptz.append(arg_str);	
		            cmd_ptz.append(",");
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
			//    arg_str = "0";
		    //} catch (...) {
			//    std::cout << "unexceptional error occured position" << std::endl;
            //}			
            cmd_ptz.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		       case HTTP_ONLY:
		       http_cmd(*s, host, cmd_ptz);
		       break;
		 
		       case HTTPS_TLS12:
		       http_cmd(*s_tls, host, cmd_ptz);
		       break;
		 
		       default:
		       break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_ptz, my_ip );
#else
		    set_msg_end_close_connection( cmd_ptz, my_ip );
#endif
	        request_ostream << cmd_ptz;
#endif				
			std::cout << "\033[33m command : \033[0m" << cmd_ptz << std::endl;	
        }
        break;

        case Audio:
		{
		    //try {
			    if (vm.count("xxx")) {
	                auto itr = std::find(listOfAudioParams.begin(), listOfAudioParams.end(), vm["string"].as<std::string>());
					std::int16_t paramIdx = get_index_from_list(listOfAudioParams, *itr);
					if (paramIdx != -1) {                                                                                                       // then the parameter is a valid audio parameter
	                    std::string cmd_audio = "POST /audio_control/camera.cgi?";
				        cmd_audio.append(*itr);
				        cmd_audio.append(" = ");
					    std::vector<std::string> valuesForParam = list_values_in_map_matching_option(mapOfAudioParamVals, paramIdx);   // get a list of allowable values for that param
						auto itr2 = std::find(valuesForParam.begin(), valuesForParam.end(), vm["xxx"].as<std::string>());        // check that the correct param value was passed
				        cmd_audio.append(*itr2);
				        cmd_audio.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	                    switch (g_opertation_type) {
		                    case HTTP_ONLY:
		                    http_cmd(*s, host, cmd_audio);
		                    break;
		 
		                    case HTTPS_TLS12:
		                    http_cmd(*s_tls, host, cmd_audio);
		                    break;
		 
		                    default:
		                    break;
	                    }
#else
			            std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                        set_msg_end_keep_connection( cmd_audio, my_ip );
#else
		                set_msg_end_close_connection( cmd_audio, my_ip );
#endif
				        request_ostream << cmd_audio;
#endif
			            std::cout << "\033[33m command : \033[0m" << cmd_audio << std::endl;	
					}
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured Audio" << std::endl;
            //}	
        }			
		break;

        case SetUp:
		{
		    //try {
			    if (vm.count("xxx")) {
	                auto itr = std::find(listOfSetUpParams.begin(), listOfSetUpParams.end(), vm["string"].as<std::string>());
					std::int16_t paramIdx = get_index_from_list(listOfSetUpParams, *itr);
					if (paramIdx != -1) {                                                                                                       // then the parameter is a valid audio parameter
	                    std::string cmd_set = "POST /setup/camera.cgi?";
				        cmd_set.append(*itr);
				        cmd_set.append(" = ");
					    std::vector<std::string> valuesForParam = list_values_in_map_matching_option(mapOfSetUpParamVals, paramIdx);   // get a list of allowable values for that param
						auto itr2 = std::find(valuesForParam.begin(), valuesForParam.end(), vm["xxx"].as<std::string>());        // check that the correct param value was passed
				        cmd_set.append(*itr2);
				        cmd_set.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	                    switch (g_opertation_type) {
		                    case HTTP_ONLY:
		                    http_cmd(*s, host, cmd_set);
		                    break;
		 
		                    case HTTPS_TLS12:
		                    http_cmd(*s_tls, host, cmd_set);
		                    break;
		 
		                    default:
		                    break;
	                    }
#else
			            std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                        set_msg_end_keep_connection( cmd_set, my_ip );
#else
		                set_msg_end_close_connection( cmd_set, my_ip );
#endif
				        request_ostream << cmd_set;
#endif
						std::cout << "\033[33m command : \033[0m" << cmd_set << std::endl;
					}
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured SetUp" << std::endl;
            //}	
        }			
		break;
		
		case Stabiliz:
		{
            // change state of stabilizer
            // 
	        std::string cmd_stabil = "/stabilizer/camera.cgi?Stabilizer=off";
		    //try {
	            if (vm["value"].as<int>() == 1) {
	                cmd_stabil = std::regex_replace("/stabilizer/camera.cgi?Stabilizer=off", std::regex("off"), "on");
	            }
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured Stabilize" << std::endl;
            //}		
	        std::string req_msg = "POST ";
	        req_msg.append(cmd_stabil);
	        req_msg.append(" HTTP/1.0\r\n\r\n");
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		        case HTTP_ONLY:
		        http_cmd(*s, host, req_msg);
		        break;
		 
		        case HTTPS_TLS12:
		        http_cmd(*s_tls, host, req_msg);
		        break;
		 
		        default:
		        break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( req_msg, my_ip );
#else
		    set_msg_end_close_connection( req_msg, my_ip );
#endif
            request_ostream << req_msg;
#endif

			std::cout << "\033[33m command : \033[0m" << req_msg << std::endl;
		}
		break;

        case Picture:
		{
		    //try {
	            auto itr = std::find(listOfPictureParams.begin(), listOfPictureParams.end(), vm["string"].as<std::string>());
	            std::string cmd_p = "POST /picture/imaging.cgi?";
				cmd_p.append(*itr);
				cmd_p.append(" = ");
			    if (vm.count("value")&& (vm["value"].as<int>() != DEFAULT_VAL_NOT_SET)) {				
				    cmd_p.append(std::to_string(vm["value"].as<int>()));
                } else if (vm.count("xxx")) {
	               auto itr_s = std::find(listOfPictureStrings.begin(), listOfPictureStrings.end(), vm["xxx"].as<std::string>());	
				   cmd_p.append(*itr_s);				   
                }
				cmd_p.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	            switch (g_opertation_type) {
		            case HTTP_ONLY:
		            http_cmd(*s, host, cmd_p);
		            break;
		 
		            case HTTPS_TLS12:
		            http_cmd(*s_tls, host, cmd_p);
		            break;
		 
		            default:
		            break;
	            }
#else
			    std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_p, my_ip );
#else
		        set_msg_end_close_connection( cmd_p, my_ip );
#endif
				request_ostream << cmd_p;
#endif				

				std::cout << "\033[33m command : \033[0m" << cmd_p << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured picture" << std::endl;
            //}
        }			
		break;

        case AdaptRateCont:
		{
		    //try {
	            auto itr = std::find(listOfArcParams.begin(), listOfArcParams.end(), vm["string"].as<std::string>());
	            std::string cmd_p = "POST /arc_config/camera.cgi?";
				cmd_p.append(*itr);
				cmd_p.append(" = ");
			    if (vm.count("value")&& (vm["value"].as<int>() != DEFAULT_VAL_NOT_SET)) {				
				    cmd_p.append(std::to_string(vm["value"].as<int>()));
                } else if (vm.count("xxx")) {
	               auto itr_s = std::find(listOfArcStrings.begin(), listOfArcStrings.end(), vm["xxx"].as<std::string>());	
				   cmd_p.append(*itr_s);				   
                }
				cmd_p.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	            switch (g_opertation_type) {
		            case HTTP_ONLY:
		            http_cmd(*s, host, cmd_p);
		            break;
		 
		            case HTTPS_TLS12:
		            http_cmd(*s_tls, host, cmd_p);
		            break;
		 
		            default:
		            break;
	            }
#else
			    std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                set_msg_end_keep_connection( cmd_p, my_ip );
#else
		        set_msg_end_close_connection( cmd_p, my_ip );
#endif
				request_ostream << cmd_p;
#endif					
				std::cout << "\033[33m command : \033[0m" << cmd_p << std::endl;
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured picture" << std::endl;
            //}
		}
        break;

        case Image:
		{
		    //try {
			    if (vm.count("xxx")) {
	                auto itr = std::find(listOfImageParams.begin(), listOfImageParams.end(), vm["string"].as<std::string>());
					std::int16_t paramIdx = get_index_from_list(listOfImageParams, *itr);
					std::string cmd_set = "POST /";
					if (paramIdx != -1) {   					// then the parameter is a valid audio parameter
	                    std::string commandName = "image";
						commandName += vm["image_num"].as<std::string>();
						commandName += "ImageCodecMaxNum";
						cmd_set.append(commandName);
						cmd_set.append("/camera.cgi?");
				        cmd_set.append(*itr);
				        cmd_set.append(" = ");
					    std::vector<std::string> valuesForParam = list_values_in_map_matching_option(mapOfImageParamVals, paramIdx);   // get a list of allowable values for that param
                        int val = 0; 
                        if (((paramIdx >= 2) || (paramIdx <= 5)) || (paramIdx == 8)) {
						    auto itr2 = std::find(valuesForParam.begin(), valuesForParam.end(), vm["xxx"].as<std::string>());        // check that the correct param value was passed
				            cmd_set.append(*itr2);
						} else if (((paramIdx >= 6) || (paramIdx <= 7)) || (paramIdx == 11)) {
			                try {
								  std::string s_val = vm["xxx"].as<std::string>();
                                  val = std::atoi(s_val.c_str());       
			                } catch (exception& e) {
                                  cout << e.what();
				                  return -1;
                            } catch (...) {
			                      std::cout << "unexceptional error occured Image_value" << std::endl;
                            }
                            if (valuesForParam.size() == 2) {
								int v_min = 0;
								int v_max = 1;
			                    try {
								  //sort(valuesForParam.begin(), valuesForParam.end());
                                  v_min = std::atoi(valuesForParam[0].c_str());      
                                  v_max = std::atoi(valuesForParam[1].c_str());     								  
			                    } catch (exception& e) {
                                  cout << e.what();
				                  return -1;
                                } catch (...) {
			                      std::cout << "unexceptional error occured Image_range_check" << std::endl;
                                }
								if ((val >= v_min) && (val <= v_max)) {
									cmd_set.append(std::to_string(val));
								}
                            }	
						} else if (paramIdx == 10) {
			                try {
								  std::string s_val = vm["xxx"].as<std::string>();
                                  val = std::atoi(s_val.c_str());       
			                } catch (exception& e) {
                                  cout << e.what();
				                  return -1;
                            } catch (...) {
			                      std::cout << "unexceptional error occured Image_value" << std::endl;
                            }
                            if (valuesForParam.size() >= 4) {
								int v_minmin = 0;
								int v_min = 0;
								int v_max = 1;
								int v_maxmax = 1;
								std::vector<int> vv;
								for(size_t ii=0; ii<valuesForParam.size(); ii++) {
			                        try {									
									    vv.push_back(std::atoi(valuesForParam[ii].c_str()));
			                        } catch (exception& e) {
                                        std::cout << e.what();
				                        return -1;
                                    } catch (...) {
			                            std::cout << "unexceptional error occured Image_range_check for JpBandWidth" << std::endl;
                                    }									
								}
								sort(vv.begin(), vv.end());              // sort as integers
                                v_minmin = vv[0];      
                                v_maxmax = vv[3]; 
                                v_min = vv[1];      
                                v_max = vv[2];     								  

								if (((val >= v_min) && (val <= v_max)) || ((val == v_minmin) || (val == v_maxmax))) {
									cmd_set.append(std::to_string(val));
								}
                            }								
						} else {
						    cmd_set.append(vm["xxx"].as<std::string>());
					    }
				        cmd_set.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	                    switch (g_opertation_type) {
		                   case HTTP_ONLY:
		                   http_cmd(*s, host, cmd_set);
		                   break;
		 
		                   case HTTPS_TLS12:
		                   http_cmd(*s_tls, host, cmd_set);
		                   break;
		 
		                   default:
		                   break;
	                    }
#else
			            std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
                        set_msg_end_keep_connection( cmd_set, my_ip );
#else
		                set_msg_end_close_connection( cmd_set, my_ip );
#endif
				        request_ostream << cmd_set;
#endif
						std::cout << "\033[33m command : \033[0m" << cmd_set << std::endl;
					}
				}
		    //} catch (boost::bad_any_cast &e) {
			//    std::cout << e.what() << std::endl;
		    //} catch (...) {
			//    std::cout << "unexceptional error occured SetUp" << std::endl;
            //}	
	    }
        break;

		case iSet:
		{
		    std::string req_cmd = "POST /image_set/camera.cgi?InsertIFrame";
		    std::string iNum = vm["image_num"].as<std::string>();
		    req_cmd += iNum;
		    req_cmd.append("=on");
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		        case HTTP_ONLY:
		        http_cmd(*s, host, req_cmd);
		        break;
		 
		        case HTTPS_TLS12:
		        http_cmd(*s_tls, host, req_cmd);
		        break;
		 
		        default:
		        break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( req_cmd, my_ip );
#else
		    set_msg_end_close_connection( req_cmd, my_ip );
#endif
		    request_ostream << req_cmd;	
#endif	
		    std::cout << "The option was not valid" << std::endl;
		}
		break;	

		case datTim:
		{
		//try {
	        auto itr = std::find(listOfDatTimParams.begin(), listOfDatTimParams.end(), vm["string"].as<std::string>());
	        std::string cmd_p = "POST /datetime/system.cgi?";
			cmd_p.append(*itr);
			cmd_p.append(" = ");
            if (vm.count("xxx")) {
	            std::string itr_s = vm["xxx"].as<std::string>();	
				cmd_p.append(itr_s);				   
            }
			cmd_p.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		        case HTTP_ONLY:
		        http_cmd(*s, host, cmd_p);
		        break;
		 
		        case HTTPS_TLS12:
		        http_cmd(*s_tls, host, cmd_p);
		        break;
		 
		        default:
		        break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_p, my_ip );
#else
		    set_msg_end_close_connection( cmd_p, my_ip );
#endif
			request_ostream << cmd_p;
#endif				
			std::cout << "\033[33m command : \033[0m" << cmd_p << std::endl;
		//} catch (boost::bad_any_cast &e) {
		//    std::cout << e.what() << std::endl;
		//} catch (...) {
		//    std::cout << "unexceptional error occured picture" << std::endl;
        //}
        }		
        break;		

		case nTp:
		{
		//try {
	        auto itr = std::find(listOfNtpParams.begin(), listOfNtpParams.end(), vm["string"].as<std::string>());
	        std::string cmd_set = "POST /ntp_setting/system.cgi?";
			cmd_set.append(*itr);
			cmd_set.append(" = ");
			std::int16_t paramIdx = get_index_from_list(listOfNtpParams, *itr);
			std::vector<std::string> valuesForParam = list_values_in_map_matching_option(mapOfNtpParamVals, paramIdx);   // get a list of allowable values for that param
            int val = 0;
            if ((paramIdx == 4) || (paramIdx == 1)) {
				auto itr2 = std::find(valuesForParam.begin(), valuesForParam.end(), vm["xxx"].as<std::string>());        // check that the correct param value was passed
				cmd_set.append(*itr2);
			} else if (paramIdx == 2) {
			    try {
						std::string s_val = vm["xxx"].as<std::string>();
                        val = std::atoi(s_val.c_str());       
			        } catch (exception& e) {
                        cout << e.what();
				        return -1;
                    } catch (...) {
			            std::cout << "unexceptional error occured ntp_setting" << std::endl;
                    }
                    if (valuesForParam.size() == 2) {
						int v_min = 0;
						int v_max = 1;
			            try {
							   //sort(valuesForParam.begin(), valuesForParam.end());
                               v_min = std::atoi(valuesForParam[0].c_str());      
                               v_max = std::atoi(valuesForParam[1].c_str());     								  
			            } catch (exception& e) {
                                cout << e.what();
				                return -1;
                        } catch (...) {
			                    std::cout << "unexceptional error occured Image_range_check" << std::endl;
                        }
						if ((val >= v_min) && (val <= v_max)) {
							cmd_set.append(std::to_string(val));
						}
                    }	
			} else {
			    std::string s_val = vm["xxx"].as<std::string>();
				if (s_val.length() <= 64) {
					cmd_set.append(s_val);					
				} else {
					return -2;
				}
			}
			cmd_set.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		        case HTTP_ONLY:
		        http_cmd(*s, host, cmd_set);
		        break;
		 
		        case HTTPS_TLS12:
		        http_cmd(*s_tls, host, cmd_set);
		        break;
		 
		        default:
		        break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_set, my_ip );
#else
		    set_msg_end_close_connection( cmd_set, my_ip );
#endif
			request_ostream << cmd_set;
#endif				
			std::cout << "\033[33m command : \033[0m" << cmd_set << std::endl;
		//} catch (boost::bad_any_cast &e) {
		//    std::cout << e.what() << std::endl;
		//} catch (...) {
		//    std::cout << "unexceptional error occured picture" << std::endl;
        //}	
        }		
        break;

	    case dot1x_config:
	    {
		//try {
	        auto itr = std::find(listOfdot1XParams.begin(), listOfdot1XParams.end(), vm["string"].as<std::string>());
	        std::string cmd_p = "POST /dot1x_config/dot1x.cgi?";
			cmd_p.append(*itr);
			cmd_p.append(" = ");
			std::int16_t paramIdx = get_index_from_list(listOfdot1XParams, *itr);
			std::vector<std::string> valuesForParam = list_values_in_map_matching_option(mapOfdot1XParamVals, paramIdx);   // get a list of allowable values for that param
            if (paramIdx == 1) {
				auto itr2 = std::find(valuesForParam.begin(), valuesForParam.end(), vm["xxx"].as<std::string>());        // check that the correct param value was passed
				cmd_p.append(*itr2);
			} else {
			    std::string s_val = vm["xxx"].as<std::string>();
				if ((paramIdx >= 3) &&  (s_val.length() <= 50)) {
					cmd_p.append(s_val);
                } else if (s_val.length() <= 250) {
					cmd_p.append(s_val);					
				} else {
					return -2;
				}
			}
			cmd_p.append(" HTTP/1.0\r\n\r\n");	
#if defined(TLS_SSH_VERSION)
	        switch (g_opertation_type) {
		        case HTTP_ONLY:
		        http_cmd(*s, host, cmd_p);
		        break;
		 
		        case HTTPS_TLS12:
		        http_cmd(*s_tls, host, cmd_p);
		        break;
		 
		        default:
		        break;
	        }
#else
			std::string my_ip = YOUR_CAM_IP_ADDR;
#if defined(KEEP_CONN_ALIVE)
            set_msg_end_keep_connection( cmd_p, my_ip );
#else
		    set_msg_end_close_connection( cmd_p, my_ip );
#endif
			request_ostream << cmd_p;
#endif		
			std::cout << "\033[33m command : \033[0m" << cmd_p << std::endl;
		//} catch (boost::bad_any_cast &e) {
		//    std::cout << e.what() << std::endl;
		//} catch (...) {
		//    std::cout << "unexceptional error occured picture" << std::endl;
        //}
	    }		
        break;
	
		default:
		std::cout << "The option was not valid" << std::endl;
		break;		
	}
/*
	
	*/	

#ifndef TLS_SSH_VERSION

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

    // convert the result from the http query to a string
    //
    std::string result = boost::asio::buffer_cast<const char*>(buffer.data());
    std::istrstream is1(result.c_str());
    std::cout << " http query result is :: " << result << std::endl;

    // look at it line by line
    //
    std::string str;
    std::istream is(&buffer);
    int numline=0;
    std::string s = "The data read back is \n";
	std::string ss,ss2;
    std::int16_t response_code_http = 0;

	// ==== this is for testing parsing the html reply =====
	//
	std::string result_parse_test = "HTTP/1.0 200 OK\r\n\r\n Content-Type: text/plain\r\n Content-Length: 345\r\n \r\n var <no1>=\"<63>\" \r\n var <paramno2>=\"<73>\" \r\n var <no3_para>=\"<675>\" \r\n";
    char split_char = 10;                                                                                            // its LF  (NL line feed, new line)
	char split_char2 = 62;                                                                                           // its >
    std::vector<std::string> lines = split_naive(result_parse_test, split_char);
	for ( auto l : lines ) {
	  ++numline;
	  if (numline > 4) {  
          split_char = 60;                                                                                           // its <
	      std::vector<std::string> values = split_naive(l, split_char);
	      if (values.size() > 2) {
	          std::vector<std::string> values2 = split_naive(values[1], split_char2);
	          std::string tagName = values2[0];
	          std::vector<std::string> values3 = split_naive(values[2], split_char2);
              std::string valueField = values3[0];	  
	          //s.push_back(l);  ,---- ??? no longer works it used to means the same anyway
			  s.append(l);
              s += " \n";
		      //ss.push_back(tagName);
		      //ss.push_back(" = ");
		      //ss.push_back(valueField);
		      ss.append(tagName);
		      ss.append(" = ");
		      ss.append(valueField);
		      ss += "\n";
	      }
      } else if ( numline == 1 ) {
		  split_char = 32;
	      std::vector<std::string> values4 = split_naive(l, split_char);
          if (values4.size() > 2) {
			  try {
                 response_code_http = std::atoi(values4[1].c_str());       // response code from http request
			  } catch (exception& e) {
                 cout << e.what();
				 return -1;
              } catch (...) {
			    std::cout << "unexceptional error occured split_naive" << std::endl;
              }
			  if ( response_code_http != 200 ) {
			     std::cout << " error response recieved" << std::endl;
				 return -1;
			  } 
		  }
      }			
	}
	std::cout << "\033[32m ============= test using naive_split ============== \033[0m \n";
	std::cout << s << std::endl;
	std::cout << ss << std::endl;
	
    // alternative way
    std::vector<std::pair<std::string, std::int16_t>> in_brackets = get_inside_brackets( result_parse_test );
    for (auto ele : in_brackets ) {
        std::cout << " data : " << ele.first << std::endl;
    }
	
	// alternative splitting the HTML using boost::optional::split
	//
    std::string delim ("\n");
	std::string delim2;
    std::list<string> list_string;	
    boost::split(list_string, result_parse_test, boost::is_any_of(delim));
	std::int16_t nl = 0;
    BOOST_FOREACH(std::string s1, list_string) {
	   ++nl;
       cout << "line is :" << s1 << endl;
	   if ( nl > 4 ) {  
          delim = '<';  		  // its <
		  delim2 = '>';
		  std::vector<std::string> values;
		  boost::split(values, s1, boost::is_any_of(delim));
	      if (values.size() > 2) {
	          std::vector<std::string> values2;
			  boost::split(values2, values[1], boost::is_any_of(delim2));
	          std::string tagName = values2[0];
	          std::vector<std::string> values3;
			  boost::split(values3, values[2], boost::is_any_of(delim2));			 
              std::string valueField = values3[0];	  
	          //s.push_back(s1);
			  s.append(s1);
              s += " \n";
		      //ss2.push_back(tagName);
		      //ss2.push_back(" = ");
		      //ss2.push_back(valueField);
		      ss2.append(tagName);
		      ss2.append(" = ");
		      ss2.append(valueField);
		      ss2 += "\n";
	      }
      } else if ( nl == 1 ) {
		  delim = ' ';
	      std::vector<std::string> values4;
		  boost::split(values4, s, boost::is_any_of(delim));
          if (values4.size() > 2) {
			  try {
                 response_code_http = std::atoi(values4[1].c_str());       // response code from http request
			  } catch (exception& e) {
                 cout << e.what();
				 return -1;
              } catch (...) {
			    std::cout << "unexceptional error occured split_boost" << std::endl;
              }
			  if ( response_code_http != 200 ) {
				 std::cout << " error response recieved" << std::endl;
				 return -1;
			  }
		  }
      }		   
    }
	std::cout << "\033[33m ============= test using boos::optional::split ============== \033[0m \n";
	std::cout << s << std::endl;
	std::cout << ss2 << std::endl;
	
    // is >> s --> will write response  We cant do as it contains 3 other lines at the start that we dont want so parse as shown
	//
	// http ok / content type / content length
	//
	numline = 0;
    while(buffer.size() !=0)
    {
      ++numline;
      std::getline(is, str, '\n');
      //std::cout << str << " " << numline << std::endl;
	  if (numline > 4) { 
          split_char = 60;                                                                                       // its <
	      std::vector<std::string> values = split_naive(str, split_char);
	      if (values.size() > 2) {
	          std::vector<std::string> values2 = split_naive(values[1], split_char2);
	          std::string tagName = values2[0];
	          std::vector<std::string> values3 = split_naive(values[2], split_char2);
              std::string valueField = values3[0];	  
	          //s.push_back(str);
	          s.append(str);
              s += " \n";
		      //ss.push_back(tagName);
		      //ss.push_back(" = ");
		      //ss.push_back(valueField);
		      ss.append(tagName);
		      ss.append(" = ");
		      ss.append(valueField);
		      ss += "\n";
	      }
      } else if ( numline == 1 ) {
		  split_char = 10;
	      std::vector<std::string> values4 = split_naive(str, split_char);
          if (values4.size() > 2) {
			  try {
                 response_code_http = std::atoi(values4[1].c_str());       // response code from http request
			  } catch (exception& e) {
                 cout << e.what();
				 return -1;
              }
			  if ( response_code_http != 200 ) {
				 std::cout << " error response recieved" << std::endl;
				 return -1;
			  }
		  }
      }		  
    }
	// print the responses from the html
	//
	std::cout << s << std::endl;
	std::cout << "==================================\n";
	std::cout << ss << std::endl;	
#endif
	
	return 0;
}
catch (exception& e)
{
    cout << e.what();
	return -1;
}
#ifndef __rfc3164_
#define __rfc3164_
// ==========================================================================================
//
// There are two types of packet formats in the syslog.
// One is in BSD format (RFC3164) and the other is in IETF format (RFC5424).
//
// ==========================================================================================
#define SYSLOG_PORT_NUM 514
#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <climits>
#include "mavlink_msg_statustext.h"

typedef enum {

   kernel = 0,
   user_level = 1,
   mail_system = 2,
   system_daemons = 3,
   security_authorization = 4,
   internal_syslogd = 5,
   line_printer_subsystem = 6,
   network_news_subsystem = 7,
   UUCP_subsystem = 8,
   clock_daemon_cron = 9,
   authpriv = 10,
   FTP_daemon = 11,
   NTP_subsystem = 12,
   log_audit = 13,
   log_alert = 14,
   clock_daemon_at = 15,
   local_1 = 16,
   local_2 = 17,
   local_3 = 18,
   local_4 = 19,
   local_5 = 20,
   local_6 = 21,
   local_7 = 22

} rfc3164_facility_e;

typedef enum {

   emerg = 0,    // system unavailable 
   alert = 1,    // immediate response is required 
   crit = 2,     // fatal condition 
   err = 3,      // error 
   warn = 4,     // warning 
   notice = 5,   // important information 
   info = 6,     // normal information 
   debug = 7    // debug information 

} rfc3164_severity_e;

//
// sendStatusTextMessage : in chunks of 50 chars....
//
int sendStatusTextMessage( std::string my_msg, rfc3164_facility_e facil, rfc3164_severity_e sevi, std::uint16_t& id_start, std::uint8_t sys_id  )
{

	mavlink_statustext_t setValue;
	int sent_bytes = 0;
	int len = 0;
	setValue.severity = (facil*8) + sevi;                                                                                     // as per (RFC5424) 
//	mavlink_message_t message;
	std::uint8_t buf[MAVLINK_MSG_ID_STATUSTEXT_LEN];

    char* cstr = new char[my_msg.size() + 1];                                                                                // allocate char* size of string
    std::char_traits<char>::copy(cstr, my_msg.c_str(), my_msg.size() + 1);                                                   // copy the string to the char*
    std::cout << "len " << my_msg.length() << "into " << sizeof(setValue.text) << std::endl;
    int n = 0;
	setValue.id = id_start;
	setValue.chunk_seq = 0; 
    while(n < my_msg.length()) {
        if ((my_msg.length()-n) < sizeof(setValue.text)) {                                                                    // its a final part-chunk
            std::strncpy(&setValue.text[0],cstr+n,(my_msg.length()-n));
            setValue.text[my_msg.length()-n] = '\0';
            printf(" STATUSTEXT partial chunk sent %s\n",setValue.text);
            n += (my_msg.length()-n)+1;
        } else {
            std::strncpy(&setValue.text[0],cstr+n,sizeof(setValue.text));                                                     // send a full chunk sizeof 50 .text[]
            printf(" STATUSTEXT chunk sent %s\n",setValue.text);
            n += sizeof(setValue.text);
        }
		++setValue.chunk_seq % UCHAR_MAX;
		++setValue.id % UINT_MAX;
	    //len = mavlink_msg_statustext_encode(sys_id, MAV_COMP_ID_CAMERA, &message, &setValue);                                   // encode
	    //len = mavlink_msg_to_send_buffer(buf, &message);
	    printf("storage info bytes sent %d\n", len);
	    // now send buf where you want to
	    sent_bytes  += sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(gcAddr));
    }
    delete [] cstr;                                                                                                             // delete the temporary char* to the string
	id_start = setValue.chunk_seq;
	return sent_bytes;
		
}
		
#endif


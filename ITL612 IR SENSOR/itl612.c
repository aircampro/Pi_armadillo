#ifndef  __itl612C__
#define  __itl612C__
// •••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
//    itl612.c : WuHan Global Infa Red Sesnor Camera (IR/Temperature sensor) protocol stack
//    An API container and command library for the WuHan ITL612
//
//    Version : @(#) 1.0
//    Copyright (C) 2022 AiRobot Dynamics Walkerburn Scotland
//
//    Compiles with Intel C/C++ compilers :- /opt/intel/oneapi/compiler/2022.2.1/linux/bin/icx -o test_ir test_ir.c -Wall -lusb -O2 -qopt-report
//                                           /opt/intel/oneapi/compiler/2022.2.1/linux/bin/intel64/icpc -o test_ir test_ir.c -Wall -lusb -O2 -qopt-report
//                  GNU   C/C++ compilers :- gcc -o test_ir test_ir.c -Wall -O2 -lusb
//                                           g++-8 -o test_ir test_ir.c -Wall -lusb -O2     
//                  Apple Clang           :- clang -o test_ir test_ir.c -Wall -lusb -O2                                                     
//    
//
// •••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
/**
* @file itl612.h
* @brief An API container and command library for the WuHan ITL612
* @author AiRobot Dynamics Walkerburn Scotland
* @date 11-11-2022
* @details WuHan Global Infa Red Sesnor Camera (IR/Temperature sensor) protocol stack
*/
// for help on using doxygen
// https://www.yokoweb.net/2020/06/29/ubuntu-linux-doxygen-install/
// https://qiita.com/wakaba130/items/faa6671bd5c954cb2d02
//
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// you can use this c++ header if you want to write c++ std::uint8_t for example
//#include <string>
//#include <boost/cstdint.hpp>
//#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "itl612.h"

// Function : To compose a set-up message
//
/**
* @brief compose_setup_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_SetUp_Option_CC_e opt, uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a setup query which can be copied to a char buffer and sent to the IR device
*/
void compose_setup_msg( ITL612_DataFrameQuery_t* msg, ITL612_SetUp_Option_CC_e opt, uint8_t cw4 )
{
	uint8_t chk_byte = 0x07u;                                       // calculate msg xor without STX and ETX starts at length
	uint8_t value_clamped = 0;
    ITL612_PageNum_CC_e page = page1_set;	
	ITL612_FuncClass_CC_e fc = setup;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
	if (opt == auto_comp_time) {
		SET_ATC(value_clamped,cw4);
		msg->CmdWord4 = value_clamped;
	} else {
        msg->CmdWord4 = cw4;		
	}
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord4;
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a slash message (shutter control ?)
//
/**
* @brief compose_slash_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Slash_Option_CC_e opt, uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a slash method query which can be copied to a char buffer and sent to the IR device
*/
void compose_slash_msg( ITL612_DataFrameQuery_t* msg, ITL612_Slash_Option_CC_e opt, uint8_t cw4 )
{
	uint8_t chk_byte = 0x7u;                                       // calculate msg xor without STX and ETX
    ITL612_PageNum_CC_e page = page2;	
	ITL612_FuncClass_CC_e fc = slash;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
    msg->CmdWord4 = cw4;	
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord4;
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a video page 1 message
//
/**
* @brief compose_video_page2_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p2_CC_e opt, uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a video page2 query which can be copied to a char buffer and sent to the IR device
*/
void compose_video_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p2_CC_e opt, uint8_t cw4 )
{
	uint8_t chk_byte = 0x7u;                                       // calculate msg xor without STX and ETX
    ITL612_PageNum_CC_e page = page2;	
	ITL612_FuncClass_CC_e fc = video;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
    msg->CmdWord4 = cw4;	
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord4;
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a video page 3 message
//
/**
* @brief compose_video_page3_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p3_CC_e opt, uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a video page3 query which can be copied to a char buffer and sent to the IR device
*/
void compose_video_page3_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p3_CC_e opt, uint8_t cw4 )
{
	uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
	uint8_t value_clamped = 0;
    ITL612_PageNum_CC_e page = page3;	
	ITL612_FuncClass_CC_e fc = video;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
	if (opt == brightness) {
		SET_BRIGHT(value_clamped,cw4);
		msg->CmdWord4 = value_clamped;
	} else if (opt == contrast) {
		SET_CONTRAST(value_clamped,cw4);
		msg->CmdWord4 = value_clamped;
	} else if (opt == enhanced_detail) {
		SET_EDG(value_clamped,cw4);
		msg->CmdWord4 = value_clamped;		
	} else {
        msg->CmdWord4 = cw4;		
	}
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord4;	
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a application page 2 message
//
/**
* @brief compose_app_page2_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p2_CC_e opt, uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page2 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p2_CC_e opt, uint16_t cw4 )
{
	uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
    ITL612_PageNum_CC_e page = page2;	
	ITL612_FuncClass_CC_e fc = app;
	uint8_t Cword = 0;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;	
	if (opt == cursor_coordinate_X) {
        SET_CW3(Cword,cw4);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,cw4);
		msg->CmdWord4 = Cword;
	} else if (opt == cursor_coordinate_Y) {
        SET_CW3(Cword,cw4);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,cw4);
		msg->CmdWord4 = Cword;	
	} else {
        msg->CmdWord4 = cw4;		
	}
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord3;	
	chk_byte ^= msg->CmdWord4;	
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a application page 4 message
//
/**
* @brief compose_app_page4_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p4_CC_e opt, uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page4 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page4_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p4_CC_e opt, uint16_t cw4 )
{
	uint8_t chk_byte = 0u;                                       // calculate msg xor without STX and ETX
	uint8_t Cword = 0u;
	uint16_t clampV = 0u;
    ITL612_PageNum_CC_e page = page4;	
	ITL612_FuncClass_CC_e fc = app;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
	if (opt == region_UL_X) {
		SET_RAX(clampV,cw4);
        SET_CW3(Cword,clampV);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;
	} else if (opt == region_UL_Y) {
		SET_RAY(clampV,cw4);
        SET_CW3(Cword,clampV);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;	
	} else {
        msg->CmdWord4 = cw4;		
	}	
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord3;	
	chk_byte ^= msg->CmdWord4;	
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a application page 6 message
//
/**
* @brief compose_app_page6_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page6 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page6_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, uint16_t cw4 )
{
	uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
	uint8_t Cword = 0u;
	uint16_t clampV = 0u;
    ITL612_PageNum_CC_e page = page6_hst;	
	ITL612_FuncClass_CC_e fc = app;
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
	if (opt == Isotherm_thres_Up) {
		SET_IT(clampV,cw4);
        SET_CW3(Cword,clampV);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;
	} else if (opt == Isotherm_thres_Lo) {
		SET_IT(clampV,cw4);
        SET_CW3(Cword,clampV);
		msg->CmdWord3 = Cword;
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;	
	} else {
        msg->CmdWord4 = cw4;		
	}
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord3;	
	chk_byte ^= msg->CmdWord4;	
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

// Function : To compose a temperature message
//
/**
* @brief compose_temperature_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a temperature query which can be copied to a char buffer and sent to the IR device
*/
void compose_temperature_msg( ITL612_DataFrameQuery_t* msg, ITL612_Temp_Option_p1_CC_e opt, uint8_t cw4 )
{
	uint8_t chk_byte = 0;                                       //! checksum byte is calculated from msg bytes xor'ed without STX1/2 and ETX  
	uint8_t Cword = 0u;
	uint16_t clampV = 0u;
    ITL612_PageNum_CC_e page = page1_set;	                         //! code page sets to 1 
	ITL612_FuncClass_CC_e fc = temperature;                          //! function class is set to temperature 
	msg->FrameHeaderSTX1 = ITL612_IR_SENSOR_STX1;
	msg->FrameHeaderSTX2 = ITL612_IR_SENSOR_STX2;
    msg->Length = 0x07u;
    msg->FuncCategory = fc;
    msg->Page = page;
    msg->Option = opt;	
    msg->CmdWord1 = 0u;
    msg->CmdWord2 = 0u;
    msg->CmdWord3 = 0u;
	if (opt == Distance) {
		SET_TDIST(clampV,cw4);
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;
	} else if (opt == Emissivity) {
		SET_TEMIS(clampV,cw4);
        SET_CW4(Cword,clampV);
		msg->CmdWord4 = Cword;		
	} else {
        msg->CmdWord4 = cw4;		
	}
	chk_byte = msg->Length;
	chk_byte ^= msg->FuncCategory;                                  // fast to just XOR the bytes here
	chk_byte ^= msg->Page;	
	chk_byte ^= msg->Option;
	chk_byte ^= msg->CmdWord1;	
	chk_byte ^= msg->CmdWord2;	
	chk_byte ^= msg->CmdWord3;	
	chk_byte ^= msg->CmdWord4;	
	msg->Checksum = chk_byte;
	msg->FrameEnd = ITL612_IR_SENSOR_ETX;
}

/**
* @brief do_checksum
* @param[in] uint8_t *buf, uint8_t *chksum
* @param[out] uint8_t *chksum
* @return void
* @details perform checksum on incoming message buffer
*/
void do_checksum( uint8_t *buf, size_t b_len, uint8_t *chksum ) {
	 *chksum = 0;
     for ( size_t i = ITL612_CC_LEN_POS; i < (b_len-2) ; i++) {
		 printf("chksum %02X = %lu %lu", buf[i],i, sizeof(buf));
		 *chksum ^= buf[i];
	 }
	 printf("chksum tot = %02X\n", *chksum);
}

/**
* @brief copy_query_to_send_buffer
* @param[in] ITL612_DataFrameQuery_t* msg, size_t len
* @param[out] char* buf,
* @return void
* @details Compose a temperature query which can be copied to a char buffer and sent to the IR device
*/
inline static void copy_query_to_send_buffer( char* buf, ITL612_DataFrameQuery_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__MEMCPY_S)
   errno_t e = memcpy_s((void*) buf, len, (void*) msg, len);
   printf("memcpy_s error %d\n",e);
#else   
   memcpy((void*) buf, (void*) msg, len);
#endif
}	

/**
* @brief copy_rcv_frame_to_confirm
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvFrameConf_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_confirm( char* buf, size_t b_len, ITL612_RcvFrameConf_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else   
   if (b_len >= len)
       memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_query
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvQueryReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_query( char* buf, size_t b_len, ITL612_RcvQueryReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else   
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_status
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvStatusReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_status( char* buf, size_t b_len, ITL612_RcvStatusReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else  
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_su
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvSetUpReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_su( char* buf, size_t b_len, ITL612_RcvSetUpReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else   
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_vid
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvVideoReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_vid( char* buf, size_t b_len, ITL612_RcvVideoReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else   
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_app
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvAppReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_app( char* buf, size_t b_len, ITL612_RcvAppReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else   
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_advapp
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvAdvAppReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_advapp( char* buf, size_t b_len, ITL612_RcvAdvAppReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else 
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_menu
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvMenuReturn_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_menu( char* buf, size_t b_len, ITL612_RcvMenuReturn_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else  
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_hottrack
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvHotTrackP3Return_t* msg,
* @return void
* @details Read data recieved from hot track message
*/
inline static void copy_rcv_frame_to_hottrack( char* buf, size_t b_len, ITL612_RcvHotTrackP3Return_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else 
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_thermogp1
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvThermographyReturnPage1_t* msg,
* @return void
* @details Read data recieved from thermography page 1 message
*/
inline static void copy_rcv_frame_to_thermogp1( char* buf, size_t b_len, ITL612_RcvThermographyReturnPage1_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else  
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}

/**
* @brief copy_rcv_frame_to_thermogp2
* @param[in] char* buf, size_t len
* @param[out] ITL612_RcvThermographyReturnPage2_t* msg,
* @return void
* @details Read data recieved from thermography page 2 message
*/
inline static void copy_rcv_frame_to_thermogp2( char* buf, size_t b_len, ITL612_RcvThermographyReturnPage2_t* msg, size_t len ) {
    if ((buf == NULL) || (msg == NULL)) {
		return;
	}
#if defined(__STDC_WANT_LIB_EXT1__)
   errno_t e = memcpy_s((void*) msg, len, (void*) buf, len);
   printf("memcpy_s error %d\n",e);
#else 	   
   if (b_len >= len)	
      memcpy((void*) msg, (void*) buf, len);
#endif
}


/**
* @brief Safe strncpy() function
* @param dest destination string
* @param size_dest Size of destination string buffer
* @param src source string
* @param n number of characters to copy
* @return void char*
* The value of @return dest 
*/
inline static char *safe_strncpy( char *dest, size_t size_dest, const char *src, size_t n )
{
    n ++;
    if ( n < size_dest ) size_dest = n;
    if ( 0 < size_dest ) {
        const size_t m = size_dest - 1;
        strncpy(dest,src,m);
        dest[m] = '\0';
    }
    return dest;
}

/**
* @brief pack_rcv_frames() function
* @param char* buf_in = received frame buffer
* @param ITL612_RcvPacket_u *rcv_container = union of structures containg every possible input frame
* @return int16_t denoting what type was received
* @details General reciever function for ITL612  
*/
int16_t pack_rcv_frames( char* buf_in, size_t b_len, ITL612_RcvPacket_u *rcv_container ) {

    uint8_t chksm = 0;
    int16_t ret = 0;
	int16_t unique_msg_id = 0;
	printf("in receive packer");
	if (buf_in == NULL) {                                                                    // NULL pointr passed 
	    ret = -0x7FFF;
	} else if (ITL612_IR_RCV_CONF_LEN > sizeof(buf_in))  {                                   // receive buffer too small
	    ret = -255;
	} else if ( buf_in[ITL612_CC_LEN_POS] == ITL612_IR_MSG_CONF_LEN ) {                      // check for simple confirm message 1st 
		memset((void*)&rcv_container->conf_frame, 0, ITL612_IR_RCV_CONF_LEN);
		copy_rcv_frame_to_confirm( buf_in, b_len, &rcv_container->conf_frame, ITL612_IR_RCV_CONF_LEN );
		do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		ret = ITL612_IR_MSG_CONF_LEN * ((buf_in[ITL612_IR_RCV_CONF_LEN-2] == chksm) ? 1 : -1);		
	} else {
        unique_msg_id = ((buf_in[ITL612_CC_LEN_POS] + buf_in[ITL612_CC_FuncClass_POS] + buf_in[ITL612_CC_Page_POS]) * buf_in[ITL612_CC_FuncClass_POS]);
	    switch (unique_msg_id) {
	
	        case ITL612_IR_STAT_LFP:
		    memset((void*)&rcv_container->status_frame, 0, ITL612_IR_RCV_STATUS_LEN);
		    copy_rcv_frame_to_status( buf_in, b_len, &rcv_container->status_frame, ITL612_IR_RCV_STATUS_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_STAT_LFP * ((buf_in[ITL612_IR_RCV_STATUS_LEN-2] == chksm) ? 1 : -1);
			printf("status\n");
		    break;
		 
	        case ITL612_IR_SET_LFP:
		    memset((void*)&rcv_container->setup_frame, 0, ITL612_IR_RCV_SETUP_LEN);
		    copy_rcv_frame_to_su( buf_in, b_len, &rcv_container->setup_frame, ITL612_IR_RCV_SETUP_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_SET_LFP * ((buf_in[ITL612_IR_RCV_SETUP_LEN-2] == chksm) ? 1 : -1);
			printf("set");
		    break;
		 
	        case ITL612_IR_VIDEO_LFP:
		    memset((void*)&rcv_container->video_frame, 0, ITL612_IR_RCV_VIDEO_LEN);
		    copy_rcv_frame_to_vid( buf_in, b_len, &rcv_container->video_frame, ITL612_IR_RCV_VIDEO_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_VIDEO_LFP * ((((uint8_t)buf_in[ITL612_IR_RCV_VIDEO_LEN-2]) == chksm) ? 1 : -1);
		    break;
		 
	        case ITL612_IR_APP_LFP:
		    memset((void*)&rcv_container->app_frame, 0, ITL612_IR_RCV_APP_LEN);
		    copy_rcv_frame_to_app( buf_in, b_len, &rcv_container->app_frame, ITL612_IR_RCV_APP_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_APP_LFP * ((buf_in[ITL612_IR_RCV_APP_LEN-2] == chksm) ? 1 : -1);
			printf("application");
		    break;
		 
	        case ITL612_IR_ADVAPP_LFP:
		    memset((void*)&rcv_container->app_adv_frame, 0, ITL612_IR_RCV_ADVAPP_LEN);
		    copy_rcv_frame_to_advapp( buf_in, b_len, &rcv_container->app_adv_frame, ITL612_IR_RCV_ADVAPP_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_ADVAPP_LFP * ((buf_in[ITL612_IR_RCV_ADVAPP_LEN-2] == chksm) ? 1 : -1);
			printf("advanced app");
		    break;
		 
	        case ITL612_IR_MENU_LFP:
		    memset((void*)&rcv_container->hottrk1_frame, 0, ITL612_IR_RCV_MENU_LEN);
		    copy_rcv_frame_to_menu( buf_in, b_len, &rcv_container->hottrk1_frame, ITL612_IR_RCV_MENU_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_MENU_LFP * ((buf_in[ITL612_IR_RCV_MENU_LEN-2] == chksm) ? 1 : -1);
			printf("menu hottrk1");
		    break;
		 
	        case ITL612_IR_HOTTRK_LFP:
		    memset((void*)&rcv_container->hottrk2_frame, 0, ITL612_IR_RCV_HOTTRK_LEN);
		    copy_rcv_frame_to_hottrack( buf_in, b_len, &rcv_container->hottrk2_frame, ITL612_IR_RCV_HOTTRK_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_HOTTRK_LFP * ((buf_in[ITL612_IR_RCV_HOTTRK_LEN-2] == chksm) ? 1 : -1);
			printf("hot track2");
		    break;
		 
	        case ITL612_IR_THERM_LFP:
		    memset((void*)&rcv_container->therm1_frame, 0, ITL612_IR_RCV_THERM_LEN);
		    copy_rcv_frame_to_thermogp1( buf_in, b_len, &rcv_container->therm1_frame, ITL612_IR_RCV_THERM_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_THERM_LFP * ((buf_in[ITL612_IR_RCV_THERM_LEN-2] == chksm) ? 1 : -1);
			printf("thermo1");
		    break;
		 
	        case ITL612_IR_THERMP2_LFP:
		    memset((void*)&rcv_container->therm2_frame, 0, ITL612_IR_RCV_THERMP2_LEN);
		    copy_rcv_frame_to_thermogp2( buf_in, b_len, &rcv_container->therm2_frame, ITL612_IR_RCV_THERMP2_LEN );
		    do_checksum( (uint8_t*) buf_in, b_len, &chksm );
		    ret = ITL612_IR_THERMP2_LFP * ((buf_in[ITL612_IR_RCV_THERMP2_LEN-2] == chksm) ? 1 : -1);
			printf("thermo2");
		    break;
		}
		 
	}
	return ret;

}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  

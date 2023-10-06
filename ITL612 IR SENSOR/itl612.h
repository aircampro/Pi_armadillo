#ifndef  __itl612__
#define  __itl612__
// •••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
//    itl612.h : WuHan Global Infa Red Sesnor Camera (IR/Temperature sensor) protocol stack
//    An API container and command library for the WuHan ITL612
//
//    Version : @(#) 1.0
//    Copyright (C) 2022 AiRobot Dynamics Walkerburn Scotland
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
//#include <iostream>
//#include <string>
//#include <strstream>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
//#include <boost/cstdint.hpp>
//#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
* ITL612PACKED : Is a defined as the packing format needed for your CPU and compiler to pack the structure as per the structure definition.
*/
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ITL612PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) // seems no longer works with this using g++-8 ALIGNED(1)
#elif _PIC32_
  #define ITL612PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1) // pic32 gcc compiler
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ITL612PACKED __attribute__((packed))                                  /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ITL612PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ITL612PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ITL612PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

// #define __STDC_WANT_LIB_EXT1__ 1                                             if you have memcpy_s

/**
* conv_u : Is a union for converting 32bit integer to/from float 
*/
typedef union {
	float f;
	uint32_t u32;
} conv_u;                                                                      // converts float to u32 integer and vice versa

// •••••••••••••••••••••••••••••••••••••••••• Data Frame Structure ••••••••••••••••••••••••••••••••••••••••••
//
// ----------------------------------------- send messages --------------------------------------------------
/**
* ITL612_DataFrameQuery_t : Is the structure to send a request to the ITL612 IR device
*/
#define ITL612_DATA_FRAME_QUERY_LEN 12u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t Option;                                     // for set-up message ITL612_SetUp_Option_CC_e we could use this here but i chose not to for embedded C compatibility
        uint32_t CmdWord1 : 8u;
        uint32_t CmdWord2 : 8u;
        uint32_t CmdWord3 : 8u;
        uint32_t CmdWord4 : 8u;
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_DataFrameQuery_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t Option;
        uint32_t CmdWord1 : 8u;
        uint32_t CmdWord2 : 8u;
        uint32_t CmdWord3 : 8u;
        uint32_t CmdWord4 : 8u;
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_DataFrameQuery_t;
#endif

// ---------------------------------- receive messages --------------------------------------------------
/**
* @def ITL612_IR_RCV_CONF_LEN
* @brief length of confirm message reply
* @details length field of confirmation reply message
*/
#define ITL612_IR_RCV_CONF_LEN 6u
#define ITL612_IR_MSG_CONF_LEN 1u
/**
* ITL612_RcvFrameConf_t : Is the structure which packages the confirmation sent from the ITL612 IR device
*/
#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t RcvConf;
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvFrameConf_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t RcvConf;
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvFrameConf_t;
#endif

/**
* ITL612_RcvQueryReturn_t : Is the structure which packages the query return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_QUERY_LEN 24u
#define ITL612_IR_MSG_QUERY_LEN 19u

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t Option[17u];
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvQueryReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t Option[17u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvQueryReturn_t;
#endif

/**
* ITL612_RcvStatusReturn_t : Is the structure which packages the status return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_STATUS_LEN 24u
#define ITL612_IR_MSG_STATUS_LEN 19u
#define ITL612_IR_STAT_FC 0
#define ITL612_IR_STAT_PAGE 0
#define ITL612_IR_STAT_LFP (ITL612_IR_MSG_STATUS_LEN+ITL612_IR_STAT_FC+ITL612_IR_STAT_PAGE) // which is 19

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t ModuleID;
        uint8_t ComObjID;		
        uint8_t Year;
        uint8_t Month;	
		uint8_t Day;
        uint16_t FocalSpotTempHi : 8;
        uint16_t FocalSpotTempLo : 8;
        uint8_t videoSystem;	
        uint8_t resolutionID;	
		uint32_t machineID;
		uint8_t reserve[4u];
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvStatusReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t ModuleID;
        uint8_t ComObjID;		
        uint8_t Year;
        uint8_t Month;	
		uint8_t Day;
        uint16_t FocalSpotTempHi : 8;
        uint16_t FocalSpotTempLo : 8;
        uint8_t videoSystem;	
        uint8_t resolutionID;	
		uint32_t machineID;
		uint8_t reserve[4u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvStatusReturn_t;
#endif

/**
* ITL612_RcvSetUpReturn_t : Is the structure which packages the setup instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_SETUP_LEN 24u
#define ITL612_IR_MSG_SETUP_LEN 19u
#define ITL612_IR_SET_FC 1
#define ITL612_IR_SET_PAGE 0
#define ITL612_IR_SET_LFP ((ITL612_IR_MSG_SETUP_LEN+ITL612_IR_SET_FC+ITL612_IR_SET_PAGE)*ITL612_IR_SET_FC)  // 20*1

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t autoCompTime;
		uint8_t imageFreeze;
		uint8_t imageType;
		uint8_t tempCalib;
        uint8_t ShutterMode;
		uint8_t ShutterPos;
		uint8_t lowNoiseMode;
		uint8_t reserved[9u];
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvSetUpReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t autoCompTime;
		uint8_t imageFreeze;
		uint8_t imageType;
		uint8_t tempCalib;
        uint8_t ShutterMode;
		uint8_t ShutterPos;
		uint8_t lowNoiseMode;
		uint8_t reserved[9u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvSetUpReturn_t;
#endif

/**
* ITL612_RcvVideoReturn_t : Is the structure which packages the video instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_VIDEO_LEN 24u
#define ITL612_IR_MSG_VIDEO_LEN 19u
#define ITL612_IR_VIDEO_FC 2u
#define ITL612_IR_VIDEO_PAGE 1u
#define ITL612_IR_VIDEO_LFP ((ITL612_IR_MSG_VIDEO_LEN+ITL612_IR_VIDEO_FC+ITL612_IR_VIDEO_PAGE)*ITL612_IR_VIDEO_FC)  // 22*2

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t cmdOption1;
        uint8_t cmdOption2;	
        uint8_t cmdOption3;
        uint8_t cmdOption4;		
        uint8_t cmdOption5;	
        uint8_t reserved[11u];		
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvVideoReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t cmdOption1;
        uint8_t cmdOption2;	
        uint8_t cmdOption3;
        uint8_t cmdOption4;		
        uint8_t cmdOption5;	
        uint8_t reserved[11u];		
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvVideoReturn_t;
#endif

/**
* ITL612_RcvAppReturn_t : Is the structure which packages the application instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_APP_LEN 24u
#define ITL612_IR_MSG_APP_LEN 19u
#define ITL612_IR_APP_FC 2u
#define ITL612_IR_APP_PAGE 2u
#define ITL612_IR_APP_LFP ((ITL612_IR_MSG_APP_LEN+ITL612_IR_APP_FC+ITL612_IR_APP_PAGE)*ITL612_IR_APP_FC)  // 23*2

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t anti_straition;
        uint8_t brightness;
        uint8_t contrast;
        uint8_t enhanced_detail;
		uint8_t enhance_algo;
        uint8_t noise_reduction;
		uint8_t DRC_mode;
		uint8_t reserve[10u];
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvAppReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t anti_straition;
        uint8_t brightness;
        uint8_t contrast;
        uint8_t enhanced_detail;
		uint8_t enhance_algo;
        uint8_t noise_reduction;
		uint8_t DRC_mode;
		uint8_t reserve[10u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvAppReturn_t;
#endif

/**
* ITL612_RcvAdvAppReturn_t : Is the structure which packages the advanced application instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_ADVAPP_LEN 24u
#define ITL612_IR_MSG_ADVAPP_LEN 19u
#define ITL612_IR_ADVAPP_FC 3u
#define ITL612_IR_ADVAPP_PAGE 1u
#define ITL612_IR_ADVAPP_LFP ((ITL612_IR_MSG_ADVAPP_LEN+ITL612_IR_ADVAPP_FC+ITL612_IR_ADVAPP_PAGE)*ITL612_IR_ADVAPP_FC)  // 23*3

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t reserved;
		uint16_t cursorX;
        uint16_t cursorY;
        uint16_t ADvalueCursor;		
		uint8_t reserve[8u];
		uint16_t cursorPointY16;
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvAdvAppReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t reserved;
		uint16_t cursorX;
        uint16_t cursorY;
        uint16_t ADvalueCursor;		
		uint8_t reserve[8u];
		uint16_t cursorPointY16;
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvAdvAppReturn_t;
#endif

/**
* ITL612_RcvMenuReturn_t : Is the structure which packages the menu return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_MENU_LEN 38u
#define ITL612_IR_MSG_MENU_LEN 40u
#define ITL612_IR_MENU_FC 3u
#define ITL612_IR_MENU_PAGE 1u
#define ITL612_IR_MENU_LFP ((ITL612_IR_MSG_MENU_LEN+ITL612_IR_MENU_FC+ITL612_IR_MENU_PAGE)*ITL612_IR_MENU_FC)  // 44*3

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;   
        uint8_t FrameHeaderSTX2;   
        uint8_t Length;            
        uint8_t FuncCategory;      
        uint8_t Page;              
        uint8_t option1;           
		uint16_t option2;          
		uint16_t option3;          
		uint16_t option4;          
		uint16_t option5;          
		uint8_t preamble[7u];
		uint16_t coldSpotCoordX;   
		uint16_t coldSpotCoordY;  
		uint16_t coldSpotTempY16;  
		uint16_t hotSpotCoordX;    
		uint16_t hotSpotCoordY;    
		uint16_t hotSpotTempY16;   
		uint16_t cursorSpotCoordX;  
		uint16_t cursorSpotCoordY;  
		uint16_t cursorSpotTempY16; 
		uint16_t regionalAvgTempY16; 
        uint8_t reserve[2u];         
        uint8_t Checksum;           
        uint8_t FrameEnd;         
} ITL612_RcvMenuReturn_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t option1;
		uint16_t option2;
		uint16_t option3;
		uint16_t option4;
		uint16_t option5;
		uint8_t preamble[7u];
		uint16_t coldSpotCoordX;
		uint16_t coldSpotCoordY;
		uint16_t coldSpotTempY16;
		uint16_t hotSpotCoordX;
		uint16_t hotSpotCoordY;
		uint16_t hotSpotTempY16;
		uint16_t cursorSpotCoordX;
		uint16_t cursorSpotCoordY;
		uint16_t cursorSpotTempY16;
		uint16_t regionalAvgTempY16;
        uint8_t reserve[2u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvMenuReturn_t;
#endif

/**
* ITL612_RcvHotTrackP3Return_t : Is the structure which packages the hotspot tracking page 3 return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_HOTTRK_LEN 28u
#define ITL612_IR_MSG_HOTTRK_LEN 25u
#define ITL612_IR_HOTTRK_FC 3u
#define ITL612_IR_HOTTRK_PAGE 1u
#define ITL612_IR_HOTTRK_LFP ((ITL612_IR_MSG_HOTTRK_LEN+ITL612_IR_HOTTRK_FC+ITL612_IR_HOTTRK_PAGE)*ITL612_IR_HOTTRK_FC)  // 28*3

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t reserved1[11u];
		uint8_t isotherm;
		uint8_t displayMode;
		uint16_t ULIsoThermThres;
		uint16_t LLIsoThermThres;
		uint8_t reserved2[9u];
		uint8_t spectralRange;
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvHotTrackP3Return_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t reserved1[11u];
		uint8_t isotherm;
		uint8_t displayMode;
		uint16_t ULIsoThermThres;
		uint16_t LLIsoThermThres;
		uint8_t reserved2[9u];
		uint8_t spectralRange;
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvHotTrackP3Return_t;
#endif

/**
* ITL612_RcvThermographyReturnPage1_t : Is the structure which packages the thermography return sent from the ITL612 IR device page1
*/
#define ITL612_IR_RCV_THERM_LEN 28u
#define ITL612_IR_MSG_THERM_LEN 25u
#define ITL612_IR_THERM_FC 4u
#define ITL612_IR_THERM_PAGE 0u
#define ITL612_IR_THERM_LFP ((ITL612_IR_MSG_THERM_LEN+ITL612_IR_THERM_FC+ITL612_IR_THERM_PAGE)*ITL612_IR_THERM_FC)  // 29*4

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t distance;
		uint8_t emissivity;
        uint8_t temperatLocation;
		uint8_t temperatUnits;
		uint8_t reserved[2u];
		uint16_t coordXpt1;
        uint16_t coordYpt1;
        uint16_t temperature_pt1;
		uint16_t coordXpt2;
        uint16_t coordYpt2;
        uint16_t temperature_pt2;
        uint16_t reflectedTemp;
        uint8_t humidity;
        uint8_t tempRange;	
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvThermographyReturnPage1_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint8_t distance;
		uint8_t emissivity;
        uint8_t temperatLocation;
		uint8_t temperatUnits;
		uint8_t reserved[2u];
		uint16_t coordXpt1;
        uint16_t coordYpt1;
        uint16_t temperature_pt1;
		uint16_t coordXpt2;
        uint16_t coordYpt2;
        uint16_t temperature_pt2;
        uint16_t reflectedTemp;
        uint8_t humidity;
        uint8_t tempRange;	
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvThermographyReturnPage1_t;
#endif

/**
* ITL612_RcvThermographyReturnPage2_t : Is the structure which packages the thermography return sent from the ITL612 IR device page2
*/
#define ITL612_IR_RCV_THERMP2_LEN 30u
#define ITL612_IR_MSG_THERMP2_LEN 25u
#define ITL612_IR_THERMP2_FC 4u
#define ITL612_IR_THERMP2_PAGE 1u
#define ITL612_IR_THERMP2_LFP ((ITL612_IR_MSG_THERMP2_LEN+ITL612_IR_THERMP2_FC+ITL612_IR_THERMP2_PAGE)*ITL612_IR_THERMP2_FC)  // 30*4

#if defined(D_FT900)
typedef struct ITL612PACKED {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint16_t loBlackBodyTemp;
		uint16_t hiBlackBodyTemp;
		uint16_t singleBlackBodyTemp;
		uint8_t reserved[17u];
        uint8_t Checksum;
        uint8_t FrameEnd;
} ITL612_RcvThermographyReturnPage2_t;
#else
ITL612PACKED(
typedef struct {
        uint8_t FrameHeaderSTX1;
        uint8_t FrameHeaderSTX2;
        uint8_t Length;
        uint8_t FuncCategory;
        uint8_t Page;
        uint16_t loBlackBodyTemp;
		uint16_t hiBlackBodyTemp;
		uint16_t singleBlackBodyTemp;
		uint8_t reserved[17u];
        uint8_t Checksum;
        uint8_t FrameEnd;
}) ITL612_RcvThermographyReturnPage2_t;
#endif

// ••••••••••••••••••••••••••••••••••••••••• STX •••••••••••••••••••••••••••••••••
//
// define the start commands and message id for the protocol
//
/**
* @def ITL612_IR_SENSOR_STX1
* @brief first byte in message
* @details defines the 1st STX byte in the message
*/
#define ITL612_IR_SENSOR_STX1 0x55U                                                                                         // First byte of the Frame Header
/**
* @def ITL612_IR_SENSOR_STX2
* @brief second byte in message
* @details defines the 2nd STX byte in the message
*/
#define ITL612_IR_SENSOR_STX2 0xAAU                                                                                         // 2nd byte of the Frame Header

// ••••••••••••••••••••••••••••••••••••••••• Length •••••••••••••••••••••••••••••••••
/**
* @def ITL612_CC_LEN
* @brief the length specified in the conrtol command query message to the IR Sensor
* @details the length specified in the conrtol command query message to the IR Sensor
*/
#define ITL612_CC_LEN 0x07U                                                                                                 // Length of the control command
/**
* @def ITL612_CC_LEN_POS
* @brief the position in the message array for length
* @details the position in the message array for length
*/
#define ITL612_CC_LEN_POS 2u                                                                                                // postion in message

/**
* @def ITL612_MAKE_CMD_WORD
* @brief make a word 32bit from 4 bytes
* @details pass 4 8bit bytes and return a 32bit word.
*/
#define ITL612_MAKE_CMD_WORD(byte1,byte2,byte3,byte4) ( (byte1<<24) | (byte2<<16) | (byte3<<8) | (byte4)  )                 // sets the MSB of the option byte for writing


// ••••••••••••••••••••••••••••••••••••••••• Functional Class ••••••••••••••••••••••••••••••••••••••••• 
//
/**
* @def ITL612_CC_FuncClass_POS
* @brief position in message of functional class byte
* @details position in message of functional class byte
*/
#define ITL612_CC_FuncClass_POS 3u                                                                                                // postion in message

/**
* @brief ITL612_FuncClass_CC_e
* @details Is the enumnerated type describing the function class field of the message
*/
typedef enum {
	status = 0x0u,
	setup = 0x01u,
	video = 0x02u,                                                                                    
	app = 0x03u,
	temperature = 0x04u,
	slash = 0xA0,                                                                                                             // use for shutter manual
	numOfContrComFuncClass
} ITL612_FuncClass_CC_e;                                                                                                 // Function Class for Control Command

// ••••••••••••••••••••••••••••••••••••••••• Page ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//
/**
* @def ITL612_CC_Page_POS
* @brief position in message of page descriptor byte
* @details position in message of page descriptor byte
*/
#define ITL612_CC_Page_POS 4u                                                                                                // postion in message
/**
* @brief ITL612_PageNum_CC_e
* @details Is the enumnerated type describing the page field of the message
*/
typedef enum {
	page1_set = 0x0u,
	page2 = 0x01u,
	page3 = 0x02u,
	page4 = 0x03u,
	page5 = 0x04u,
	page6_hst = 0x05u,
	numOfContrComPages
} ITL612_PageNum_CC_e;                                                                                                        // Page number for Control Command


// ••••••••••••••••••••••••••••••••••••••••• Option ••••••••••••••••••••••••••••••••••••••••••••••••••••••
// FunctionalClass == video options
//
// ••••••••••••••••••••••••••••••••••••••••• option byte macro ••••••••••••••••••••••••••••••••••••••••••
/**
* @def ITL612_HOST_WRITE(byte)
* @brief use for writing data
* @details sets the MSB of the option byte for writing
*/
#define ITL612_HOST_WRITE(byte) ( byte | 0b10000000 )                                                                       // sets the MSB of the option byte for writing
/**
* @def ITL612_HOST_READ(byte)
* @brief use for reading data
* @details clears the MSB of the option byte when reading
*/
#define ITL612_HOST_READ(byte) ( byte & 0b01111111 )                                                                        // unsets the MSB of the option byte for writing
/**
* @def ITL612_CC_Option_POS
* @brief posiiton of the option byte in the message
* @details posiiton of the option byte in the message
*/
#define ITL612_CC_Option_POS 5u                                                                                             // postion in message

// FunctionalClass == setup options
//
/**
* @brief ITL612_SetUp_Option_CC_e
* @details Is the enumnerated type describing the set-up options in query the message
*/
typedef enum {
	auto_comp_time = 0x1u,
	image_freezing = 0x02u,
	save = 0x04u,
	factory_restore = 0x5u,
	temp_calib = 0x07u,
	shutter_manu = 0x08u,                                                                                                      // may be it should be a slash check the message in manual ?
	numOfSetUpOptionsPages
} ITL612_SetUp_Option_CC_e;

// FunctionalClass == video page 2
//
/**
* @brief ITL612_Video_Option_p2_CC_e
* @details Is the enumnerated type describing the video page2 options in query the message
*/
typedef enum {
	ext_sync_switch = 0x1u,
    digital_port_type = 0x2u,
	cmos_content_sel = 0x03u,
	cmos8_content_sel = 0x04u,
	frame_rate = 0x05u,
	scene = 0x06u,
	shutter = 0x07u,
	numOfVideoOptionsP2Pages
} ITL612_Video_Option_p2_CC_e;                                                                                               // Video Options page 2 for Control Command

// FunctionalClass == video page 3
//
/**
* @brief ITL612_Video_Option_p3_CC_e
* @details Is the enumnerated type describing the video page3 (algorithm) options in query the message
*/
typedef enum {
    Anti_striation_switch = 0x05u,
	image_mode = 0x06u,
	brightness = 0x0au,
	contrast = 0x0bu,
	enhanced_detail = 0x12u,
	dimming_mode = 0x18u,
	hue = 0x19u,
	image_obs_mode = 0x20u,
	numOfVideoOptionsP3Pages
} ITL612_Video_Option_p3_CC_e;                                                                                               // Video Options page 3 for Control Command

// FunctionalClass == application (badpixel) options (page2)
//
/**
* @brief ITL612_App_Option_p2_CC_e
* @details Is the enumnerated type describing the application page 2 options in query the message
*/
typedef enum {
    cursor_coordinate_X = 0x02u,
    cursor_coordinate_Y = 0x03u,
	Defective_pixel_addition = 0x04u,
	Defective_pixel_saving = 0x05u,
	numOfAppOptionsP2Pages
} ITL612_App_Option_p2_CC_e;                                                                                                   // Application for Control Command page2

// FunctionalClass == application options (page4)
//
/**
* @brief ITL612_App_Option_p4_CC_e
* @details Is the enumnerated type describing the application page 4 options in query the message
*/
typedef enum {
    Anaysis_Mode = 0x01u,
    region_UL_X = 0x02u,
	region_UL_Y = 0x03u,
	numOfAppOptionsP4Pages
} ITL612_App_Option_p4_CC_e;                                                                                                   // Application for Control Command page 4

// FunctionalClass == application options (page6)
//
/**
* @brief ITL612_App_Option_p6_CC_e
* @details Is the enumnerated type describing the application page 6 options in query the message
*/
typedef enum {
    Isotherm_switch = 0x06u,
    Isotherm_model = 0x07u,
    Isotherm_thres_Up = 0x08u,
    Isotherm_thres_Lo = 0x09u,
	numOfAppOptionsP6Pages
} ITL612_App_Option_p6_CC_e;                                                                                                   // Application for Control Command page 6 hst

// FunctionalClass == (temperature)
//
/**
* @brief ITL612_Temp_Option_p1_CC_e
* @details Is the enumnerated type describing the temperature options in query the message
*/
typedef enum {
    Distance = 0x01u,
    Emissivity = 0x02u,
    Measurement = 0x03u,
	Save_settings  = 0x4u,
    Factory_reset = 0x06u,
    Reflected = 0x07u,
    Humidity = 0x08u,
	Temperature  = 0x09u,
	numOfTempOptionsP1Pages
} ITL612_Temp_Option_p1_CC_e;                                                                                                   // Application for Control Command page 6 hst

// FunctionalClass == slash options
//
/**
* @brief ITL612_Slash_Option_CC_e
* @details Is the enumnerated type describing the slash options in query the message
*/
typedef enum {
	shutter_man = 0x08u,                                                                                                        // seems to show page 3 as well
	numOfSlashOptionsPages
} ITL612_Slash_Option_CC_e;                                                                                                     // Slash Options for Control Command

/**
* @brief ITL612_ESS_VID_e
* @details Is the enumnerated type describing external sync switch options for the video page
*/
typedef enum {
	Slave_Off,  
	Slave_on,     
	Master,  
	numOfExtSyncSwitchOptions
} ITL612_ESS_VID_e;

/**
* @brief ITL612_DigitalPort_VID_e
* @details Is the enumnerated type describing digital ports for the video page
*/
typedef enum {
	DP_Off,  
	DP_BT656,     
	DP_CMOS,  
	numOfDigitalPortOptions
} ITL612_DigitalPort_VID_e;

/**
* @brief ITL612_CMOSContent_VID_e
* @details Is the enumnerated type describing cmos content for the video page
*/
typedef enum {
	YUV422,  
	YUV422_PL,     
	Y16,  
	Y16_PL, 
	Y16_YUV422,
	Y16_PL_YUV422,
	numOfCMOSContentSettings
} ITL612_CMOSContent_VID_e;

/**
* @brief ITL612_CMOS8Content_VID_e
* @details Is the enumnerated type describing cmos8 content for the video page
*/
typedef enum {
	CMOS8_MSB,  
	CMOS8_LSB,     
	numOfCMOS8ContentSettings
} ITL612_CMOS8Content_VID_e;

/**
* @brief ITL612_FrameRate_VID_e
* @details Is the enumnerated type describing frame rate for the video page
*/
typedef enum {
	FR50_60Hz,  
	FR25_30Hz,     
	FR9_Hz,
	numOfFrameRateSettings
} ITL612_FrameRate_VID_e;

/**
* @brief ITL612_Option1Cmd_AC_e
* @details Is the enumnerated type describing option 1 command of algorithm control page
*/
typedef enum {
	AntiStraitionOff,  
	AntiStraitionOn,     
	numOfOption1CmdAC
} ITL612_Option1Cmd_AC_e;

/**
* @brief ITL612_Option5Cmd_AC_e
* @details Is the enumnerated type describing option 5 command of algorithm control page
*/
typedef enum {
	EEEnhancementOff,  
	EEEnhancementOn,     
	numOfOption5CmdAC
} ITL612_Option5Cmd_AC_e;

/**
* @brief ITL612_Option6Cmd_AC_e
* @details Is the enumnerated type describing option 6 command of algorithm control page
*/
typedef enum {
	Op2d_NoiseReduct0,  
	Op2d_NoiseReduct1, 
	Op2d_NoiseReduct2,     
	numOfOption6CmdAC
} ITL612_Option6Cmd_AC_e;  

/**
* @brief ITL612_Option1Cmd_HTP1_e
* @details Is the enumnerated type describing option 1 command of hot tracking page
*/
typedef enum {
	CloseAnalysis,  
	FullScreenAnalysis,
    Region1,
    Region2,
    Region3,	
	numOfOption1CmdHTP
} ITL612_Option1Cmd_HTP1_e;

/**
* @brief ITL612_Byte12Cmd_IT_e
* @details Is the enumnerated type describing byte 12 command of isotherm page
*/
typedef enum {
	IsoThermOff,  
	IsoThermOn,     
	numOfByte12CmdIT
} ITL612_Byte12Cmd_IT_e;

/**
* @brief ITL612_Byte27Cmd_IT_e
* @details Is the enumnerated type describing byte 27 command of isotherm page
*/
typedef enum {
	WhiteHeat,  
	Fulgarite,     
	IronRed,  
	HotIron, 
	Medical,  
	Arctic, 
	Rainbow1,  
	Rainbow2, 
	TraceRed,  
	BlackHeat, 
	numOfByte27CmdIT
} ITL612_Byte27Cmd_IT_e;

/**
* @brief ITL612_Byte7Cmd_TF_e
* @details Is the enumnerated type describing byte 7 command of thermography function page
*/
typedef enum {
	MinMaxTemp,  
	CrossCursorSpotMax,     
	CrossCursorSpotMin,  
	numOfByte7CmdTF
} ITL612_Byte7Cmd_TF_e;

/**
* @brief ITL612_Byte8Cmd_TF_e
* @details Is the enumnerated type describing byte 8 command of thermography function page
*/
typedef enum {
	Celsius,  
	Farenheit,     
	Kelvin,  
	numOfByte8CmdTF
} ITL612_Byte8Cmd_TF_e;

/**
* @brief ITL612_Byte26Cmd_TF_e
* @details Is the enumnerated type describing byte 26 command of thermography function page
*/
typedef enum {
	range1,  
	range2,      
	numOfByte26CmdTF
} ITL612_Byte26Cmd_TF_e;

// ••••••••••••••••••••••••••••••••••••••••• Command Words •••••••••••••••••••••••••••••••••
//
/**
* @def ITL612_CC_CW_POS
* @brief postion of control word in the message
* @details postion of control word in the message
*/
#define ITL612_CC_CW_POS 6u                                                                                                     // postion in message
// ••••••••••••••••••••••••••••••••••••••••• Set up ••••••••••••••••••••••••••••••••••••••••
/**
* @def ITL612_CC_CW4_NOT_FREEZE
* @brief No Freezing
* @details set no freeze in the control byte
*/
#define ITL612_CC_CW4_NOT_FREEZE 0x0U                                                                                           // No Freezing
/**
* @def ITL612_CC_CW4_FREEZE
* @brief Freezing active
* @details set freeze in the control byte
*/
#define ITL612_CC_CW4_FREEZE 0x1U                                                                                               // Freezing active
/**
* @def ITL612_CC_CW4_SAVE_SET
* @brief save the settings
* @details save settings control byte
*/
#define ITL612_CC_CW4_SAVE_SET 0x1U 

/**
* @def ITL612_CC_CW4_FACT_DEF
* @brief Restore factory default
* @details restore the setting sto factory default
*/
#define ITL612_CC_CW4_FACT_DEF 0x1U 

/**
* @def ITL612_CC_CW4_TEMPCAL_OFF
* @brief No Temperature Calibrtion
* @details set temperature calibration to off in the control byte
*/
#define ITL612_CC_CW4_TEMPCAL_OFF 0x0U                                                                                          // No Temperature Calibrtion
/**
* @def ITL612_CC_CW4_TEMPCAL_OFF
* @brief Temperature Calibrtion
* @details set temperature calibration to active in the control byte
*/
#define ITL612_CC_CW4_TEMPCAL_ON 0x1U                                                                                           // Temperature Calibrtion
/**
* @def ITL612_CC_CW4_DO_IT
* @brief do the operation
* @details do the operation
*/
#define ITL612_CC_CW4_DO_IT 0x1U                                                                                                // Do Operation
/**
* @def ITL612_CC_CW4_SHUTTR_CLOSE
* @brief shutter close
* @details shutter close
*/
#define ITL612_CC_CW4_SHUTTR_CLOSE 0x0U                                                                                         // Shutter Close
/**
* @def ITL612_CC_CW4_SHUTTR_OPEN
* @brief shutter open
* @details shutter open
*/
#define ITL612_CC_CW4_SHUTTR_OPEN 0x1U                                                                                          // Shutter Open
/**
* @def ITL612_CC_CW4_ATC_MIN
* @brief Automatic Time Compensation min
* @details Automatic Time Compensation min
*/
#define ITL612_CC_CW4_ATC_MIN 0U                                                                                                // Automatic Time Compensation min 
/**
* @def ITL612_CC_CW4_ATC_MAX
* @brief Automatic Time Compensation max
* @details Automatic Time Compensation max
*/
#define ITL612_CC_CW4_ATC_MAX 100U                                                                                              // Automatic Time Compensation max
/**
* @def SET_ATC(clamp,val)
* @brief Automatic Time Compensation set clamp to value if in range
* @details Automatic Time Compensation set clamp to val constrained by specified range
*/
#define SET_ATC(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_ATC_MIN) && (val <= ITL612_CC_CW4_ATC_MAX)) { clamp = val; } } while(0)  // set clamp to value if in range
					   
// ••••••••••••••••••••••••••••••••••••••••• Video •••••••••••••••••••••••••••••••••••••••••
/**
* @def ITL612_CC_CW4_ESS_SMON
* @brief External Sync Switch Slave Mode On
* @details Automatic External Sync Switch Slave Mode On
*/
#define ITL612_CC_CW4_ESS_SMON 0x0U                                                                                             // External Sync Switch Slave Mode On
/**
* @def ITL612_CC_CW4_ESS_SMOFF
* @brief External Sync Switch Slave Mode Off
* @details External Sync Switch Slave Mode Off
*/
#define ITL612_CC_CW4_ESS_SMOFF 0x1U                                                                                            // External Sync Switch Slave Mode Off
/**
* @def ITL612_CC_CW4_ESS_MM
* @brief External Sync Switch Master Mode
* @details External Sync Switch Master Mode
*/
#define ITL612_CC_CW4_ESS_MM 0x2U                                                                                               // External Sync Switch Master Mode

/**
* @def ITL612_CC_CW4_DPT_SMON
* @brief Digital Port Type Off
* @details Digital Port Type Off
*/
#define ITL612_CC_CW4_DPT_SMON 0x0U                                                                                             // Digital Port Type Off
/**
* @def ITL612_CC_CW4_DPT_BT656
* @brief Digital Port Type BT656
* @details Digital Port Type BT656
*/
#define ITL612_CC_CW4_DPT_BT656 0x1U                                                                                            // Digital Port Type BT656
/**
* @def ITL612_CC_CW4_DPT_CMOS
* @brief Digital Port Type CMOS
* @details Digital Port Type CMOS
*/
#define ITL612_CC_CW4_DPT_CMOS 0x2U                                                                                             // Digital Port Type CMOS

// ••••••••••••••••• page 2 •••••••••••••••••
/**
* @def ITL612_CC_CW4_CMOS_YUV422
* @brief CMOS content selection YUV422
* @details CMOS content selection YUV422
*/
#define ITL612_CC_CW4_CMOS_YUV422 0x0U                                                                                          // CMOS content selection YUV422
/**
* @def ITL612_CC_CW4_CMOS_YUV422PL
* @brief CMOS content selection YUV422 paramter line
* @details CMOS content selection YUV422 paramter line
*/
#define ITL612_CC_CW4_CMOS_YUV422PL 0x1U                                                                                        // CMOS content selection YUV422 paramter line
/**
* @def ITL612_CC_CW4_CMOS_YUV16
* @brief CMOS content selection YUV16
* @details CMOS content selection YUV16
*/
#define ITL612_CC_CW4_CMOS_YUV16 0x2U                                                                                           // CMOS content selection YUV16
/**
* @def ITL612_CC_CW4_CMOS_YUV16PL
* @brief CMOS content selection YUV16 paramter line
* @details CMOS content selection YUV16 paramter line
*/
#define ITL612_CC_CW4_CMOS_YUV16PL 0x3U                                                                                         // CMOS content selection YUV16 paramter line
/**
* @def ITL612_CC_CW4_CMOS_Y16_YUV422
* @brief CMOS content selection Y16_YUV422
* @details CMOS content selection Y16_YUV422
*/
#define ITL612_CC_CW4_CMOS_Y16_YUV422 0x4U                                                                                      // CMOS content selection Y16_YUV422
/**
* @def ITL612_CC_CW4_CMOS_Y16PL_YUV422
* @brief CMOS content selection Y16_paramter line YUV422
* @details CMOS content selection Y16_paramter line YUV422
*/
#define ITL612_CC_CW4_CMOS_Y16PL_YUV422 0x5U                                                                                    // CMOS content selection Y16_paramter line YUV422
/**
* @def ITL612_CC_CW4_CMOS8_MSB
* @brief CMOS8 MSB 1st
* @details CMOS8 MSB 1st
*/
#define ITL612_CC_CW4_CMOS8_MSB 0x1U                                                                                            // CMOS8 MSB 1st
/**
* @def ITL612_CC_CW4_CMOS8_LSB
* @brief CMOS8 LSB 1st
* @details CMOS8 LSB 1st
*/
#define ITL612_CC_CW4_CMOS8_LSB 0x2U                                                                                            // CMOS8 LSB 1st

/**
* @def ITL612_CC_CW4_FRS_50
* @brief Frame Rate Setting 50/60Hz
* @details Frame Rate Setting 50/60Hz
*/
#define ITL612_CC_CW4_FRS_50 0x0U                                                                                               // Frame Rate Setting 50/60Hz
/**
* @def ITL612_CC_CW4_FRS_25
* @brief Frame Rate Setting 25Hz
* @details Frame Rate Setting 25Hz
*/
#define ITL612_CC_CW4_FRS_25 0x1U                                                                                               // Frame Rate Setting 25Hz
/**
* @def ITL612_CC_CW4_FRS_9
* @brief Frame Rate Setting 9Hz
* @details Frame Rate Setting 9Hz
*/
#define ITL612_CC_CW4_FRS_9 0x2U                                                                                                // Frame Rate Setting 9Hz
/**
* @def ITL612_CC_CW4_SceneC_ON
* @brief Scene Compensation on
* @details Scene Compensation on
*/
#define ITL612_CC_CW4_SceneC_ON 0x1U                                                                                             // Scene Compensation on
/**
* @def ITL612_CC_CW4_ShutC_ON
* @brief Shutter Compensation on
* @details Shutter Compensation on
*/
#define ITL612_CC_CW4_ShutC_ON 0x1U                                                                                              // Shutter Compensation on

// •••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••• video algorithms page 3 ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
/**
* @def ITL612_CC_CW4_ASS_OFF
* @brief ASS off
* @details ASS (anti straition switch) off
*/
#define ITL612_CC_CW4_ASS_OFF 0x0U                                                                                               // ASS off
/**
* @def ITL612_CC_CW4_ASS_ON
* @brief ASS on
* @details ASS on
*/
#define ITL612_CC_CW4_ASS_ON 0x1U                                                                                                // ASS on
/**
* @def ITL612_CC_CW4_IMS_SOFT
* @brief Image Sharpening Soft mode
* @details Image Sharpening Soft mode
*/
#define ITL612_CC_CW4_IMS_SOFT 0x0U                                                                                              // Image Sharpening Soft mode 
/**
* @def ITL612_CC_CW4_IMS_STD
* @brief Image Sharpening Standard mode
* @details Image Sharpening Standard mode
*/
#define ITL612_CC_CW4_IMS_STD 0x1U                                                                                               // Image Sharpening Standard mode 
/**
* @def ITL612_CC_CW4_IMS_EM
* @brief Image Sharpening Enhancement mode
* @details Image Sharpening Enhancement mode
*/
#define ITL612_CC_CW4_IMS_EM 0x2U                                                                                                // Image Sharpening Enhancement mode

/**
* @def ITL612_CC_CW4_BRIGHT_MIN
* @brief Brightness min
* @details Brightness min
*/
#define ITL612_CC_CW4_BRIGHT_MIN 0U                                                                                              // Brightness min 
/**
* @def ITL612_CC_CW4_BRIGHT_MAX
* @brief Brightness max
* @details Brightness max
*/
#define ITL612_CC_CW4_BRIGHT_MAX 16U                                                                                             // Brightness max
/**
* @def SET_BRIGHT(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_BRIGHT(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_BRIGHT_MIN) && (val <= ITL612_CC_CW4_BRIGHT_MAX)) { clamp = val; } } while(0)  // set clamp to value if in range

/**
* @def ITL612_CC_CW4_CONTRAST_MIN
* @brief Contrast min
* @details Contrast min
*/
#define ITL612_CC_CW4_CONTRAST_MIN 0U                                                                                            // Contrast min 
/**
* @def ITL612_CC_CW4_CONTRAST_MAX
* @brief Contrast max
* @details Contrast max
*/
#define ITL612_CC_CW4_CONTRAST_MAX 255U                                                                                          // Contrast max
/**
* @def SET_CONTRAST(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_CONTRAST(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_CONTRAST_MIN) && (val <= ITL612_CC_CW4_CONTRAST_MAX)) { clamp = val; } } while(0)  // set clamp to value if in range

/**
* @def ITL612_CC_CW4_EDG_MIN
* @brief Enhanced Detail Gain min
* @details Enhanced Detail Gain min
*/
#define ITL612_CC_CW4_EDG_MIN 0U                                                                                                 // Enhanced Detail Gain min 
/**
* @def ITL612_CC_CW4_EDG_MAX
* @brief Enhanced Detail Gain max
* @details Enhanced Detail Gain max
*/
#define ITL612_CC_CW4_EDG_MAX 255U                                                                                               // Enhanced Detail Gain min
/**
* @def SET_EDG(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_EDG(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_EDG_MIN) && (val <= ITL612_CC_CW4_EDG_MAX)) { clamp = val; } } while(0)    // set clamp to value if in range

/**
* @def ITL612_CC_CW4_DIM_1
* @brief Dimming mode 1
* @details Dimming mode 1
*/					   
#define ITL612_CC_CW4_DIM_1 0x0U                                                                                                 // Dimming mode 
/**
* @def ITL612_CC_CW4_DIM_2
* @brief Dimming mode 2
* @details Dimming mode 2
*/
#define ITL612_CC_CW4_DIM_2 0x1U                                                                                                 // Dimming mode 
/**
* @def ITL612_CC_CW4_DIM_3
* @brief Dimming mode 3
* @details Dimming mode 3
*/
#define ITL612_CC_CW4_DIM_3 0x2U                                                                                                 // Dimming mode 
/**
* @def ITL612_CC_CW4_HUE_COOL
* @brief hue cool
* @details hue cool
*/
#define ITL612_CC_CW4_HUE_COOL 0x1U                                                                                              // hue cool 
/**
* @def ITL612_CC_CW4_HUE_WARM
* @brief hue warm
* @details hue warm
*/
#define ITL612_CC_CW4_HUE_WARM 0x0U                                                                                              // hue warm 
/**
* @def ITL612_CC_CW4_OBS_IM
* @brief image
* @details image
*/
#define ITL612_CC_CW4_OBS_IM 0x0U                                                                                                // image 
/**
* @def ITL612_CC_CW4_OBS_TEMP
* @brief temperature
* @details temperature
*/
#define ITL612_CC_CW4_OBS_TEMP 0x1U                                                                                              // temperature 

// ••••••••••••••••••••••••••••••••••••••••• Application •••••••••••••••••••••••••••••••••••••••••
// page 2
/**
* @def ITL612_CC_CW4_DPA
* @brief Defective pixsel additon
* @details Defective pixsel additon
*/
#define ITL612_CC_CW4_DPA 0x1U                                                                                                   // Defective pixsel additon
/**
* @def ITL612_CC_CW4_DRA
* @brief Defective row addition
* @details Defective row addition
*/
#define ITL612_CC_CW4_DRA 0x2U                                                                                                   // Defective row addition
/**
* @def ITL612_CC_CW4_DCA
* @brief Defective column addition
* @details Defective column addition
*/
#define ITL612_CC_CW4_DCA 0x3U                                                                                                   // Defective column addition
/**
* @def ITL612_CC_CW4_DPS
* @brief Defective picsel saving
* @details Defective picsel saving
*/
#define ITL612_CC_CW4_DPS 0x1U                                                                                                   // Defective picsel saving
		
// page 4
// sets value to command words 1-4 for Hots tracking page 1 (region analysis)
/**
* @def ITL612_CC_CW4_Ana_Off
* @brief analysis off
* @details analysis off
*/
#define ITL612_CC_CW4_Ana_Off 0x0U                                                                                                // analysis off
/**
* @def ITL612_CC_CW4_Ana_FS
* @brief full screen anaysis
* @details full screen anaysis
*/
#define ITL612_CC_CW4_Ana_FS 0x1U                                                                                                 // full screen anaysis
/**
* @def ITL612_CC_CW4_Ana_R1
* @brief region 1
* @details region 1
*/
#define ITL612_CC_CW4_Ana_R1 0x2U                                                                                                 // region 1
/**
* @def ITL612_CC_CW4_Ana_R2
* @brief region 2
* @details region 2
*/
#define ITL612_CC_CW4_Ana_R2 0x3U                                                                                                 // region 2	
/**
* @def ITL612_CC_CW4_Ana_R3
* @brief region 3
* @details region 3
*/
#define ITL612_CC_CW4_Ana_R3 0x4U                                                                                                 // region 3
/**
* @def ITL612_CC_CW4_RAX_MIN
* @brief Region analysis X min
* @details Region analysis X min3
*/
#define ITL612_CC_CW4_RAX_MIN 0U                                                                                                  // Region analysis X min 
/**
* @def ITL612_CC_CW4_RAX_MAX
* @brief Region analysis X max
* @details Region analysis X max
*/
#define ITL612_CC_CW4_RAX_MAX 639U                                                                                                // Region analysis X max
/**
* @def SET_RAX(clamp,val)
* @brief Rset clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_RAX(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_RAX_MIN) && (val <= ITL612_CC_CW4_RAX_MAX)) { clamp = val; } } while(0)     // set clamp to value if in range
/**
* @def ITL612_CC_CW4_RAY_MIN
* @brief Region analysis Y min
* @details Region analysis Y min
*/
#define ITL612_CC_CW4_RAY_MIN 0U                                                                                                  // Region analysis Y min 
/**
* @def ITL612_CC_CW4_RAY_MAX
* @brief Region analysis Y max
* @details Region analysis Y max
*/
#define ITL612_CC_CW4_RAY_MAX 511U                                                                                                // Region analysis Y max
/**
* @def SET_RAY(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_RAY(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_RAY_MIN) && (val <= ITL612_CC_CW4_RAY_MAX)) { clamp = val; } } while(0)     // set clamp to value if in range
					   
// page 6 hot spot tracking
/**
* @def ITL612_CC_IS_OFF
* @brief isotherm switch on
* @details isotherm switch on
*/
#define ITL612_CC_IS_OFF 0x0U                                                                                                     // isotherm switch on
/**
* @def ITL612_CC_IS_ON
* @brief isotherme switch off
* @details isotherme switch off
*/
#define ITL612_CC_IS_ON 0x1U                                                                                                      // isotherme switch off
/**
* @def ITL612_CC_IM_UD
* @brief isotherm model up/down
* @details isotherm model up/down
*/
#define ITL612_CC_IM_UD 0x0U                                                                                                      // isotherm model up/down
/**
* @def ITL612_CC_IM_MED
* @brief isotherme model medium
* @details isotherme model medium
*/
#define ITL612_CC_IM_MED 0x1U                                                                                                     // isotherme model medium	
/**
* @def ITL612_CC_IT_MIN
* @brief isothermal threshold min
* @details isothermal threshold min
*/
#define ITL612_CC_IT_MIN 0U      
/**
* @def ITL612_CC_IT_MAX
* @brief isothermal threshold max
* @details isothermal threshold max
*/                                                                                                                                  
#define ITL612_CC_IT_MAX 0xFFU                                                                                                    // isothermal threshold max
/**
* @def SET_IT(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_IT(clamp,val) \
                       do { if ((val >= ITL612_CC_IT_MIN) && (val <= ITL612_CC_IT_MAX)) { clamp = val; } } while(0)               // set clamp to value if in range


// ••••••••••••••••••••••••••••••••••••••••• Temperature •••••••••••••••••••••••••••••••••••••••••
// page 1 setup
/**
* @def ITL612_CC_CW4_TDIST_MIN
* @brief Temperature Distance min
* @details Temperature Distance min
*/
#define ITL612_CC_CW4_TDIST_MIN 0U                                                                                                // Temperature Distance min 
/**
* @def ITL612_CC_CW4_TDIST_MAX
* @brief Temperature Distance max
* @details Temperature Distance max
*/
#define ITL612_CC_CW4_TDIST_MAX 100U                                                                                              // Temperature Distance max
/**
* @def SET_TDIST(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_TDIST(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_TDIST_MIN) && (val <= ITL612_CC_CW4_TDIST_MAX)) { clamp = val; } } while(0)  // set clamp to value if in range
/**
* @def ITL612_CC_CW4_TEMIS_MIN
* @brief Temperature Emissivity min
* @details Temperature Emissivity min
*/
#define ITL612_CC_CW4_TEMIS_MIN 0U                                                                                                // Temperature Emissivity min 
/**
* @def ITL612_CC_CW4_TEMIS_MAX
* @brief Temperature Emissivity max
* @details Temperature Emissivity max
*/
#define ITL612_CC_CW4_TEMIS_MAX 100U                                                                                              // Temperature Emissivity max
/**
* @def SET_TEMIS(clamp,val)
* @brief set clamp to value if in range
* @details set clamp to value if in range
*/
#define SET_TEMIS(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_TEMIS_MIN) && (val <= ITL612_CC_CW4_TEMIS_MAX)) { clamp = val; } } while(0)  // set clamp to value if in range
/**
* @def ITL612_CC_CW4_MM_1
* @brief Min + max temp.
* @details Min + max temp.e
*/					   
#define ITL612_CC_CW4_MM_1 0x0U                                                                                                   // Min + max temp. 
/**
* @def ITL612_CC_CW4_MM_2
* @brief cursor spot+ max temp
* @details cursor spot+ max temp
*/
#define ITL612_CC_CW4_MM_2 0x1U                                                                                                   // cursor spot+ max temp
/**
* @def ITL612_CC_CW4_FR_1
* @brief Factory Reset min + cursor spot temp.
* @details Factory Reset min + cursor spot temp.
*/
#define ITL612_CC_CW4_FR_1 0x2U                                                                                                   // Factory Reset min + cursor spot temp. 
/**
* @def ITL612_CC_CW4_FR_2
* @brief Factory Reset Setting
* @details Factory Reset Setting
*/
#define ITL612_CC_CW4_FR_2 0x1U                                                                                                   // Factory Reset Setting
/**
* @def ITL612_CC_CW4_TRefSet_MIN
* @brief Reflective Setting min
* @details Reflective Setting min
*/
#define ITL612_CC_CW4_TRefSet_MIN 0U                                                                                              // Reflective Setting min 
/**
* @def ITL612_CC_CW4_TRefSet_MAX
* @brief Reflective Setting max
* @details Reflective Setting max
*/
#define ITL612_CC_CW4_TRefSet_MAX 0xFFU                                                                                           // Reflective Setting max
/**
* @def ITL612_CC_CW4_TRefSet_MAX
* @brief Reflective Setting
* @details Reflective Setting
*/
#define SET_TRefSet(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_TRefSet_MIN) && (val <= ITL612_CC_CW4_TRefSet_MAX)) { clamp = val; } } while(0)  // Reflective Setting
/**
* @def ITL612_CC_CW4_TEMP_SS
* @brief Save Setting
* @details Save Setting
*/
#define ITL612_CC_CW4_TEMP_SS 0x1U                                                                                                // Save Setting
/**
* @def ITL612_CC_CW4_THSS_MIN
* @brief Humidity Save Setting min
* @details Humidity Save Setting min
*/
#define ITL612_CC_CW4_THSS_MIN 0U                                                                                                 // Humidity Save Setting min 
/**
* @def ITL612_CC_CW4_THSS_MAX
* @brief Humidity Save Setting max
* @details Humidity Save Setting max
*/
#define ITL612_CC_CW4_THSS_MAX 0xFU                                                                                               // Humidity Save Setting max
/**
* @def SET_THSSSet(clamp,val)
* @brief Humidity Save Setting
* @details Humidity Save Setting
*/
#define SET_THSSSet(clamp,val) \
                       do { if ((val >= ITL612_CC_CW4_THSS_MIN) && (val <= ITL612_CC_CW4_THSS_MAX)) { clamp = val; } } while(0)   // Humidity Save Setting
/**
* @def ITL612_CC_CW4_TM1_MIN
* @brief Temp Meas min
* @details Temp Meas min
*/					   
#define ITL612_CC_CW4_TM1_MIN -20.0f                                                                                              // Temp Meas min 
/**
* @def ITL612_CC_CW4_TM1_MAX
* @brief Temp Meas max
* @details Temp Meas max
*/
#define ITL612_CC_CW4_TM1_MAX 150.0f                                                                                              // Temp Meas max
/**
* @def SET_TM1Set(clamp,val)
* @brief Temp Measurement send float (not required)
* @details Temp Measurement send float (not required)
*/
#define SET_TM1Set(clamp,val) \
                       do { if (((float)(val) >= ITL612_CC_CW4_TM1_MIN) && ((float)(val) <= ITL612_CC_CW4_TM1_MAX)) { \
					   conv_u uv; uv.f = (float)(val); clamp = uv.u32; } } while(0)                                               // Temp Measurement send float (not required)
/**
* @def ITL612_CC_CW4_TM_minus20_150
* @brief set to the above range
* @details set to the above range
*/
#define ITL612_CC_CW4_TM_minus20_150 0u                                                                                           // set to the above range                      
/**
* @def ITL612_CC_CW4_TM2_MIN
* @brief Temp Meas min
* @details Temp Meas min
*/
#define ITL612_CC_CW4_TM2_MIN -20.0f                                                                                              // Temp Meas min 
/**
* @def ITL612_CC_CW4_TM2_MAX
* @brief Temp Meas max
* @details Temp Meas max
*/
#define ITL612_CC_CW4_TM2_MAX 550.0f                                                                                              // Temp Meas max
/**
* @def SET_TM2Set(clamp,val)
* @brief Temp Measurement send float (not required)
* @details Temp Measurement send float (not required)
*/
#define SET_TM2Set(clamp,val) \
                       do { if (((float)(val) >= ITL612_CC_CW4_TM2_MIN) && ((float)(val) <= ITL612_CC_CW4_TM2_MAX)) { \
					   conv_u uv; uv.f = (float)(val); clamp = uv.u32; } } while(0)                                               // Temp Measurement send float (not required)
/**
* @def ITL612_CC_CW4_TM_minus20_550
* @brief set to the above range
* @details set to the above range
*/
#define ITL612_CC_CW4_TM_minus20_550 1u                                                                                           // set to the above range 
					   
// ••••••••••••••••••••••••••••••••••••••••• General •••••••••••••••••••••••••••••••••••••••••
/**
* @def SET_CW1(cw1,val)
* @brief move the result to CW1
* @details take unsigned integer 32 val and return single MSB 8bit byte into cw1 
*/	 
#define SET_CW1(cw1,val) \
                       do { cw1 = (uint8_t)(((val & 0xFF000000u)>>24u)&0xFu); } while(0)                                         // move the result to CW1
/**
* @def SET_CW2(cw2,val)
* @brief move the result to CW2
* @details take unsigned integer 32 val and return single 2nd 8bit byte into cw2
*/
#define SET_CW2(cw2,val) \
                       do { cw2 = (uint8_t)(((val & 0x00FF0000u)>>16u)&0xFu); } while(0)                                         // move the result to CW2
/**
* @def SET_CW3(cw3,val)
* @brief move the result to CW3
* @details take unsigned integer 32 val and return single 3rd 8bit byte into cw3
*/
#define SET_CW3(cw3,val) \
                       do { cw3 = (uint8_t)(((val & 0xFF00u)>>8u)&0xFu); } while(0)                                            // move the result to CW3
/**
* @def SET_CW4(cw4,val)
* @brief move the result to CW4
* @details take unsigned integer 32 val and return single LSB 8bit byte into cw4
*/
#define SET_CW4(cw4,val) \
                       do { cw4 = (uint8_t)(val & 0xFFu); } while(0)                                                          // move the result to CW4			
					   

// ••••••••••••••••••••••••••••••••••••••••• ETX ••••••••••••••••••••••••••••••••••••••••• 					   
#define ITL612_CC_ETX_POS 12u                                                                                                     // postion in message
/**
* @def ITL612_IR_SENSOR_ETX
* @brief ETX byte
* @details last byte of query frame to denote end of transmission ETX
*/					   
#define ITL612_IR_SENSOR_ETX 0xF0U                                                                                                // End Transmission char for Frame Header

// ••••••••••••••••••••••••••••••••••••••••• Receive Confirmation •••••••••••••••••••••••• 
//                                           Option Byte
//
/**
* @def ITL612_RCV_OPTION_POS
* @brief position of option byte in confirmation frame received.
* @details position of option byte in confirmation frame received.
*/
#define ITL612_RCV_OPTION_POS 3u                                                                                                     // postion in message

/** Converts an unaligned four-byte little-endian integer into an int32 */
#define DW_TO_INT(p) ((p)[0] | ((p)[1] << 8) | ((p)[2] << 16) | ((p)[3] << 24))
/** Converts an unaligned two-byte little-endian integer into an int16 */
#define SW_TO_SHORT(p) ((p)[0] | ((p)[1] << 8))
/** Converts an int16 into an unaligned two-byte little-endian integer */
#define SHORT_TO_SW(s, p) \
    (p)[0] = (s); \
    (p)[1] = (s) >> 8;
/** Converts an int32 into an unaligned four-byte little-endian integer */
#define INT_TO_DW(i, p) \
    (p)[0] = (i); \
    (p)[1] = (i) >> 8; \
    (p)[2] = (i) >> 16; \
    (p)[3] = (i) >> 24;
	
/**
* @brief ITL612_Rcv_Option_CC_e
* @details Is the enumnerated type describing the receive options from a command confirmation reply frame
*/
typedef enum {
	recv_conf = 0x00u,
	recv_error = 0x01u,
    Save_settin = 0x02u,
    Restore_factory_settings = 0x03u,
    Restart = 0x04u, 
    Scene_compensation = 0x05u,
    Shutter_compensation = 0x06u,
    BL_compensation = 0x13u,
    BH_compensation = 0x14u,
    Calculate_K = 0x15u,
    Save_K = 0x16u,
    Load_K  = 0x17u,
    Load_initial_K = 0x18u,
    Temperature_restored_defualt = 0x29u,
    Upload_B0 = 0x1Au,
    Upload_B1 = 0x1Bu,
    Upload_B2 = 0x1Cu,
    Upload_B3 = 0x1Du,
    Upload_B4 = 0x1Eu,
    Upload_B5 = 0x1Fu,
    Upload_B6 = 0x20u,
    Upload_B7 = 0x21u,
    Upload_B8 = 0x22u,
    Upload_B9 = 0x23u,
    Upload_K = 0x24u,
    Upload_BL = 0x25u,
    Upload_BH = 0x26u,
    Upload_NUC = 0x27u,
    Upload_PROGRAM = 0x50u,
    Upload_FILTER = 0x51u,
    Upload_RMS = 0x52u,
    Upload_IDE = 0x53u,
    Upload_IMAGE_RGB = 0x54u,
    Upload_SINGLE_TMP = 0x55u,
    Upload_START_IMAGE_RGB = 0x56u,
    Upload_START_IMAGE = 0x57u,
    Upload_MENU_RGB = 0x58u,
    Upload_MENU = 0x59u,
    Upload_LOG = 0x5Au,
    Upload_HF_CURSOR = 0x5Bu,
    Upload_ZSP_PROGRAM = 0x5Cu,
    Program_upgrading = 0x34u,
    Defective_pxl_save = 0x39u,
    Defective_pxl_addition = 0x40u,
    Low_temperature_blackbody_collection = 0x47u,
    High_temperature_blackbody_collection = 0x41u,
    Two_point_calibration_successful = 0x42u,
    Two_point_calibration_failed = 0x43u,
    Single_point_collection_completed = 0x44u,
    Single_point_calibration_successful = 0x45u,
    Single_point_calibration_failed = 0x46u,
    start_to_upload = 0xA0u,
    upgrading_failed = 0xA1u,
    asic_starts_to_flash = 0xA2u,
	NoOfRcvOptions
} ITL612_Rcv_Option_CC_e; 

// =============================== define a union containing all possible receive packets =================
typedef union {
  ITL612_RcvFrameConf_t conf_frame;
  ITL612_RcvStatusReturn_t status_frame;
  ITL612_RcvSetUpReturn_t setup_frame;
  ITL612_RcvVideoReturn_t video_frame;
  ITL612_RcvAppReturn_t app_frame;
  ITL612_RcvAdvAppReturn_t app_adv_frame;
  ITL612_RcvMenuReturn_t hottrk1_frame;
  ITL612_RcvHotTrackP3Return_t hottrk2_frame;
  ITL612_RcvThermographyReturnPage1_t therm1_frame;
  ITL612_RcvThermographyReturnPage2_t therm2_frame;  
} ITL612_RcvPacket_u;

/* ------------------------------- function protypes --------------------------------------------------- */
void compose_setup_msg( ITL612_DataFrameQuery_t* msg, ITL612_SetUp_Option_CC_e opt, uint8_t cw4 );
void compose_slash_msg( ITL612_DataFrameQuery_t* msg, ITL612_Slash_Option_CC_e opt, uint8_t cw4 );
void compose_video_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p2_CC_e opt, uint8_t cw4 );
void compose_video_page3_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p3_CC_e opt, uint8_t cw4 );
void compose_app_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p2_CC_e opt, uint16_t cw4 );
void compose_app_page4_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p4_CC_e opt, uint16_t cw4 );
void compose_app_page6_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, uint16_t cw4 );
void compose_temperature_msg( ITL612_DataFrameQuery_t* msg, ITL612_Temp_Option_p1_CC_e opt, uint8_t cw4 );
void do_checksum( uint8_t *buf, size_t b_len, uint8_t *chksum );
inline static void copy_query_to_send_buffer( char* buf, ITL612_DataFrameQuery_t* msg, size_t len );
inline static void copy_rcv_frame_to_confirm( char* buf, size_t b_len, ITL612_RcvFrameConf_t* msg, size_t len );
inline static void copy_rcv_frame_to_query( char* buf, size_t b_len, ITL612_RcvQueryReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_status( char* buf, size_t b_len, ITL612_RcvStatusReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_su( char* buf, size_t b_len, ITL612_RcvSetUpReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_vid( char* buf, size_t b_len, ITL612_RcvVideoReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_app( char* buf, size_t b_len, ITL612_RcvAppReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_advapp( char* buf, size_t b_len, ITL612_RcvAdvAppReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_menu( char* buf, size_t b_len, ITL612_RcvMenuReturn_t* msg, size_t len );
inline static void copy_rcv_frame_to_hottrack( char* buf, size_t b_len, ITL612_RcvHotTrackP3Return_t* msg, size_t len );
inline static void copy_rcv_frame_to_thermogp1( char* buf, size_t b_len, ITL612_RcvThermographyReturnPage1_t* msg, size_t len );
inline static void copy_rcv_frame_to_thermogp2( char* buf, size_t b_len, ITL612_RcvThermographyReturnPage2_t* msg, size_t len );
inline static char *safe_strncpy( char *dest, size_t size_dest, const char *src, size_t n );
int16_t pack_rcv_frames( char* buf_in, size_t b_len, ITL612_RcvPacket_u *rcv_container );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                 
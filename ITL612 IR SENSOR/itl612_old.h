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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
* ITL612PACKED : Is a defined as the packing format needed for your CPU and compiler to pack the structure as per the structure definition.
*/
#ifdef __GNUC__                                                                 /* Macro to define packed structures the mikroe compiler is gcc based in definitions.h */
  #define ITL612PACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define ITL612PACKED __attribute__((packed))                                  /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define ITL612PACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define ITL612PACKED
#else                                                                           /* for MPLAB PIC32 */
  #define ITL612PACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

/**
* conv_u : Is a union for converting 32bit integer to/from float 
*/
typedef union {
	float f;
	std::uint32_t u32;
} conv_u;                                                                      // converts float to u32 integer and vice versa

// •••••••••••••••••••••••••••••••••••••••••• Data Frame Structure ••••••••••••••••••••••••••••••••••••••••••
//
// ----------------------------------------- send messages --------------------------------------------------
/**
* ITL612_DataFrameQuery_t : Is the structure to send a request to the ITL612 IR device
*/
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t Option;
        std::uint32_t CmdWord1 : 8u;
        std::uint32_t CmdWord2 : 8u;
        std::uint32_t CmdWord3 : 8u;
        std::uint32_t CmdWord4 : 8u;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_DataFrameQuery_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t Option;
        std::uint32_t CmdWord1 : 8u;
        std::uint32_t CmdWord2 : 8u;
        std::uint32_t CmdWord3 : 8u;
        std::uint32_t CmdWord4 : 8u;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_DataFrameQuery_t;
#endif

// ---------------------------------- receive messages --------------------------------------------------
/**
* @def ITL612_IR_RCV_CONF_LEN
* @brief length of confirm message reply
* @details length field of confirmation reply message
*/
#define ITL612_IR_RCV_CONF_LEN 6u
/**
* ITL612_RcvFrameConf_t : Is the structure which packages the confirmation sent from the ITL612 IR device
*/
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t RcvConf;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvFrameConf_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t RcvConf;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvFrameConf_t;
#endif

/**
* ITL612_RcvQueryReturn_t : Is the structure which packages the query return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_QUERY_LEN 23u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t Option[16u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvQueryReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t Option[16u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvQueryReturn_t;
#endif

/**
* ITL612_RcvStatusReturn_t : Is the structure which packages the status return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_STATUS_LEN 24u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t ModuleID;
        std::uint8_t ComObjID;		
        std::uint8_t Year;
        std::uint8_t Month;	
		std::uint8_t Day;
        std::uint16_t FocalSpotTempHi : 8;
        std::uint16_t FocalSpotTempLo : 8;
        std::uint8_t videoSystem;	
        std::uint8_t resolutionID;	
		std::uint32_t machineID;
		std::uint8_t reserve[4u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvStatusReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t ModuleID;
        std::uint8_t ComObjID;		
        std::uint8_t Year;
        std::uint8_t Month;	
		std::uint8_t Day;
        std::uint16_t FocalSpotTempHi : 8;
        std::uint16_t FocalSpotTempLo : 8;
        std::uint8_t videoSystem;	
        std::uint8_t resolutionID;	
		std::uint32_t machineID;
		std::uint8_t reserve[4u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvStatusReturn_t;
#endif

/**
* ITL612_RcvSetUpReturn_t : Is the structure which packages the setup instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_SETUP_LEN 14u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t autoCompTime;
		std::uint8_t imageFreeze;
		std::uint8_t imageType;
		std::uint8_t tempCalib;
        std::uint8_t ShutterMode;
		std::uint8_t ShutterPos;
		std::uint8_t lowNoiseMode;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvSetUpReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t autoCompTime;
		std::uint8_t imageFreeze;
		std::uint8_t imageType;
		std::uint8_t tempCalib;
        std::uint8_t ShutterMode;
		std::uint8_t ShutterPos;
		std::uint8_t lowNoiseMode;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvSetUpReturn_t;
#endif

/**
* ITL612_RcvVideoReturn_t : Is the structure which packages the video instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_VIDEO_LEN 13u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t cmdOption1;
        std::uint8_t cmdOption2;	
        std::uint8_t cmdOption3;
        std::uint8_t cmdOption4;		
        std::uint8_t cmdOption5;	
        std::uint8_t reserved[11u];		
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvVideoReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t cmdOption1;
        std::uint8_t cmdOption2;	
        std::uint8_t cmdOption3;
        std::uint8_t cmdOption4;		
        std::uint8_t cmdOption5;	
        std::uint8_t reserved[11u];		
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvVideoReturn_t;
#endif

/**
* ITL612_RcvAppReturn_t : Is the structure which packages the application instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_APP_LEN 16u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t anti_straition;
        std::uint8_t brightness;
        std::uint8_t contrast;
        std::uint8_t enhanced_detail;
		std::uint8_t enhance_algo;
		std::uint8_t enhance_algo;
        std::uint8_t noise_reduction;
		std::uint8_t DRC_mode;
		std::uint8_t reserve[10u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvAppReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t anti_straition;
        std::uint8_t brightness;
        std::uint8_t contrast;
        std::uint8_t enhanced_detail;
		std::uint8_t enhance_algo;
		std::uint8_t enhance_algo;
        std::uint8_t noise_reduction;
		std::uint8_t DRC_mode;
		std::uint8_t reserve[10u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvAppReturn_t;
#endif

/**
* ITL612_RcvAdvAppReturn_t : Is the structure which packages the advanced application instruction return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_ADVAPP_LEN 24u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t reserved;
		std::uint16_t cursorX;
        std::uint16_t cursorY;
        std::uint16_t ADvalueCursor;		
		std::uint8_t reserve[8u];
		std::uint16_t cursorPointY16;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvAdvAppReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t reserved;
		std::uint16_t cursorX;
        std::uint16_t cursorY;
        std::uint16_t ADvalueCursor;		
		std::uint8_t reserve[8u];
		std::uint16_t cursorPointY16;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvAdvAppReturn_t;
#endif

/**
* ITL612_RcvMenuReturn_t : Is the structure which packages the menu return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_MENU_LEN 37u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t option1;
		std::uint16_t option2;
		std::uint16_t option3;
		std::uint16_t option4;
		std::uint16_t option5;
		std::uint16_t coldSpotCoordX;
		std::uint16_t coldSpotCoordY;
		std::uint16_t coldSpotTempY16;
		std::uint16_t hotSpotCoordX;
		std::uint16_t hotSpotCoordY;
		std::uint16_t hotSpotTempY16;
		std::uint16_t cursorSpotCoordX;
		std::uint16_t cursorSpotCoordY;
		std::uint16_t cursorSpotTempY16;
		std::uint16_t regionalAvgTempY16;
        std::uint8_t reserve[2u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvMenuReturn_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t option1;
		std::uint16_t option2;
		std::uint16_t option3;
		std::uint16_t option4;
		std::uint16_t option5;
		std::uint16_t coldSpotCoordX;
		std::uint16_t coldSpotCoordY;
		std::uint16_t coldSpotTempY16;
		std::uint16_t hotSpotCoordX;
		std::uint16_t hotSpotCoordY;
		std::uint16_t hotSpotTempY16;
		std::uint16_t cursorSpotCoordX;
		std::uint16_t cursorSpotCoordY;
		std::uint16_t cursorSpotTempY16;
		std::uint16_t regionalAvgTempY16;
        std::uint8_t reserve[2u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvMenuReturn_t;
#endif

/**
* ITL612_RcvHotTrackP3Return_t : Is the structure which packages the hotspot tracking page 3 return sent from the ITL612 IR device
*/
#define ITL612_IR_RCV_HOTTRK_LEN 34u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t reserved1[11u];
		std::uint8_t isotherm;
		std::uint8_t displayMode;
		std::uint16_t ULIsoThermThres;
		std::uint16_t LLIsoThermThres;
		std::uint8_t reserved2[9u];
		std::uint8_t spectralRange;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvHotTrackP3Return_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t reserved1[11u];
		std::uint8_t isotherm;
		std::uint8_t displayMode;
		std::uint16_t ULIsoThermThres;
		std::uint16_t LLIsoThermThres;
		std::uint8_t reserved2[9u];
		std::uint8_t spectralRange;
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvHotTrackP3Return_t;
#endif

/**
* ITL612_RcvThermographyReturnPage1_t : Is the structure which packages the thermography return sent from the ITL612 IR device page1
*/
#define ITL612_IR_RCV_THERM_LEN 28u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t distance;
		std::uint8_t emissivity;
        std::uint8_t temperatLocation;
		std::uint8_t temperatUnits;
		std::uint8_t reserved[2u];
		std::uint16_t coordXpt1;
        std::uint16_t coordYpt1;
        std::uint16_t temperature_pt1;
		std::uint16_t coordXpt2;
        std::uint16_t coordYpt2;
        std::uint16_t temperature_pt2;
        std::uint16_t reflectedTemp;
        std::uint8_t humidity;
        std::uint8_t tempRange;	
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvThermographyReturnPage1_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint8_t distance;
		std::uint8_t emissivity;
        std::uint8_t temperatLocation;
		std::uint8_t temperatUnits;
		std::uint8_t reserved[2u];
		std::uint16_t coordXpt1;
        std::uint16_t coordYpt1;
        std::uint16_t temperature_pt1;
		std::uint16_t coordXpt2;
        std::uint16_t coordYpt2;
        std::uint16_t temperature_pt2;
        std::uint16_t reflectedTemp;
        std::uint8_t humidity;
        std::uint8_t tempRange;	
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
}) ITL612_RcvThermographyReturnPage1_t;
#endif

/**
* ITL612_RcvThermographyReturnPage2_t : Is the structure which packages the thermography return sent from the ITL612 IR device page2
*/
#define ITL612_IR_RCV_THERMP2_LEN 14u
#if defined(D_FT900)
typedef struct ITL612PACKED {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint16_t loBlackBodyTemp;
		std::uint16_t hiBlackBodyTemp;
		std::uint16_t singleBlackBodyTemp;
		std::uint8_t reserved[17u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
} ITL612_RcvThermographyReturnPage2_t;
#else
ITL612PACKED(
typedef struct {
        std::uint8_t FrameHeaderSTX1;
        std::uint8_t FrameHeaderSTX2;
        std::uint8_t Length;
        std::uint8_t FuncCategory;
        std::uint8_t Page;
        std::uint16_t loBlackBodyTemp;
		std::uint16_t hiBlackBodyTemp;
		std::uint16_t singleBlackBodyTemp;
		std::uint8_t reserved[17u];
        std::uint8_t Checksum;
        std::uint8_t FrameEnd;
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
	shutter_man = 0x08u,                                                                                                      // may be it should be a slash check the message in manual ?
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
* @details Is the enumnerated type describing the video page3 options in query the message
*/
typedef enum {
    Anti_striation_switch = 0x05u,
	image_mode = 0x06u,
	brightness = 0x0au,
	contrast = 0x0bu,
	enhanced_detail = 0z12u,
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
	numOfAppOptionsP4Pages
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
	numOfSetUpOptionsPages
} ITL612_Slash_Option_CC_e;                                                                                                     // Slash Options for Control Command

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

// ••••••••••••••••• page 3 •••••••••••••••••
/**
* @def ITL612_CC_CW4_ASS_OFF
* @brief ASS off
* @details ASS off
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
                       do { cw1 = (std::uint8_t)(((val & 0xF000u)>>24u)&0xFu); } while(0)                                         // move the result to CW1
/**
* @def SET_CW2(cw2,val)
* @brief move the result to CW2
* @details take unsigned integer 32 val and return single 2nd 8bit byte into cw2
*/
#define SET_CW2(cw2,val) \
                       do { cw2 = (std::uint8_t)(((val & 0x0F00u)>>16u)&0xFu); } while(0)                                         // move the result to CW2
/**
* @def SET_CW3(cw3,val)
* @brief move the result to CW3
* @details take unsigned integer 32 val and return single 3rd 8bit byte into cw3
*/
#define SET_CW3(cw3,val) \
                       do { cw3 = (std::uint8_t)(((val & 0xF0u)>>8u)&0xFu); } while(0)                                            // move the result to CW3
/**
* @def SET_CW4(cw4,val)
* @brief move the result to CW4
* @details take unsigned integer 32 val and return single LSB 8bit byte into cw4
*/
#define SET_CW4(cw4,val) \
                       do { cw4 = (std::uint8_t)(val & 0xFu); } while(0)                                                          // move the result to CW4			
					   

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


/**
* @brief ITL612_Rcv_Option_CC_e
* @details Is the enumnerated type describing the receive options from a command confirmation reply frame
*/
typedef enum {
	recv_conf = 0x00u,
	recv_error = 0x01u,
    Save_settings = 0x02u,
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
    Upload_BL = 0x25u,
    Upload_BH = 0x26u,
    Upload_NUC = 0x28u,
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
    Defective_pixel_saving = 0x39u,
    Defective_pixel_addition = 0x40u,
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

// Function : To compose a set-up message
//
/**
* @brief compose_setup_msg
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_SetUp_Option_CC_e opt, std::uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a setup query which can be copied to a char buffer and sent to the IR device
*/
void compose_setup_msg( ITL612_DataFrameQuery_t* msg, ITL612_SetUp_Option_CC_e opt, std::uint8_t cw4 )
{
	std::uint8_t chk_byte = 0x07u;                                       // calculate msg xor without STX and ETX starts at length
	std::uint8_t value_clamped = 0;
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Slash_Option_CC_e opt, std::uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a slash method query which can be copied to a char buffer and sent to the IR device
*/
void compose_slash_msg( ITL612_DataFrameQuery_t* msg, ITL612_Slash_Option_CC_e opt, std::uint8_t cw4 )
{
	std::uint8_t chk_byte = 0x7u;                                       // calculate msg xor without STX and ETX
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p2_CC_e opt, std::uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a video page2 query which can be copied to a char buffer and sent to the IR device
*/
void compose_video_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p2_CC_e opt, std::uint8_t cw4 )
{
	std::uint8_t chk_byte = 0x7u;                                       // calculate msg xor without STX and ETX
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p3_CC_e opt, std::uint8_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a video page3 query which can be copied to a char buffer and sent to the IR device
*/
void compose_video_page3_msg( ITL612_DataFrameQuery_t* msg, ITL612_Video_Option_p3_CC_e opt, std::uint8_t cw4 )
{
	std::uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
	std::uint8_t value_clamped = 0;
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p2_CC_e opt, std::uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page2 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page2_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p2_CC_e opt, std::uint16_t cw4 )
{
	std::uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
    ITL612_PageNum_CC_e page = page2;	
	ITL612_FuncClass_CC_e fc = app;
	std::uint8_t Cword = 0;
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p4_CC_e opt, std::uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page4 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page4_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p4_CC_e opt, std::uint16_t cw4 )
{
	std::uint8_t chk_byte = 0u;                                       // calculate msg xor without STX and ETX
	std::uint8_t Cword = 0u;
	std::uint16_t clampV = 0u;
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, std::uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a application page6 query which can be copied to a char buffer and sent to the IR device
*/
void compose_app_page6_msg( ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, std::uint16_t cw4 )
{
	std::uint8_t chk_byte = 0;                                       // calculate msg xor without STX and ETX
	std::uint8_t Cword = 0u;
	std::uint16_t clampV = 0u;
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
* @param[in] ITL612_DataFrameQuery_t* msg, ITL612_App_Option_p6_CC_e opt, std::uint16_t cw4
* @param[out] ITL612_DataFrameQuery_t* msg
* @return void
* @details Compose a temperature query which can be copied to a char buffer and sent to the IR device
*/
void compose_temperature_msg( ITL612_DataFrameQuery_t* msg, ITL612_Temp_Option_p1_CC_e opt, std::uint8_t cw4 )
{
	std::uint8_t chk_byte = 0;                                       //! checksum byte is calculated from msg bytes xor'ed without STX1/2 and ETX  
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
* @param[in] std::uint8_t *buf, std::uint8_t *chksum
* @param[out] std::uint8_t *chksum
* @return void
* @details perform checksum on incoming message buffer
*/
void do_checksum( std::uint8_t *buf, std::uint8_t *chksum ) {
	 *chksum = 0;
     for ( size_t i = ITL612_CC_LEN_POS; i < (sizeof(buf)-2) ; i++) {
		 *chksum ^= buf[i];
	 }
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif                 
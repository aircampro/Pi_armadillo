/*
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006-2018 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * Also includes code from Copyright © 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 * and Copyright (C) 2010 Doc Walker and Stephen Makonin.  All right reserved.
 * Copyright (C) 2010-2019 Oryx Embedded SARL. All rights reserved.
 *
 * Rebuilt Compiled and ported by ACP Aviation including support for the Leddar Vu Vu-8
 * and compience with entire modbus error codes.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MB_PROTO_H
#define _MB_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "crc.h"

#ifdef __GNUC__                                                                 /* Macro to define packed structures the compiler is gcc based in definitions.h */
  #define MODPACKED( __Declaration__ ) __Declaration__ __attribute__((packed)) ALIGNED(1)
#elif (defined(D_FT900) || defined(__TI_ARM__))                                 /* mikroe FT900 C or TI ref http://www.keil.com/support/man/docs/armclang_intro/armclang_intro_xxq1474359912082.htm */
  #define MODPACKED __attribute__((packed))                                   /* keil arm can also use #pragma pack(1) to pack to byte boundaries #pragma pack(4) to align to 4 byte boundaries */
#elif (defined(__CC_ARM) || defined(__IAR_SYSTEMS_ICC__))                       /* Keil MDK-ARM compiler ? or IAR C compiler */
  #define MODPACKED __packed
#elif (defined(_WIN32) || defined(__CWCC__))                                    /* windows or code warrior */
  #define MODPACKED
#else                                                                           /* for MPLAB PIC32 */
  #define MODPACKED( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#endif

//--------------- Endian Conversion --------------------------------------------
// 32-bit integer manipulation macros (little endian)
#ifndef GET_UINT32_LE                                                           // to use : GET_UINT32_LE( X[ 0], data,  0 );
#define GET_UINT32_LE(n,b,i)                              \
{                                                         \
    (n) = ( (uint32_t) (b)[(i)    ]       )               \
        | ( (uint32_t) (b)[(i) + 1U] <<  8U )             \
        | ( (uint32_t) (b)[(i) + 2U] << 16U )             \
        | ( (uint32_t) (b)[(i) + 3U] << 24U );            \
}
#endif

#ifndef GET_UINT16_LE                                                           // to use : GET_UINT16_LE( X[ 0], data,  0 );
#define GET_UINT16_LE(n,b,i)                              \
{                                                         \
    (n) = ( (uint32_t) (b)[(i)    ]       )               \
        | ( (uint32_t) (b)[(i) + 1U] <<  8U )             \
}
#endif

#ifndef PUT_UINT32_LE                                                           // to use : PUT_UINT32_LE( ctx->state[0], output,  0 );  PUT_UINT32_LE( ctx->state[1], output,  4 );
#define PUT_UINT32_LE(n,b,i)                                 \
{                                                            \
    (b)[(i)    ] = (uint8_t) ( ( (n)       ) & 0xFFU );      \
    (b)[(i) + 1U] = (uint8_t) ( ( (n) >>  8U ) & 0xFFU );    \
    (b)[(i) + 2U] = (uint8_t) ( ( (n) >> 16U ) & 0xFFU );    \
    (b)[(i) + 3U] = (uint8_t) ( ( (n) >> 24U ) & 0xFFU );    \
}
#endif

#ifndef PUT_UINT16_LE                                                           // to use : PUT_UINT16_LE( ctx->state[0], output,  0 );  PUT_UINT16_LE( ctx->state[1], output,  4 );
#define PUT_UINT16_LE(n,b,i)                                 \
{                                                            \
    (b)[(i)    ] = (uint8_t) ( ( (n)       ) & 0xFFU );      \
    (b)[(i) + 1U] = (uint8_t) ( ( (n) >>  8U ) & 0xFFU );    \
}
#endif

// 32-bit integer manipulation macros (big endian)
#ifndef GET_UINT32_BE                                                           // to use : GET_UINT32_BE( X[ 0], data,  0 );
#define GET_UINT32_BE(n,b,i)                     \
{                                                \
    (n) = ( (uint32_t) (b)[(i)    ] << 24U );    \
        | ( (uint32_t) (b)[(i) + 1U] << 16U );   \
        | ( (uint32_t) (b)[(i) + 2U] << 8U );    \
        | ( (uint32_t) (b)[(i) + 3U] );          \
}
#endif

#ifndef GET_UINT16_BE                                                           // to use : GET_UINT16_BE( X[ 0], data,  0 );
#define GET_UINT16_BE(n,b,i)                        \
{                                                   \
    (n) = ( (uint32_t) (b)[(i) ] << 8U )            \
        | ( (uint32_t) (b)[(i) + 1U]  )             \
}
#endif

#ifndef PUT_UINT32_BE                                                           // to use : PUT_UINT32_BE( ctx->state[0], output,  0 );  PUT_UINT32_BE( ctx->state[1], output,  4 );
#define PUT_UINT32_BE(n,b,i)                                \
{                                                           \
    (b)[(i)    ] = (uint8_t) ( ( (n) >> 24U ) & 0xFFU );    \
    (b)[(i) + 1U] = (uint8_t) ( ( (n) >> 16U ) & 0xFFU );   \
    (b)[(i) + 2U] = (uint8_t) ( ( (n) >> 8U ) & 0xFFU );    \
    (b)[(i) + 3U] = (uint8_t) ( ( (n) ) & 0xFFU );          \
}
#endif

#ifndef PUT_UINT16_BE
#define PUT_UINT16_BE(n,b,i)                                 \
{                                                            \
    (b)[(i)    ] = (uint8_t) ( ( (n) >>  8U ) & 0xFFU );     \
    (b)[(i) + 1U] = (uint8_t) ( ( (n) ) & 0xFFU );           \
}
#endif

#define SWAP_UINT16(x) (((x) >> 8U) | ((x) << 8U))
#define SWAP_INT16(x) (x << 8U) | ((x >> 8U) & 0xFFU)
#define SWAP_UINT32(x) (((x) >> 24U) | (((x) & 0x00FF0000UL) >> 8U) | (((x) & 0x0000FF00UL) << 8U) | ((x) << 24U))
#define SWAP_INT32(x) ((x>24)&0xffu) | ((x<<8)&0xff0000ul) | ((x>>8)&0xff00u) | ((x<<24)&0xff000000ul)

/* %%%%%%%%%%%%%%%%%%%%% Timing for RTU %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   */
/* ref :- Simon Platten                                                       */
 typedef enum {
 mbBAUDtim_1200 = 12,
 mbBAUDtim_2400 = 24,
 mbBAUDtim_4800 = 48,
 mbBAUDtim_9600 = 96,
 mbBAUDtim_19200 = 192,
 mbBAUDtim_38400 = 384,
 mbBAUDtim_57600 = 576,
 mbBAUDtim_115200 = 1152
 } mbBaudRateDelay_e;
 
// Interpacket delays selection of mbGapPreset
#define MODBUS_2CHAR 2lu  
#define GAP_SETPT_1200 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_1200))
#define GAP_SETPT_2400 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_2400))
#define GAP_SETPT_4800 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_4800))
#define GAP_SETPT_9600 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_9600))
#define GAP_SETPT_19200 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_19200))
#define GAP_SETPT_38400 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_38400))
#define GAP_SETPT_57600 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_57600))
#define GAP_SETPT_115200 (uint64_t)(MODBUS_2CHAR*110lu*Clock_kHz()/(4lu*mbBAUDtim_115200))

#define MbRxGapSetPt1_0char(mbGapPreset) (UINT16_MAX-(mbGapPreset >> 1lu))      // 1 char
#define MbRxGapSetPt2_0char(mbGapPreset) (UINT16_MAX-mbGapPreset)               // 2 chars
#define MbRxGapSetPt2_5char(mbGapPreset) (UINT16_MAX-(mbGapPreset*5lu >> 2lu))  // 2.5 chars
#define MbRxGapSetPt4_5char(mbGapPreset) (UINT16_MAX-(mbGapPreset*9lu >> 2lu))  // 4.5 chars

// Response packet timeouts for modbus master
#define PACKET_TIMEOUT_1200 2750u
#define PACKET_TIMEOUT_2400 1375u
#define PACKET_TIMEOUT_4800 688u
#define PACKET_TIMEOUT_9600 344u
#define PACKET_TIMEOUT_19200 172u
#define PACKET_TIMEOUT_38400 86u
#define PACKET_TIMEOUT_57600 57u
#define PACKET_TIMEOUT_115200 29u

/* ----------------------- Defines ------------------------------------------*/
#define MB_ADDRESS_BROADCAST 0U                                                 /*! Modbus broadcast address. */
#define MB_ADDRESS_MIN 1U                                                       /*! Smallest possible slave address. */
#define MB_ADDRESS_MAX 247U                                                     /*! Biggest possible slave address. */
//
// should reply back with same (loopback response) use to test comms w/o register info
//
#define MB_FUNC_NONE ( 0U )

/**
 Modbus function 0x01 Read Coils.
 This function code is used to read from 1 to 2000 contiguous status of
 coils in a remote device. The request specifies the starting address,
 i.e. the address of the first coil specified, and the number of coils.
 Coils are addressed starting at zero.

 The coils in the response buffer are packed as one coil per bit of the
 data field. Status is indicated as 1=ON and 0=OFF. The LSB of the first
 data word contains the output addressed in the query. The other coils
 follow toward the high order end of this word and from low order to high
 order in subsequent words.

 If the returned quantity is not a multiple of sixteen, the remaining
 bits in the final data word will be padded with zeros (toward the high
 order end of the word).

 @param ReadAddress address of first coil (0x0000..0xFFFF)
 @param BitQty quantity of coils to read (1..2000, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup discrete
 */
#define MB_FUNC_READ_COILS ( 1U )
/**
 * @brief Read Coils request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfCoils;                                                    //3-4
} ModbusReadCoilsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfCoils;                                                    //3-4
}) ModbusReadCoilsReq_t;                                                          // Request header
#endif

/**
 * @brief Read Coils response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t coilStatus[];                                                        //2  warning dynamic memory allocation you might want to define max number here
} ModbusReadCoilsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t coilStatus[];                                                        //2  warning dynamic memory allocation you might want to define max number here
}) ModbusReadCoilsResp_t;
#endif

/**
 Modbus function 0x02 Read Discrete Inputs.

 This function code is used to read from 1 to 2000 contiguous status of
 discrete inputs in a remote device. The request specifies the starting
 address, i.e. the address of the first input specified, and the number
 of inputs. Discrete inputs are addressed starting at zero.

 The discrete inputs in the response buffer are packed as one input per
 bit of the data field. Status is indicated as 1=ON; 0=OFF. The LSB of
 the first data word contains the input addressed in the query. The other
 inputs follow toward the high order end of this word, and from low order
 to high order in subsequent words.

 If the returned quantity is not a multiple of sixteen, the remaining
 bits in the final data word will be padded with zeros (toward the high
 order end of the word).

 @param ReadAddress address of first discrete input (0x0000..0xFFFF)
 @param BitQty quantity of discrete inputs to read (1..2000, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup discrete
 */
#define MB_FUNC_READ_DISCRETE_INPUTS ( 2U )
/**
 * @brief Read Discrete Inputs request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfInputs;                                                   //3-4
} ModbusReadDiscreteInputsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfInputs;                                                   //3-4
}) ModbusReadDiscreteInputsReq_t;
#endif

/**
 * @brief Read Discrete Inputs response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t inputStatus[];                                                       //2 warning dynamic memory allocation you might want to define max number here
} ModbusReadDiscreteInputsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t inputStatus[];                                                       //2 warning dynamic memory allocation you might want to define max number here
}) ModbusReadDiscreteInputsResp_t;
#endif

/**
 Modbus function 0x05 Write Single Coil.

 This function code is used to write a single output to either ON or OFF
 in a remote device. The requested ON/OFF state is specified by a
 constant in the state field. A non-zero value requests the output to be
 ON and a value of 0 requests it to be OFF. The request specifies the
 address of the coil to be forced. Coils are addressed starting at zero.

 @param WriteAddress address of the coil (0x0000..0xFFFF)
 @param State 0=OFF, non-zero=ON (0x0000..0xFF00)
 @return 0 on success; exception number on failure
 @ingroup discrete
 */
#define MB_FUNC_WRITE_SINGLE_COIL ( 5U )
/**
 * @brief Write Single Coil request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t outputAddr;                                                         //1-2
   uint16_t outputValue;                                                        //3-4
} ModbusWriteSingleCoilReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t outputAddr;                                                         //1-2
   uint16_t outputValue;                                                        //3-4
}) ModbusWriteSingleCoilReq_t;
#endif

/**
 * @brief Write Single Coil response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t outputAddr;                                                         //1-2
   uint16_t outputValue;                                                        //3-4
} ModbusWriteSingleCoilResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t outputAddr;                                                         //1-2
   uint16_t outputValue;                                                        //3-4
}) ModbusWriteSingleCoilResp_t;
#endif


/**
 Modbus function 0x0F Write Multiple Coils.

 This function code is used to force each coil in a sequence of coils to
 either ON or OFF in a remote device. The request specifies the coil
 references to be forced. Coils are addressed starting at zero.

 The requested ON/OFF states are specified by contents of the transmit
 buffer. A logical '1' in a bit position of the buffer requests the
 corresponding output to be ON. A logical '0' requests it to be OFF.

 @param WriteAddress address of the first coil (0x0000..0xFFFF)
 @param BitQty quantity of coils to write (1..2000, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup discrete
 */
#define MB_FUNC_WRITE_MULTIPLE_COILS  ( 15U )
/**
 * @brief Write Multiple Coils request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;       //0
   uint16_t startingAddr;      //1-2
   uint16_t quantityOfOutputs; //3-4
   uint8_t byteCount;          //5
   uint8_t outputValue[];      //6
} ModbusWriteMultipleCoilsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;       //0
   uint16_t startingAddr;      //1-2
   uint16_t quantityOfOutputs; //3-4
   uint8_t byteCount;          //5
   uint8_t outputValue[];      //6
}) ModbusWriteMultipleCoilsReq_t;
#endif

/**
 * @brief Write Multiple Coils response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;       //0
   uint16_t startingAddr;      //1-2
   uint16_t quantityOfOutputs; //3-4
} ModbusWriteMultipleCoilsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;       //0
   uint16_t startingAddr;      //1-2
   uint16_t quantityOfOutputs; //3-4
}) ModbusWriteMultipleCoilsResp_t;
#endif 

/**
 Modbus function 0x03 Read Holding Registers.

 This function code is used to read the contents of a contiguous block of
 holding registers in a remote device. The request specifies the starting
 register address and the number of registers. Registers are addressed
 starting at zero.

 The register data in the response buffer is packed as one word per
 register.

 @param ReadAddress address of the first holding register (0x0000..0xFFFF)
 @param ReadQty quantity of holding registers to read (1..125, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup register
 
 Read holding register (function code 0x3) on Leddar Tech Page 50
 As per the Modbus protocol, register values are returned in big-endian format
 
 These are read write parameters and also can be used with commands
 write register (function code 0x6), write multiple register (function code 0x10), and read/write multiple register (function code 0x17)
 */
#define MB_FUNC_READ_HOLDING_REGISTER ( 3U )

#if defined(LEDDARTECH_LIDDAR_USED)
#define LEDDAR_ADDR_EXP1 0U                                                     // Exponent for the number of accumulations (that is, if the content of this register is n, 2n accumulations are performed)
#define LEDDAR_ADDR_EXP2 1U                                                     // Exponent for the number of oversamplings (that is, if the content of this register is n, 2n oversamplings are performed)
#define LEDDAR_ADDR_BASE_SAM 2U                                                 // Number of base samples
#define LEDDAR_ADDR_DET_THRESH 4U                                               // Detection threshold as a fixed-point value with a 6-bit fractional part (i.e. threshold value is this register divided by 64)
#define LEDDAR_ADDR_LIGHT_POW 5U                                                // Light source power in percentage of the maximum. A value above 100 is an error. If a value is specified that is not one of the pre-defined values, the closest pre-defined value will be used. The register can be read back to know the actual value set. 6
#define LEDDAR_ADDR_BIT_FIELD 6U                                                // Bit field of acquisition options: Bit-0: Automatic light source power enabled Bit-1: Demerge object enabled Bit-2: Static noise removal enabled Bit-3: Precision enabled Bit-4: Saturation compensation enabled Bit-5: Overshoot management enabled
#define LEDDAR_ADDR_AUTO_LIGHT 7U                                               // Auto light source power change delay in number of measurements
#define LEDDAR_ADDR_NUM_ECHO 9U                                                 // Number of echoes for saturation acceptance: The number of echoes can be saturated to avoid decreasing the light source power in automatic mode
#define LEDDAR_ADDR_OP_MODE 10U                                                 // Operation mode  Write mode: 0: Stop (stop acquisition) 1: Continuous 2: Single (acquisition of a single detection frame) Read mode: 10: Stopped (sensor is stopped) 11: Continuous acquisition mode 12: Single frame busy (acquisition in progress) 13: Sensor is busy
#define LEDDAR_ADDR_SMOOTHING 11U                                               // Smoothing: Stabilizes the module measurements. The behavior of the smoothing algorithm can be adjusted by a value ranging from –16 through 16
#define LEDDAR_ADDR_LOW_SEGMENT 12U                                             // Low 16 bits of segment enabled: Bit-field of enabled segment
#define LEDDAR_ADDR_HIGH_SEGMENT 13U                                            // High 16 bits of segment enabled

#define LEDDAR_ADDR_MAX 14U                                                     // maximum number
#endif /* end leddar */

// ENRON modbus Hourly and Daily Archive Collection
#define ENRON_ADDR_HISTORY1 0x02BDU                                             // first history table address follow this word with the #index number needed
/**
 * @brief Read Holding Registers request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfRegs;                                                     //3-4
} ModbusReadHoldingRegsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfRegs;                                                     //3-4
}) ModbusReadHoldingRegsReq_t;
#endif

/**
 * @brief Read Holding Registers response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint16_t regValue[];                                                         //2 warning dynamically allocated memory you might want to define at maximum number e.g LEDDAR_ADDR_MAX
} ModbusReadHoldingRegsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint16_t regValue[];                                                         //2 warning dynamically allocated memory you might want to define at maximum number e.g LEDDAR_ADDR_MAX
}) ModbusReadHoldingRegsResp_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t dataBytes;
   float32_t datestamp;                                                         // Date Stamp 32 bit float MMDDYY (120304.0 = Dec 3, 2004)
   float32_t timestamp;                                                         // Time Stamp 32 bit float HHMMSS (080000.0 = 8am)
   float32_t item1;                                                             // 1st item logged 32-bit float
   float32_t item2;                                                             // 2nd item logged 32-bit float
   float32_t item3;                                                             // 3rd item logged 32-bit float
} ModbusReadHistoryResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint8_t dataBytes;
   float32_t datestamp;                                                         // Date Stamp 32 bit float MMDDYY (120304.0 = Dec 3, 2004)
   float32_t timestamp;                                                         // Time Stamp 32 bit float HHMMSS (080000.0 = 8am)
   float32_t item1;                                                             // 1st item logged 32-bit float
   float32_t item2;                                                             // 2nd item logged 32-bit float
   float32_t item3;                                                             // 3rd item logged 32-bit float
}) ModbusReadHistoryResp_t;
#endif

/**
 Modbus function 0x04 Read Input Registers.

 This function code is used to read from 1 to 125 contiguous input
 registers in a remote device. The request specifies the starting
 register address and the number of registers. Registers are addressed
 starting at zero.

 The register data in the response buffer is packed as one word per
 register.

 @param ReadAddress address of the first input register (0x0000..0xFFFF)
 @param ReadQty quantity of input registers to read (1..125, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup register
 */
 #if defined(LEDDARTECH_LIDDAR_USED)
 /*
 Read input register (function code 0x4) LeddarTech manual page 48
 As per the Modbus protocol, register values are returned in big-endian format.
 */
#define MB_FUNC_READ_INPUT_REGISTER ( 4U )
#define LEDDAR_ADDR_DET 1U                                                      // Detection status for polling mode:  0 = Detections not ready 1 = Detections ready: this status flag is reset to 0 after reading this register
#define LEDDAR_ADDR_NUM_SEG 2U                                                  // Number of segments (N)
#define LEDDAR_ADDR_NUM_DET 11U                                                 // Number of detections
#define LEDDAR_ADDR_LIGHT_POWER 12U                                             // Current percentage of light source power
#define LEDDAR_ADDR_ACQ_STAT 13U                                                // Bit field of acquisition status: Reserved
#define LEDDAR_ADDR_LOW_TIME 14U                                                // Low 16 bits of timestamp (number of milliseconds since the module was started)
#define LEDDAR_ADDR_HIGH_TIME 15U                                               // High 16 bits of timestamp
#define LEDDAR_ADDR_DIST1_START 16U                                             // Distance of first detection for each segment, zero if no detection in a segment. The distance unit is defined by the serial port parameters
#define LEDDAR_ADDR_DIST1_STOP(N) (16U + N-1U)                                  // Distance of last detection
#define LEDDAR_ADDR_AMP1_START(N) (16U + N)                                     // Amplitude
#define LEDDAR_ADDR_AMP1_STOP(N) (16U + (2U*N) - 1U)                            // last amplitude
#define LEDDAR_ADDR_FLAG1_START(N) (16U + (2U*N))                               // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG1_STOP(N) (16U + (3U*N) - 1U)                           // last flag
#define LEDDAR_ADDR_DIST2_START(N) (16U + (3U*N))                               // distance to 2nd detection
#define LEDDAR_ADDR_DIST2_STOP(N) (16U + (4U*N) - 1U)                           // distance to 2nd detection end
#define LEDDAR_ADDR_AMP2_START(N) (16U + (4U*N))                                // amplitude 2nd detection start
#define LEDDAR_ADDR_AMP2_STOP(N) (16U + (5U*N) - 1U)                            // amplitude 2nd detection stop
#define LEDDAR_ADDR_FLAG2_START(N) (16U + (5U*N))                               // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG2_STOP(N) (16U + (6U*N) - 1U)                           // last flag
#define LEDDAR_ADDR_DIST3_START(N) (16U + (6U*N))                               // distance to 3rd detection
#define LEDDAR_ADDR_DIST3_STOP(N) (16U + (7U*N) - 1U)                           // distance to 3rd detection end
#define LEDDAR_ADDR_AMP3_START(N) (16U + (7U*N))                                // amplitude 3rd detection start
#define LEDDAR_ADDR_AMP3_STOP(N) (16U + (8U*N) - 1U)                            // amplitude 3rd detection stop
#define LEDDAR_ADDR_FLAG3_START(N) (16U + (8U*N))                               // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG3_STOP(N) (16U + (9U*N) - 1U)                           // last flag
#define LEDDAR_ADDR_DIST4_START(N) (16U + (9U*N))                               // distance to 4th detection
#define LEDDAR_ADDR_DIST4_STOP(N) (16U + (10U*N) - 1U                           // distance to 4th detection end
#define LEDDAR_ADDR_AMP4_START(N) (16U + (10U*N))                               // amplitude 4th detection start
#define LEDDAR_ADDR_AMP4_STOP(N) (16U + (11U*N) - 1U)                           // amplitude 4th detection stop
#define LEDDAR_ADDR_FLAG4_START(N) (16U + (11U*N))                              // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG4_STOP(N)  (16U + (12U*N) - 1U)                         // last flag
#define LEDDAR_ADDR_DIST5_START(N) (16U + (12U*N))                              // distance to 4th detection
#define LEDDAR_ADDR_DIST5_STOP(N) (16U + (13U*N) - 1U)                          // distance to 4th detection end
#define LEDDAR_ADDR_AMP5_START(N) (16U + (13U*N))                               // amplitude 4th detection start
#define LEDDAR_ADDR_AMP5_STOP(N)  (16U + (14U*N) - 1U)                          // amplitude 4th detection stop
#define LEDDAR_ADDR_FLAG5_START(N) (16U + (14U*N))                              // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG5_STOP(N) (16U + (15U*N) - 1U)                          // last flag
#define LEDDAR_ADDR_DIST6_START(N) (16U + (15U*N))                              // distance to 4th detection
#define LEDDAR_ADDR_DIST6_STOP(N) (16U + (16U*N) - 1U)                          // distance to 4th detection end
#define LEDDAR_ADDR_AMP6_START(N) (16U + (16U*N))                               // amplitude 4th detection start
#define LEDDAR_ADDR_AMP6_STOP(N) (16U + (17U*N) - 1U)                           // amplitude 4th detection stop
#define LEDDAR_ADDR_FLAG6_START(N) (16U + (17U*N))                              // Flag of the first detection for each segment: Bit 0: Detection is valid (will always be set)  Bit 1: Detection was the result of object demerging Bit 2: Reserved Bit 3: Detection is saturated
#define LEDDAR_ADDR_FLAG6_STOP(N) (16U + (18U*N) - 1U)                          // last flag
#endif /* end leddar */

/**
 * @brief Read Holding Input request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfRegs;                                                     //3-4
} ModbusReadInputRegsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t quantityOfRegs;                                                     //3-4
}) ModbusReadInputRegsReq_t;
#endif

/**
 * @brief Read Holding Input response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint16_t regValue[];                                                         //2 dynamic memory alloc consider LEDDAR_ADDR_FLAG6_STOP(N)+1 as MAX
} ModbusReadInputRegsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1
   uint16_t regValue[];                                                         //2 dynamic memory alloc consider LEDDAR_ADDR_FLAG6_STOP(N)+1 as MAX
}) ModbusReadInputRegsResp_t;
#endif


typedef union {
    uint32_t holdingReg;
    float32_t danielsFloat;
    uint64_t holdingRegDouble;
    float64_t longFloat;
  } ModbusRTUDaniels_u;                                                         // for read writing float to integer values e.g. for Daniels (Emerson) modbus
  
/**
 Modbus function 0x06 Write Single Register.

 This function code is used to write a single holding register in a
 remote device. The request specifies the address of the register to be
 written. Registers are addressed starting at zero.

 @param WriteAddress address of the holding register (0x0000..0xFFFF)
 @param WriteValue value to be written to holding register (0x0000..0xFFFF)
 @return 0 on success; exception number on failure
 @ingroup register
 
 write register (function code 0x6) page 50 for Leddartech
 Big Endian format
 */
#define MB_FUNC_WRITE_REGISTER  ( 6U )
/**
 * @brief Write Single Register request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode; //0
   uint16_t regAddr;     //1-2
   uint16_t regValue;    //3-4
} ModbusWriteSingleRegReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode; //0
   uint16_t regAddr;     //1-2
   uint16_t regValue;    //3-4
}) ModbusWriteSingleRegReq_t;
#endif

/**
 * @brief Write Single Register response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode; //0
   uint16_t regAddr;     //1-2
   uint16_t regValue;    //3-4
} ModbusWriteSingleRegResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode; //0
   uint16_t regAddr;     //1-2
   uint16_t regValue;    //3-4
}) ModbusWriteSingleRegResp_t;
#endif

/**
 Modbus function 0x10 Write Multiple Registers.

 This function code is used to write a block of contiguous registers (1
 to 123 registers) in a remote device.

 The requested written values are specified in the transmit buffer. Data
 is packed as one word per register.

 @param WriteAddress address of the holding register (0x0000..0xFFFF)
 @param WriteQty quantity of holding registers to write (1..123, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup register
 
 write multiple register (function code 0x10) page 50 for Leddartech
 Big Endian format
 */
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS ( 16U )
/**
 * @brief Write Multiple Registers request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;    //0
   uint16_t startingAddr;   //1-2
   uint16_t quantityOfRegs; //3-4
   uint8_t byteCount;       //5
   uint16_t regValue[];     //6
} ModbusWriteMultipleRegsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;    //0
   uint16_t startingAddr;   //1-2
   uint16_t quantityOfRegs; //3-4
   uint8_t byteCount;       //5
   uint16_t regValue[];     //6
}) ModbusWriteMultipleRegsReq_t;
#endif

/**
 * @brief Write Multiple Registers response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;     //0
   uint16_t startingAddr;    //1-2
   uint16_t quantityOfRegs;  //3-4
} ModbusWriteMultipleRegsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;     //0
   uint16_t startingAddr;    //1-2
   uint16_t quantityOfRegs;  //3-4
}) ModbusWriteMultipleRegsResp_t;
#endif

/**
 Modbus function 0x17 Read Write Multiple Registers.

 This function code performs a combination of one read operation and one
 write operation in a single MODBUS transaction. The write operation is
 performed before the read. Holding registers are addressed starting at
 zero.

 The request specifies the starting address and number of holding
 registers to be read as well as the starting address, and the number of
 holding registers. The data to be written is specified in the transmit
 buffer.

 @param ReadAddress address of the first holding register (0x0000..0xFFFF)
 @param ReadQty quantity of holding registers to read (1..125, enforced by remote device)
 @param WriteAddress address of the first holding register (0x0000..0xFFFF)
 @param WriteQty quantity of holding registers to write (1..121, enforced by remote device)
 @return 0 on success; exception number on failure
 @ingroup register
 
  Read Write Multiple Registers (function code 0x17) page 50 for Leddartech
  Big Endian format
 */
#define MB_FUNC_READWRITE_MULTIPLE_REGISTERS ( 23U )
/**
 * @brief Read/Write Multiple Registers request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t readStartingAddr;                                                   //1-2
   uint16_t quantityToRead;                                                     //3-4
   uint16_t writeStartingAddr;                                                  //5-6
   uint16_t quantityToWrite;                                                    //7-8
   uint8_t writeByteCount;                                                      //9
   uint16_t writeRegValue[];                                                    //10  Dynamic you might want to make the max and define static memory
} ModbusReadWriteMultipleRegsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t readStartingAddr;                                                   //1-2
   uint16_t quantityToRead;                                                     //3-4
   uint16_t writeStartingAddr;                                                  //5-6
   uint16_t quantityToWrite;                                                    //7-8
   uint8_t writeByteCount;                                                      //9
   uint16_t writeRegValue[];                                                    //10  Dynamic you might want to make the max and define static memory
}) ModbusReadWriteMultipleRegsReq_t;
#endif

/**
 * @brief Read/Write Multiple Registers response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t readByteCount;                                                       //1
   uint16_t readRegValue[];                                                     //2 dynamic memory you may want to limit to max and make memory alloc static
} ModbusReadWriteMultipleRegsResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t readByteCount;                                                       //1
   uint16_t readRegValue[];                                                     //2 dynamic memory you may want to limit to max and make memory alloc static
}) ModbusReadWriteMultipleRegsResp_t;
#endif


// Read Exception Status
#define MB_FUNC_DIAG_READ_EXCEPTION (  7U )
/**
 * @brief Exception response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;  //0
   uint8_t exceptionCode; //1
} ModbusExceptionResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;  //0
   uint8_t exceptionCode; //1
}) ModbusExceptionResp_t;
#endif

// Diagnostic
#define MB_FUNC_DIAG_DIAGNOSTIC (  8U )

#define MB_SUBFUNC_LOOPBACK 0U                                                  // 08 0000 <data> = loopback
// this can also be used with subfunction 01
#define MB_SUBFUNC_RESET ( 1U )
#define MB_TESTDAT_RESTART 0U                                                   // 08 0001 0000 resets communication
#define MB_TESTDAT_RESET 0xFF00U                                                // 08 0001 FF00 resets event logs
#define MB_SUBFUNC_DIAG ( 2U )                                                  // 08 0002 0000 call for diagnostic word
#define MB_SUBFUNC_DELIM ( 3U )                                                 // 08 0003 <char> replaces LF with char in ascii mode
#define MB_SUBFUNC_LISTEN ( 4U )                                                // 08 0004 0000 listen only
#define MB_SUBFUNC_DIAG_RESET ( 0x0AU )                                         // resets diagnostic counters below
#define MB_SUBFUNC_DIAG_BUS_MSG 0x0BU                                           // Quantity of messages that the remote device has detected on the communications system since its last restart, clear counters operation, or power–up.  Messages with bad CRC are not taken into account
#define MB_SUBFUNC_DIAG_BUS_ERR 0x0CU                                           // Quantity of CRC errors encountered by the remote device since its last restart, clear counters operation, or power–up.  In case of an error detected on the character level, (overrun, parity error), or in case of a message length < 3 bytes, the receiving device is not able to calculate the CRC.  In such cases, this counter is also incremented
#define MB_SUBFUNC_DIAG_SLAVE_EXC_ERR 0x0DU                                     // Quantity of MODBUS exception error detected by the remote device since its last restart, clear counters operation, or power–up.  It comprises also the error detected in broadcast messages even if an exception message is not returned in this case
#define MB_SUBFUNC_DIAG_SLAVE_MSG 0x0EU                                         // Quantity of messages addressed to the remote device,  including broadcast messages, that the remote device has processed since its last restart, clear counters operation, or power–up.
#define MB_SUBFUNC_DIAG_SLAVE_NORESP 0x0FU                                      // Quantity of messages received by the remote device for which it returned no response (neither a normal response nor an exception response), since its last restart, clear counters operation, or power–up.  Then, this counter counts the number of broadcast messages it has received.
#define MB_SUBFUNC_DIAG_SLAVE_NAK 0x10U                                         // Quantity of messages addressed to the remote device for which it returned a Negative Acknowledge (NAK) exception response, since its last restart, clear counters operation, or power–up.
#define MB_SUBFUNC_DIAG_SLAVE_BUSY 0x11U                                        // Quantity of messages addressed to the remote device for which it returned a Slave Device Busy exception response, since its last restart, clear counters operation, or power–up.
#define MB_SUBFUNC_DIAG_BUS_OVERRUN 0x12U                                       // Quantity of messages addressed to the remote device that it could not handle due to a character overrun condition, since its last restart, clear counters operation, or power–up. A character overrun is caused by data characters arriving at the port faster than they can be stored, or by the loss of a character due to a hardware malfunction
#define MB_SUBFUNC_DIAG_IOP_OVERRUN 0x13U                                       // iop overrun
#define MB_SUBFUNC_DIAG_884_RESET 0x14U                                         // reset counters in 884
#define MB_SUBFUNC_DIAG_MODPLUS 0x15U                                           // modbus plus
#define MB_TESTDAT_MODPLUS_GET 3U                                               // 08 0015 0003 gets the data (53 words as per defined in spec)
#define MB_TESTDAT_MODPLUS_CLR 0xFF00U                                          // 08 0015 0004 resets the data

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;
   uint16_t subfunction;
   uint16_t testdata[];
} ModbusLoopBackReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;
   uint16_t subfunction;
   uint16_t testdata[];
}) ModbusLoopBackReq_t;                                                         // Loop back Request header
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;
   uint16_t subfunction;
   uint16_t testdata[];
} ModbusLoopBackResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;
   uint16_t subfunction;
   uint16_t testdata[];
}) ModbusLoopBackResp_t;                                                        // Loop back Response header
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;
   uint16_t subfunction;
} ModbusDiagReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;
   uint16_t subfunction;
}) ModbusDiagReq_t;                                                             // Diagnostics Request header
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;
   uint16_t subfunction;
} ModbusDiagResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;
   uint16_t subfunction;
}) ModbusDiagResp_t;                                                            // Diagnostics function returns same
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t cont_on_err : 1;                                                    // continues on error
   uint16_t run_light_fail : 1;                                                 // run light fail
   uint16_t tbus_tst_fail : 1;                                                  // tbus test fail
   uint16_t bus_tst_fail : 1;                                                   // bus test fail
   uint16_t force_listen : 1;                                                   // force listen only mode
   uint16_t notused : 2;
   uint16_t rom_tst_fail : 1;                                                   // rom 0 chip test faild
   uint16_t rom_chksum : 1;                                                     // continuous rom checksum
   uint16_t rom1_fail : 1;                                                      // rom chip 1 fail
   uint16_t rom2_fail : 1;
   uint16_t rom3_fail : 1;
   uint16_t ram_5000_fail : 1;                                                  // RAM chip 5000 53FF fail
   uint16_t ram_6000_even_fail : 1;                                             // RAM chip 6000 63FF even addresses failure
   uint16_t ram_6000_odd_fail : 1;                                              // RAM chip 6000 63FF odd addresses failure
   uint16_t timer_chip_failed : 1;                                              // timer chip failed
} Modbus184384DiagResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t cont_on_err : 1;                                                    // continues on error
   uint16_t run_light_fail : 1;                                                 // run light fail
   uint16_t tbus_tst_fail : 1;                                                  // tbus test fail
   uint16_t bus_tst_fail : 1;                                                   // bus test fail
   uint16_t force_listen : 1;                                                   // force listen only mode
   uint16_t notused : 2;
   uint16_t rom_tst_fail : 1;                                                   // rom 0 chip test faild
   uint16_t rom_chksum : 1;                                                     // continuous rom checksum
   uint16_t rom1_fail : 1;                                                      // rom chip 1 fail
   uint16_t rom2_fail : 1;
   uint16_t rom3_fail : 1;
   uint16_t ram_5000_fail : 1;                                                  // RAM chip 5000 53FF fail
   uint16_t ram_6000_even_fail : 1;                                             // RAM chip 6000 63FF even addresses failure
   uint16_t ram_6000_odd_fail : 1;                                              // RAM chip 6000 63FF odd addresses failure
   uint16_t timer_chip_failed : 1;                                              // timer chip failed
}) Modbus184384DiagResp_t;                                                      // 02 call for diagnostics 184/384 response
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t cont_on_err : 1;                                                    // continues on error
   uint16_t run_light_fail : 1;                                                 // run light fail
   uint16_t para_tst_fail : 1;                                                  // parallel port test fail
   uint16_t bus_tst_fail : 1;                                                   // bus test fail
   uint16_t tim0_fail : 1;                                                      // timer 0 fail
   uint16_t tim1_fail : 1;                                                      // timer 1 fail
   uint16_t tim2_fail : 1;                                                      // timer 2 fail
   uint16_t rom_tst_fail : 1;                                                   // rom 0 chip test faild
   uint16_t rom_chksum : 1;                                                     // continuous rom checksum
   uint16_t rom800_fail : 1;                                                    // rom chip 1 fail
   uint16_t rom1000_fail : 1;
   uint16_t rom1800_fail : 1;
   uint16_t ram_4000_fail : 1;                                                  // RAM chip 4000 fail
   uint16_t ram_4100_even_fail : 1;                                             // RAM chip 4100 failure
   uint16_t ram_4200_odd_fail : 1;                                              // RAM chip 4200 failure
   uint16_t ram_4300_failed : 1;                                                // RAM chip 4300 failure
} Modbus484DiagResp_t;                                                          // 02 call for diagnostics 484 response
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t cont_on_err : 1;                                                    // continues on error
   uint16_t run_light_fail : 1;                                                 // run light fail
   uint16_t para_tst_fail : 1;                                                  // parallel port test fail
   uint16_t bus_tst_fail : 1;                                                   // bus test fail
   uint16_t tim0_fail : 1;                                                      // timer 0 fail
   uint16_t tim1_fail : 1;                                                      // timer 1 fail
   uint16_t tim2_fail : 1;                                                      // timer 2 fail
   uint16_t rom_tst_fail : 1;                                                   // rom 0 chip test faild
   uint16_t rom_chksum : 1;                                                     // continuous rom checksum
   uint16_t rom800_fail : 1;                                                    // rom chip 1 fail
   uint16_t rom1000_fail : 1;
   uint16_t rom1800_fail : 1;
   uint16_t ram_4000_fail : 1;                                                  // RAM chip 4000 fail
   uint16_t ram_4100_even_fail : 1;                                             // RAM chip 4100 failure
   uint16_t ram_4200_odd_fail : 1;                                              // RAM chip 4200 failure
   uint16_t ram_4300_failed : 1;                                                // RAM chip 4300 failure
}) Modbus484DiagResp_t;                                                         // 02 call for diagnostics 484 response
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t config_err : 1;                                                     // config illegal
   uint16_t backup_chk_fail : 1;                                                // back up chacksum fail
   uint16_t logic_chk_fail : 1;                                                 // logic checksum fail
   uint16_t node_fail : 1;                                                      // invalid node
   uint16_t traffic_fail : 1;                                                   // traffic invalid
   uint16_t cpu_fail : 1;                                                       // cpu fail
   uint16_t rtc_fail : 1;                                                       // real time clock fail
   uint16_t wdog_fail : 1;                                                      // watch dog faild
   uint16_t no_end_logic : 1;                                                   // no end to the logic
   uint16_t state_ram_fail : 1;                                                 // state ram fail
   uint16_t son_not_start : 1;                                                  // son did not begin
   uint16_t bad_solve_table : 1;                                                // bad order of solve table
   uint16_t illegal_perf : 1;                                                   // illegal perfiferal
   uint16_t dim_aware : 1;                                                      // DIM awareness
   uint16_t spare : 1;                                                          //
   uint16_t perf_port_stop : 1;                                                 // periferal port stop
} Modbus584DiagResp_t;                                                          // 02 call for diagnostics 584 response
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t config_err : 1;                                                     // config illegal
   uint16_t backup_chk_fail : 1;                                                // back up chacksum fail
   uint16_t logic_chk_fail : 1;                                                 // logic checksum fail
   uint16_t node_fail : 1;                                                      // invalid node
   uint16_t traffic_fail : 1;                                                   // traffic invalid
   uint16_t cpu_fail : 1;                                                       // cpu fail
   uint16_t rtc_fail : 1;                                                       // real time clock fail
   uint16_t wdog_fail : 1;                                                      // watch dog faild
   uint16_t no_end_logic : 1;                                                   // no end to the logic
   uint16_t state_ram_fail : 1;                                                 // state ram fail
   uint16_t son_not_start : 1;                                                  // son did not begin
   uint16_t bad_solve_table : 1;                                                // bad order of solve table
   uint16_t illegal_perf : 1;                                                   // illegal perfiferal
   uint16_t dim_aware : 1;                                                      // DIM awareness
   uint16_t spare : 1;                                                          //
   uint16_t perf_port_stop : 1;                                                 // periferal port stop
}) Modbus584DiagResp_t;                                                         // 02 call for diagnostics 584 response
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t iop_overrun : 1;                                                    // iop overrrun
   uint16_t mb_overrun : 1;                                                     // modbus overrun
   uint16_t mb_iop_fail : 1;                                                    // mb iop fail
   uint16_t mb_opt_fail : 1;                                                    // mb options failed
   uint16_t iop_fail : 1;                                                       // iop fail
   uint16_t rem_io_fail : 1;                                                    // remote io fail
   uint16_t cpu_fail : 1;                                                       // main cpu fail
   uint16_t ram_chksum_fail : 1;                                                // ram checksum fail
   uint16_t too_much_logic : 1;                                                 // too much logic
   uint16_t spare : 7;                                                          //
} Modbus884DiagResp_t;                                                          // 02 call for diagnostics 584 response
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t subfunction;
   uint16_t iop_overrun : 1;                                                    // iop overrrun
   uint16_t mb_overrun : 1;                                                     // modbus overrun
   uint16_t mb_iop_fail : 1;                                                    // mb iop fail
   uint16_t mb_opt_fail : 1;                                                    // mb options failed
   uint16_t iop_fail : 1;                                                       // iop fail
   uint16_t rem_io_fail : 1;                                                    // remote io fail
   uint16_t cpu_fail : 1;                                                       // main cpu fail
   uint16_t ram_chksum_fail : 1;                                                // ram checksum fail
   uint16_t too_much_logic : 1;                                                 // too much logic
   uint16_t spare : 7;                                                          //
}) Modbus884DiagResp_t;                                                         // 02 call for diagnostics 584 response
#endif

/**
Report server ID (function code 0x11) This function returns information on the LeddarVu module in the following format:
Number of bytes of information (excluding this one).
Currently 0x99 since the size of information returned is fixed. 
Serial number as an ASCII string 
Run status 0: OFF, 0xFF: ON. Should always return 0 FF, otherwise the module is defective. 
The device name as an ASCII string 
The hardware part number as an ASCII string 
The software part number as an ASCII string 
The full firmware version as 4 16-bit values 
The full bootloader version as 4 16-bit values 
The FPGA-build version 
Internal Use 
Module identification code (9 for the module
Report Slave ID
*/
#define MB_FUNC_OTHER_REPORT_SLAVEID  ( 17U )

#if defined(LEDDARTECH_LIDDAR_USED)
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t stopAddr;                                                           //3-4
} mbCmd11_leddar_req_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t startingAddr;                                                       //1-2
   uint16_t stopAddr;                                                           //3-4
}) mbCmd11_leddar_req_t;
#endif


#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t info;                                                                // 0x99 info is fixed
   unsigned char serialNo[32U];                                                 // serial number
   uint8_t moduleStat;                                                          // should be 00 for fault FF for okay
   unsigned char devName[32U];                                                  // device
   unsigned char hwName[32U];                                                   // hard ware name
   unsigned char swName[32U];                                                   // soft ware name
   uint16_t firmVer[4U];                                                        // firmware version as 4 16 bit values
   uint16_t bootVer[4U];                                                        // bootloader version as 4 16 bit values
   uint16_t fpgaBuild;                                                          // fpga build id
   uint32_t internal;
   uint16_t moduleId;   
} mbCmd11_leddar_resp_t;                                                        // response packet structure
#else
MODPACKED(
typedef struct {
   uint8_t info;                                                                // 0x99 info is fixed
   unsigned char serialNo[32U];                                                 // serial number
   uint8_t moduleStat;                                                          // should be 00 for fault FF for okay
   unsigned char devName[32U];                                                  // device
   unsigned char hwName[32U];                                                   // hard ware name
   unsigned char swName[32U];                                                   // soft ware name
   uint16_t firmVer[4U];                                                        // firmware version as 4 16 bit values
   uint16_t bootVer[4U];                                                        // bootloader version as 4 16 bit values
   uint16_t fpgaBuild;                                                          // fpga build id
   uint32_t internal;
   uint16_t moduleId;                                                           // should read 9
}) mbCmd11_leddar_resp_t;                                                       // response packet structure
#endif

#define LEDDAR_C11_START_ADDR 0U                                                // start address
#define LEDDAR_C11_STOP_ADDR 152U                                               // end address for 0x11 command
#endif /* end leddar */

// Get Com Event Counter
#define MB_FUNC_DIAG_GET_COM_EVENT_CNT ( 11U )                                  // request is function code only
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0   
} ModbusGetComEventReq_t;                                                        // response packet structure
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
}) ModbusGetComEventReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t status;                                                             //1-2
   uint16_t ev_count;                                                           //3-4  
} ModbusGetComEventResp_t;                                                        // response packet structure
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t status;                                                             //1-2
   uint16_t ev_count;                                                           //3-4
}) ModbusGetComEventResp_t;
#endif

//Get Com Event Log
#define MB_FUNC_DIAG_GET_COM_EVENT_LOG ( 12U )                                  // request is function code only

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0 
} ModbusGetEventLogReq_t;                                                        // response packet structure
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
}) ModbusGetEventLogReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t status;                                                             //1-2
   uint16_t ev_count;                                                           //3-4
   uint16_t msg_count;                                                          // 5-6
   uint8_t events[]; 
} ModbusGetEventLogResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t status;                                                             //1-2
   uint16_t ev_count;                                                           //3-4
   uint16_t msg_count;                                                          // 5-6
   uint8_t events[];
}) ModbusGetEventLogResp_t;
#endif

// the events returned have defintion as one of the 4 below
// Remote device MODBUS Receive Event
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t spare : 1;                                                           // Not Used
   uint8_t comm_err : 1;                                                        // Communication Error
   uint8_t spare1 : 2;
   uint8_t char_ovr : 1;                                                        // Character Overrun
   uint8_t listen : 1;                                                          // Currently in Listen Only Mode
   uint8_t broadcst : 1;                                                        // Broadcast Received
   uint8_t type : 1;                                                            // 1 for this type
} ModbusRemoteRcv_t;
#else
MODPACKED(
typedef struct
{
   uint8_t spare : 1;                                                           // Not Used
   uint8_t comm_err : 1;                                                        // Communication Error
   uint8_t spare1 : 2;
   uint8_t char_ovr : 1;                                                        // Character Overrun
   uint8_t listen : 1;                                                          // Currently in Listen Only Mode
   uint8_t broadcst : 1;                                                        // Broadcast Received
   uint8_t type : 1;                                                            // 1 for this type
}) ModbusRemoteRcv_t;
#endif

// Remote device MODBUS Send Event
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t exc_snt : 1;                                                         // Read Exception Sent (Exception Codes 1-3)
   uint8_t abt_snt : 1;                                                         // Server Abort Exception Sent (Exception Code 4)
   uint8_t busy_snt : 1;                                                        // Server Busy Exception Sent (Exception Codes 5-6)
   uint8_t nak_snt : 1;                                                         // Server Program NAK Exception Sent (Exception Code 7)
   uint8_t write_to : 1;                                                        // Write Timeout Error Occurred
   uint8_t listen : 1;                                                          // Currently in Listen Only Mode
   uint8_t type : 1;                                                            // 1 for this type
   uint8_t spare : 1;                                                           //                                                           // 1 for this type
} ModbusRemoteSnd_t;
#else
MODPACKED(
typedef struct
{
   uint8_t exc_snt : 1;                                                         // Read Exception Sent (Exception Codes 1-3)
   uint8_t abt_snt : 1;                                                         // Server Abort Exception Sent (Exception Code 4)
   uint8_t busy_snt : 1;                                                        // Server Busy Exception Sent (Exception Codes 5-6)
   uint8_t nak_snt : 1;                                                         // Server Program NAK Exception Sent (Exception Code 7)
   uint8_t write_to : 1;                                                        // Write Timeout Error Occurred
   uint8_t listen : 1;                                                          // Currently in Listen Only Mode
   uint8_t type : 1;                                                            // 1 for this type
   uint8_t spare : 1;                                                           //
}) ModbusRemoteSnd_t;
#endif


// Remote device Entered Listen Only Mode
#define MB_REMOTE_LISTEN 0x04U
// Remote device Initiated Communication Restart
#define MB_REMOTE_RESTART 0U

/**
 Modbus function 0x16 Mask Write Register.

 This function code is used to modify the contents of a specified holding
 register using a combination of an AND mask, an OR mask, and the
 register's current contents. The function can be used to set or clear
 individual bits in the register.

 The request specifies the holding register to be written, the data to be
 used as the AND mask, and the data to be used as the OR mask. Registers
 are addressed starting at zero.

 The function's algorithm is:
 Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))

 @param WriteAddress address of the holding register (0x0000..0xFFFF)
 @param AndMask AND mask (0x0000..0xFFFF)
 @param OrMask OR mask (0x0000..0xFFFF)
 @return 0 on success; exception number on failure
 @ingroup register
 */
#define MB_MASK_WRITE ( 22U )
/**
 * @brief Mask Write Register request PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;   //0
   uint16_t referenceAddr; //1-2
   uint16_t andMask;       //3-4
   uint16_t orMask;        //5-6
} ModbusMaskWriteRegReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;   //0
   uint16_t referenceAddr; //1-2
   uint16_t andMask;       //3-4
   uint16_t orMask;        //5-6
}) ModbusMaskWriteRegReq_t;
#endif

/**
 * @brief Mask Write Register response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;   //0
   uint16_t referenceAddr; //1-2
   uint16_t andMask;       //3-4
   uint16_t orMask;        //5-6
} ModbusMaskWriteRegResp_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;   //0
   uint16_t referenceAddr; //1-2
   uint16_t andMask;       //3-4
   uint16_t orMask;        //5-6
}) ModbusMaskWriteRegResp_t;
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
/*
Get detections (function code 0x41) This function returns the detections/measurements

returns number of detections then a structure as below for every detection
*/
#define MB_GET_DETECTIONS ( 65U )
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
} ModbusGetDetectionsReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
}) ModbusGetDetectionsReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint16_t distance;                                                           // distance
   uint16_t amplitude;                                                          // amplitude x64
   uint8_t valid : 1U;                                                          // object valid
   uint8_t demerging : 1U;                                                      // demerging
   uint8_t resrved : 1U;
   uint8_t saturated : 1U;                                                      // saturated
   uint8_t spare : 4U;
   uint8_t segment;                                                             // segment number
   uint32_t timestamp;                                                          // ms since device booted
   uint8_t lightpower;                                                          // light source power
   uint16_t bitfield;                                                           // bit field aquisition
} mbCmd41_leddar_detection_t;                                                   // one single detection field (little endian format)
#else
MODPACKED(
typedef struct {
   uint16_t distance;                                                           // distance
   uint16_t amplitude;                                                          // amplitude x64
   uint8_t valid : 1U;                                                          // object valid
   uint8_t demerging : 1U;                                                      // demerging
   uint8_t resrved : 1U;
   uint8_t saturated : 1U;                                                      // saturated
   uint8_t spare : 4U;
   uint8_t segment;                                                             // segment number
   uint32_t timestamp;                                                          // ms since device booted
   uint8_t lightpower;                                                          // light source power
   uint16_t bitfield;                                                           // bit field aquisition
}) mbCmd41_leddar_detection_t;                                                  // one single detection field (little endian format)
#endif

/*
Read module data (function code 0x42) for Leddar Vu
*/
#define MB_READ_LEDDAR_VU  ( 66U )
#endif /* end leddar */
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t minAddr;
   uint8_t maxAddr;
} ModbusReadModuleDataReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t minAddr;
   uint8_t maxAddr;
}) ModbusReadModuleDataReq_t;
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_LEDDAR_MIN_ADDR ( 1U )
#define MB_LEDDAR_MAX_ADDR ( 247U )
#if defined(D_FT900)
typedef struct MODPACKED {
   uint32_t base_addr;                                                          // base address
   uint8_t byte_cnt;                                                            // byte count
} mbCmd42_leddar_send_read_t;                                                   // send a read to the leddar
#else
MODPACKED(
typedef struct {
   uint32_t base_addr;                                                          // base address
   uint8_t byte_cnt;                                                            // byte count
}) mbCmd42_leddar_send_read_t;                                                  // send a read to the leddar
#endif

/*
Write module data (function code 0x43)
*/
#define MB_WRITE_LEDDAR_VU ( 67U )
#endif
/*
Send opcode command (function code 0x44)
*/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t opcode;
} ModbusWriteModuleDataReq_t;                                                   // send a read to the leddar
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t opcode;
}) ModbusWriteModuleDataReq_t;
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_SND_OPCODE_LEDDAR_VU 68U
#define LEDDAR_OPCODE_READ_STATUS 0x5U                                          // valid leddar vu opcodes
#define LEDDAR_OPCODE_WRITE_ENAB 0x6U
#define LEDDAR_OPCODE_WRITE_DISAB 0x4U
#define LEDDAR_OPCODE_RESET_CONFIG 0xC7U
#define LEDDAR_OPCODE_SOFT_RESET 0x99U

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t opcode;                                                              // opcode
   uint8_t value;                                                               // 0x00 for send =opcode sent for reply
} mbCmd44_leddar_opcode_t;                                                      // send and read an opcode message
#else
MODPACKED(
typedef struct {
   uint8_t opcode;                                                              // opcode
   uint8_t value;                                                               // 0x00 for send =opcode sent for reply
}) mbCmd44_leddar_opcode_t;                                                     // send and read an opcode message
#endif

#endif
/*
Get serial port settings (function code 0x45, sub code 0x00)
Set serial port settings (function code 0x45, sub code 0x01)
Get carrier firmware information (function code 0x45, 0x02)
Get carrier device information (function code 0x45, 0x03)
Get CAN port settings (function code 0x45, 0x04)
Set CAN port settings (function code 0x45, 0x05)
*/
#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_PORT_LEDDAR_VU 69U
#endif
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t opcode;
} ModbusGetPortInfoReq_t;
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t opcode;
}) ModbusGetPortInfoReq_t;
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_GET PORT_LEDDAR_VU 0U
#endif
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t subfunc;                                                             // sub function
   uint8_t no_of_ports;                                                         // number of serial ports
   uint8_t port_no;                                                             // port number
   uint32_t baud_rate;                                                          // Baud rate, supported rates:  9,600 19,200 38,400 57,600 115,200
   uint8_t data_sz;                                                             // Date size: 8 = 8-bit size
   uint8_t parity;                                                              // Parity: 0 = None 1 = Odd 2 = Even
   uint8_t stopbit;                                                             // Stop bit: 1 = 1 stop bit 2 = 2 stop bits
   uint8_t flowcont;                                                            // Flow control: 0 = None
   uint8_t mod_addr;                                                            // modbus address
   uint8_t echo;                                                                // Max. echoes per transactions. Used for the Get Detection command (function code 0x41), max. of 40 echoes.
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m  
} mbCmd45_sub0_reply_t;                                                         // reply from a 0x45 0x00 command
#else
MODPACKED(
typedef struct {
   uint8_t subfunc;                                                             // sub function
   uint8_t no_of_ports;                                                         // number of serial ports
   uint8_t port_no;                                                             // port number
   uint32_t baud_rate;                                                          // Baud rate, supported rates:  9,600 19,200 38,400 57,600 115,200
   uint8_t data_sz;                                                             // Date size: 8 = 8-bit size
   uint8_t parity;                                                              // Parity: 0 = None 1 = Odd 2 = Even
   uint8_t stopbit;                                                             // Stop bit: 1 = 1 stop bit 2 = 2 stop bits
   uint8_t flowcont;                                                            // Flow control: 0 = None
   uint8_t mod_addr;                                                            // modbus address
   uint8_t echo;                                                                // Max. echoes per transactions. Used for the Get Detection command (function code 0x41), max. of 40 echoes.
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m                                                              // number of serial ports
}) mbCmd45_sub0_reply_t;                                                        // reply from a 0x45 0x00 command
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_SET_PORT_LEDDAR_VU 1U
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t port_no;                                                             // port number
   uint32_t baud_rate;                                                          // Baud rate, supported rates:  9,600 19,200 38,400 57,600 115,200
   uint8_t data_sz;                                                             // Date size: 8 = 8-bit size
   uint8_t parity;                                                              // Parity: 0 = None 1 = Odd 2 = Even
   uint8_t stopbit;                                                             // Stop bit: 1 = 1 stop bit 2 = 2 stop bits
   uint8_t flowcont;                                                            // Flow control: 0 = None
   uint8_t mod_addr;                                                            // modbus address
   uint8_t echo;                                                                // Max. echoes per transactions. Used for the Get Detection command (function code 0x41), max. of 40 echoes.
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m  
} mbCmd45_sub1_reply_t;                                                        // reply from a 0x45 0x01 command
#else
MODPACKED(
typedef struct {
   uint8_t port_no;                                                             // port number
   uint32_t baud_rate;                                                          // Baud rate, supported rates:  9,600 19,200 38,400 57,600 115,200
   uint8_t data_sz;                                                             // Date size: 8 = 8-bit size
   uint8_t parity;                                                              // Parity: 0 = None 1 = Odd 2 = Even
   uint8_t stopbit;                                                             // Stop bit: 1 = 1 stop bit 2 = 2 stop bits
   uint8_t flowcont;                                                            // Flow control: 0 = None
   uint8_t mod_addr;                                                            // modbus address
   uint8_t echo;                                                                // Max. echoes per transactions. Used for the Get Detection command (function code 0x41), max. of 40 echoes.
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m                                                              // number of serial ports
}) mbCmd45_sub1_reply_t;                                                        // reply from a 0x45 0x01 command
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_GET_FIRM_LEDDAR_VU 2U
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t opcode ;                                                             // opcode
   unsigned char firmWare[32U];                                                 // firmware part number
   uint64_t version;                                                            // firmware version 
} mbCmd45_sub2_reply_t;
#else
MODPACKED(
typedef struct {
   uint8_t opcode ;                                                             // opcode
   unsigned char firmWare[32U];                                                 // firmware part number
   uint64_t version;                                                            // firmware version                                                             // number of serial ports
}) mbCmd45_sub2_reply_t;                                                        // reply from a 0x45 0x02 command
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_GET_CAR_DEV_LEDDAR_VU 3U
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t opcode ;                                                             // opcode
   unsigned char hwPart[32U];                                                   // hardware part number
   unsigned char serialNo[32U];                                                 // serial number
   uint32_t option;                                                             // option number   
} mbCmd45_sub3_reply_t;                                                         // reply from a 0x45 0x03 command
#else
MODPACKED(
typedef struct {
   uint8_t opcode ;                                                             // opcode
   unsigned char hwPart[32U];                                                   // hardware part number
   unsigned char serialNo[32U];                                                 // serial number
   uint32_t option;                                                             // option number                                                             // number of serial ports
}) mbCmd45_sub3_reply_t;                                                        // reply from a 0x45 0x03 command
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_GET_CAN_PORT_LEDDAR_VU 4U
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t opcode;                                                              // opcode
   uint8_t No_of_port;                                                          // number of ports
   uint8_t logical_port;                                                        // logical port
   uint32_t baud;                                                               // Baud rate, supported rates: 10,000 20,000 50,000 100,000 125,000 250,000 500,000 1,000,000
   uint8_t frame_fmt;                                                           // Frame format: 0 = Standard 11 bits 1 = Extended 29 bits
   uint32_t tx_base;                                                            // Tx base ID
   uint32_t rx_base;                                                            // Rx base ID
   uint8_t max_det;                                                             // Maximum number of detections (measurements) returned per CAN detection message transaction: 1 through 96
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m
   uint16_t msg_delay;                                                          // Inter-message delay 0 through 65535 milliseconds
   uint16_t cyc_delay;                                                          // Inter-cycle delay 0 through 65535 milliseconds   
} mbCmd45_sub4_reply_t;                                                         // reply from a 0x45 0x04 command
#else
MODPACKED(
typedef struct {
   uint8_t opcode;                                                              // opcode
   uint8_t No_of_port;                                                          // number of ports
   uint8_t logical_port;                                                        // logical port
   uint32_t baud;                                                               // Baud rate, supported rates: 10,000 20,000 50,000 100,000 125,000 250,000 500,000 1,000,000
   uint8_t frame_fmt;                                                           // Frame format: 0 = Standard 11 bits 1 = Extended 29 bits
   uint32_t tx_base;                                                            // Tx base ID
   uint32_t rx_base;                                                            // Rx base ID
   uint8_t max_det;                                                             // Maximum number of detections (measurements) returned per CAN detection message transaction: 1 through 96
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m
   uint16_t msg_delay;                                                          // Inter-message delay 0 through 65535 milliseconds
   uint16_t cyc_delay;                                                          // Inter-cycle delay 0 through 65535 milliseconds
}) mbCmd45_sub4_reply_t;                                                        // reply from a 0x45 0x04 command
#endif

#if defined(LEDDARTECH_LIDDAR_USED)
#define MB_SET_CAN_PORT_LEDDAR_VU 5U
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t set;                                                                 // Settings of corresponding logical CAN port number to set
   uint32_t baud;                                                               // Baud rate, supported rates: 10,000 20,000 50,000 100,000 125,000 250,000 500,000 1,000,000
   uint8_t frame_fmt;                                                           // Frame format: 0 = Standard 11 bits 1 = Extended 29 bits
   uint32_t tx_base;                                                            // Tx base ID
   uint32_t rx_base;                                                            // Rx base ID
   uint8_t max_det;                                                             // Maximum number of detections (measurements) returned per CAN detection message transaction: 1 through 96
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m
   uint16_t msg_delay;                                                          // Inter-message delay 0 through 65535 milliseconds
   uint16_t cyc_delay;                                                          // Inter-cycle delay 0 through 65535 milliseconds  
} mbCmd45_sub5_reply_t;                                                        // reply from a 0x45 0x05 command
#else
MODPACKED(
typedef struct {
   uint8_t set;                                                                 // Settings of corresponding logical CAN port number to set
   uint32_t baud;                                                               // Baud rate, supported rates: 10,000 20,000 50,000 100,000 125,000 250,000 500,000 1,000,000
   uint8_t frame_fmt;                                                           // Frame format: 0 = Standard 11 bits 1 = Extended 29 bits
   uint32_t tx_base;                                                            // Tx base ID
   uint32_t rx_base;                                                            // Rx base ID
   uint8_t max_det;                                                             // Maximum number of detections (measurements) returned per CAN detection message transaction: 1 through 96
   uint16_t dist_res;                                                           // Distance resolution: ? 1 = m ? 10 = dm ? 100 = cm ? 1,000 = m
   uint16_t msg_delay;                                                          // Inter-message delay 0 through 65535 milliseconds
   uint16_t cyc_delay;                                                          // Inter-cycle delay 0 through 65535 milliseconds
}) mbCmd45_sub5_reply_t;                                                        // reply from a 0x45 0x05 command
#endif

// Encapsulated Interface Transport  CANopen General Reference Request and Response PD
#define MB_READ_DEVICE_ID 43U
#define MB_READ_DEVICE_ID2 14U                                                  // alternate command for some vendor

#define MB_MEI_TYPE 0x0DU                                                       //  MODBUS Assigned Number licensed to CiA for the CANopen general reference
#define MB_MEI_TYPE1 0x0EU                                                      // alternate for some

#define MB_OBJ_ID_VENDOR_NM 0x00U                                               // object id's
#define MB_OBJ_ID_PROD_CODE 0x01U
#define MB_OBJ_ID_MAJ_MIN_REV 0x02U
#define MB_OBJ_ID_VENDOR_URL 0x03U
#define MB_OBJ_ID_PROD_NM 0x04U
#define MB_OBJ_ID_MOD_NM 0x05U
#define MB_OBJ_ID_USER_APP_NM 0x06U

#define MB_CONFORM_BASIC 0x01U                                                  // basic identification (stream access only) 
#define MB_CONFORM_REG 0x02U                                                    // regular identification (stream access only) 
#define MB_CONFORM_EXT 0x03U                                                    // extended identification (stream access only)
#define MB_CONFORM_BASIC_STREAM 0x81U                                           // basic identification (stream access and individual access)
#define MB_CONFORM_REG_STREAM 0x82U                                             // regular identification (stream access and individual access) 
#define MB_CONFORM_EXT_STREAM 0x83U                                             // extended identification(stream access and individual

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t MEIData[];                                                           // dynamic consider fixing to what you want max size 253 byte 
} ModbusEITReq_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t MEIData[];                                                           // dynamic consider fixing to what you want max size 253 byte
}) ModbusEITReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t MEIData[];                                                           // dynamic consider fixing to what you want max size 253 byte 
} ModbusEITResp_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t MEIData[];                                                           // dynamic consider fixing to what you want max size 253 byte
}) ModbusEITResp_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t devId;                                                               //3-4 allowable values 01 / 02 / 03 / 04
   uint8_t objId;                                                               //5-6 0x00 to 0xFF known ones are defined above
} ModbusDeviceInfoReq_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t devId;                                                               //3-4 allowable values 01 / 02 / 03 / 04
   uint8_t objId;                                                               //5-6 0x00 to 0xFF known ones are defined above
}) ModbusDeviceInfoReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t id;                                                                  // 0 object id
   uint8_t len;                                                                 // 1 object length in bytes
   unsigned char value[];                                                       // value can be typically a string
} ModbusObject_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t id;                                                                  // 0 object id
   uint8_t len;                                                                 // 1 object length in bytes
   unsigned char value[];                                                       // value can be typically a string
}) ModbusObject_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t devId;                                                               //3-4 allowable values 01 / 02 / 03 / 04
   uint8_t conformity;                                                          //5-6 conformity
   uint8_t more_follows;                                                        // 00 / FF
   uint8_t next_obj_id;
   uint8_t number_obj;
   ModbusObject_t objects[];                                                    // modbus objects returned
} ModbusDeviceInfoResp_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t MEIType;                                                             //1-2
   uint8_t devId;                                                               //3-4 allowable values 01 / 02 / 03 / 04
   uint8_t conformity;                                                          //5-6 conformity
   uint8_t more_follows;                                                        // 00 / FF
   uint8_t next_obj_id;
   uint8_t number_obj;
   ModbusObject_t objects[];                                                    // modbus objects returned
}) ModbusDeviceInfoResp_t;
#endif

// Read FIFO Queue
#define MB_READ_FIFO_Q 24U

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint16_t fifoPointerAddr;                                                    //1-2
} ModbusFIFOQueReq_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint16_t fifoPointerAddr;                                                    //1-2
}) ModbusFIFOQueReq_t;
#endif

/**
 * @brief Mask Write Register response PDU
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;   //0
   uint16_t byteCount;                                                          //1-2
   uint16_t fifoCount;                                                          //3-4
   uint16_t fifoValues[];                                                       //5-6
} ModbusFIFOQueResp_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;   //0
   uint16_t byteCount;                                                          //1-2
   uint16_t fifoCount;                                                          //3-4
   uint16_t fifoValues[];                                                       //5-6
}) ModbusFIFOQueResp_t;
#endif

// Read File Record
#define MB_READ_FILE_REC 20U

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t refType;                                                             //0
   uint16_t fileNumber;                                                         //1-2
   uint16_t recNumber;                                                          //3-4
   uint16_t recLen;                                                             //5-6
} FileRecord_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t refType;                                                             //0
   uint16_t fileNumber;                                                         //1-2
   uint16_t recNumber;                                                          //3-4
   uint16_t recLen;                                                             //5-6
}) FileRecord_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
} ModbusReadFileRecReq_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
}) ModbusReadFileRecReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t respLen;                                                             //1-2
   uint8_t fileRespLen;                                                         //3-4
   uint8_t refType;                                                             //5-6
   uint16_t recData[]; 
} ModbusReadFileRecResp_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t respLen;                                                             //1-2
   uint8_t fileRespLen;                                                         //3-4
   uint8_t refType;                                                             //5-6
   uint16_t recData[];                                                          //
}) ModbusReadFileRecResp_t;
#endif

//Write File Record
#define MB_WRITE_FILE_REC 21U

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
} ModbusWriteFileRecReq_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
}) ModbusWriteFileRecReq_t;
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
   uint16_t fileData[];
} ModbusWriteFileRecReqResp_t;                                                        
#else
MODPACKED(
typedef struct
{
   uint8_t functionCode;                                                        //0
   uint8_t byteCount;                                                           //1-2
   FileRecord_t fileRecords[];
   uint16_t fileData[];
}) ModbusWriteFileRecReqResp_t;
#endif

/* Modbus BIN implementation */
typedef union
{
   ModbusReadCoilsReq_t func01;
   ModbusReadDiscreteInputsReq_t func02;
   ModbusWriteSingleCoilReq_t func05;
   ModbusWriteMultipleCoilsReq_t func15;
   ModbusReadHoldingRegsReq_t func03;
   ModbusReadInputRegsReq_t func04;
   ModbusWriteSingleRegReq_t func06;
   ModbusWriteMultipleRegsReq_t func16;
   ModbusReadWriteMultipleRegsReq_t func23;
   ModbusLoopBackReq_t func8_loop;
   ModbusDiagReq_t func8_diag;
   mbCmd11_leddar_req_t func17;
   ModbusGetComEventReq_t func11;
   ModbusGetEventLogReq_t func12;
   ModbusMaskWriteRegReq_t func22;
   ModbusGetDetectionsReq_t func65;
   ModbusReadModuleDataReq_t func66;
   ModbusWriteModuleDataReq_t func67;
   ModbusGetPortInfoReq_t func69;
   ModbusFIFOQueReq_t func24;
   ModbusReadFileRecReq_t func20;
   ModbusWriteFileRecReq_t func21;
} mb_SndMessages_u;

#if defined(D_FT900)
typedef struct MODPACKED {
    uint8_t STartBegin;                                                         // MB_BIN_START {
    mb_SndMessages_u mbusBody;
    uint8_t STopEnd;                                                            // MB_BIN_START }
} ModbusBINReq_t;                                                        
#else
MODPACKED(
typedef struct
{
    uint8_t STartBegin;                                                         // MB_BIN_START {
    mb_SndMessages_u mbusBody;
    uint8_t STopEnd;                                                            // MB_BIN_START }
}) ModbusBINReq_t;                                                              // modbus binary BIN sender structure, ensure you duplicate start stop chars in the payload before transferring to send buffer
#endif

#if defined(D_FT900)
typedef struct MODPACKED {
    uint8_t STartBegin;                                                         // MB_BIN_START {
    uint8_t PayloadBody[];
    uint8_t STopEnd;                                                            // MB_BIN_START }
} ModbusBINSend_t;                                                        
#else
MODPACKED(
typedef struct
{
    uint8_t STartBegin;                                                         // MB_BIN_START {
    uint8_t PayloadBody[];
    uint8_t STopEnd;                                                            // MB_BIN_START }
}) ModbusBINSend_t;                                                             // use this struct to add the duplciates to the payload body
#endif

typedef union
{
   ModbusReadCoilsResp_t func01;
   ModbusReadDiscreteInputsResp_t func02;
   ModbusWriteSingleCoilResp_t func05;
   ModbusWriteMultipleCoilsResp_t func15;
   ModbusReadHoldingRegsResp_t func03;
   ModbusReadInputRegsResp_t func04;
   ModbusWriteSingleRegResp_t func06;
   ModbusWriteMultipleRegsResp_t func16;
   ModbusReadWriteMultipleRegsResp_t func23;
   ModbusExceptionResp_t func7;
   ModbusLoopBackResp_t func8_loop;
   ModbusDiagResp_t func8_diag;
   Modbus484DiagResp_t func8_484;
   Modbus584DiagResp_t func8_584;
   Modbus884DiagResp_t func8_884;
   Modbus184384DiagResp_t func8_184384;
   mbCmd11_leddar_resp_t func17;
   ModbusGetComEventResp_t func11;
   ModbusGetEventLogResp_t func12;
   ModbusMaskWriteRegResp_t func22;
   mbCmd41_leddar_detection_t func65;
   mbCmd42_leddar_send_read_t func66;
   mbCmd44_leddar_opcode_t func67;
   mbCmd45_sub0_reply_t func69sub0;
   mbCmd45_sub1_reply_t func69sub1;
   mbCmd45_sub2_reply_t func69sub2;
   mbCmd45_sub3_reply_t func69sub3;
   mbCmd45_sub4_reply_t func69sub4;
   mbCmd45_sub5_reply_t func69sub5;
   ModbusFIFOQueResp_t func24;
   ModbusReadFileRecResp_t func20;
   ModbusWriteFileRecReqResp_t func21;
} mb_RcvMessages_u;

#if defined(D_FT900)
typedef struct MODPACKED {
    uint8_t STartBegin;                                                         // MB_BIN_START {
    mb_RcvMessages_u mbusBody;
    uint8_t STopEnd;                                                            // MB_BIN_START }
} ModbusBINRcv_t;                                                        
#else
MODPACKED(
typedef struct
{
    uint8_t STartBegin;                                                         // MB_BIN_START {
    mb_RcvMessages_u mbusBody;
    uint8_t STopEnd;                                                            // MB_BIN_START }
}) ModbusBINRcv_t;                                                              // modbus BIN receiver ensure you strip the duplicate Start Stop Delimters before memcpy of data into here (to do inside interrupt)
#endif

#define MB_FUNC_ERROR                         ( 128U )
/* ----------------------- Type definitions ---------------------------------*/

typedef enum                                                                    // alternative to the define
{
   MODBUS_FUNCTION_READ_COILS                = 1,
   MODBUS_FUNCTION_READ_DISCRETE_INPUTS      = 2,
   MODBUS_FUNCTION_READ_HOLDING_REGS         = 3,
   MODBUS_FUNCTION_READ_INPUT_REGS           = 4,
   MODBUS_FUNCTION_WRITE_SINGLE_COIL         = 5,
   MODBUS_FUNCTION_WRITE_SINGLE_REG          = 6,
   MODBUS_FUNCTION_READ_EXCEPTION_STATUS     = 7,
   MODBUS_FUNCTION_DIAGNOSTICS               = 8,
   MODBUS_FUNCTION_GET_COMM_EVENT_COUNTER    = 11,
   MODBUS_FUNCTION_GET_COMM_EVENT_LOG        = 12,
   MODBUS_FUNCTION_WRITE_MULTIPLE_COILS      = 15,
   MODBUS_FUNCTION_WRITE_MULTIPLE_REGS       = 16,
   MODBUS_FUNCTION_REPORT_SLAVE_ID           = 17,
   MODBUS_FUNCTION_READ_FILE_RECORD          = 20,
   MODBUS_FUNCTION_WRITE_FILE_RECORD         = 21,
   MODBUS_FUNCTION_MASK_WRITE_REG            = 22,
   MODBUS_FUNCTION_READ_WRITE_MULTIPLE_REGS  = 23,
   MODBUS_FUNCTION_READ_FIFO_QUEUE           = 24,
   MODBUS_FUNCTION_GET_DETECTIONS            = 65,
   MODBUS_FUNCTION_READ_LEDDAR               = 66,
   MODBUS_FUNCTION_WRITE_LEDDAR              = 67,
   MODBUS_FUNCTION_OPCODE_LEDDAR             = 68,
   MODBUS_FUNCTION_PORT_LEDDAR               = 69,
   MODBUS_FUNCTION_ENCAPSULATED_IF_TRANSPORT = 43
} eMBFunctionCode;

typedef enum
{
    MB_EX_NONE = 0x00U,                                                         // MBSuccess
    MB_EX_ILLEGAL_FUNCTION = 0x01U,                                             // MBIllegalFunction
    MB_EX_ILLEGAL_DATA_ADDRESS = 0x02U,                                         // MBIllegalDataAddress A request for a register that does not exist will return error code 2
    MB_EX_ILLEGAL_DATA_VALUE = 0x03U,                                           // MBIllegalDataValue Trying to set a register to an invalid value will return error code 3
    MB_EX_SLAVE_DEVICE_FAILURE = 0x04U,                                         // MBSlaveDeviceFailure If an error occurs while trying to execute the function, error code 4 will be returned
    MB_EX_ACKNOWLEDGE = 0x05U,                                                  // Acknowledge
    MB_EX_SLAVE_BUSY = 0x06U,                                                   // Slave Device Busy
    MB_EX_NACK = 0x07U,                                                         // NACK
    MB_EX_MEMORY_PARITY_ERROR = 0x08U,                                          // Memory Parity Error
    MB_EX_GATEWAY_PATH_FAILED = 0x0AU,                                          // Gateway Path Unavailable
    MB_EX_GATEWAY_TGT_FAILED = 0x0BU                                            // Gateway Target Device Failed to Respond
} eMBException;

typedef enum
{
   MB_COIL_STATE_OFF = 0x0000,
   MB_COIL_STATE_ON  = 0xFF00
} eMBCoilState;

typedef enum
{
    MBSuccess                    = 0x00U,                                       // no error occurred
    MBInvalidSlaveID             = 0xE0U,                                       // wrong slave id
    MBInvalidFunction            = 0xE1U,                                       // invalid function code sent
    MBResponseTimedOut           = 0xE2U,                                       // time out
    MBInvalidCRC                 = 0xE3U                                        // wrong crc
} eMBClassException;

/*!
 * Constants which defines the format of a modbus frame. The example is
 * shown for a Modbus RTU/ASCII frame. Note that the Modbus PDU is not
 * dependent on the underlying transport.
 *
 * <code>
 * <------------------------ MODBUS SERIAL LINE PDU (1) ------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+----------------------------+-------------+
 *  | Address   | Function Code | Data                       | CRC/LRC     |
 *  +-----------+---------------+----------------------------+-------------+
 *  |           |               |                                   |
 * (2)        (3/2')           (3')                                (4)
 *
 * (1)  ... MB_SER_PDU_SIZE_MAX = 256
 * (2)  ... MB_SER_PDU_ADDR_OFF = 0
 * (3)  ... MB_SER_PDU_PDU_OFF  = 1
 * (4)  ... MB_SER_PDU_SIZE_CRC = 2
 *
 * (1') ... MB_PDU_SIZE_MAX     = 253
 * (2') ... MB_PDU_FUNC_OFF     = 0
 * (3') ... MB_PDU_DATA_OFF     = 1
 * </code>
 */

/* ----------------------- Defines ------------------------------------------*/

#define MB_RTU_PDU_SIZE_MAX  253U                                               /*!< Maximum size of a PDU. */
#define MB_RTU_PDU_SIZE_MIN  1U                                                 /*!< Function Code */
#define MB_ASC_PDU_SIZE_MIN  3U                                                 /*!< Minimum size of a Modbus ASCII frame. */
#define MB_ASC_PDU_SIZE_MAX  256U                                               /*!< Maximum size of a Modbus ASCII frame. */

#define MB_PDU_FUNC_OFF 0U                                                      /*!< Offset of function code in PDU. */
#define MB_PDU_DATA_OFF 1U                                                      /*!< Offset for response data in PDU. */

// modbus ascii
#define MB_ASCII_START :                                                        // colon is start char
#define MB_ASCII_DEFAULT_CR '\r'                                                /*!< Default CR character for Modbus ASCII. */
#define MB_ASCII_DEFAULT_LF '\n'                                                /*!< Default LF character for Modbus ASCII. */
#define MB_SER_PDU_SIZE_LRC 1U                                                  /*!< Size of LRC field in PDU. includes the : and the CRLF */
#define MB_SER_PDU_ADDR_OFF 0U                                                  /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF 1U                                                   /*!< Offset of Modbus-PDU in Ser-PDU. */

// modbus BIN If a byte with the value 123 (0x7B) or with the value 125 (0x7D) occurs it has to be duplicated (i.e. written twice)..
#define MB_BIN_START 0x7B                                                       // modbus bin is in binary like rtu but within braces { 01 01 01 44 } pauses allowed
#define MB_BIN_STOP 0x7D

// modbus tcp
#define MB_TCP_PORT 502U                                                        //Modbus/TCP port number
#define MB_TCP_SECURE_PORT 802U                                                 //Secure Modbus/TCP port number

#define MB_PROTOCOL_ID 0U                                                       //Modbus protocol identifier
#define MB_DEFAULT_UNIT_ID 255U                                                 //Default unit identifier

#define MB_MAX_PDU_SIZE 253U                                                    //Maximum size of Modbus PDU
#define MB_MAX_ADU_SIZE 260U                                                    //Maximum size of Modbus/TCP ADU

#define MB_FUNCTION_CODE_MASK 0x7FU                                             //Function code mask
#define MB_EXCEPTION_MASK 0x80U                                                 //Exception response mask

#define MB_SET_COIL(a, n) ((a)[(n) / 8U] |= (1U << ((n) % 8U)))                 //Set coil value
#define MB_RESET_COIL(a, n) ((a)[(n) / 8U] &= ~(1U << ((n) % 8U)))              //Reset coil value
#define MB_TEST_COIL(a, n) ((a[(n) / 8U] >> ((n) % 8U)) & 1U)                   //Test coil value

/**
 * @brief MBAP header (Modbus Application Protocol)
 **/
#if defined(D_FT900)
typedef struct MODPACKED {
   uint16_t transactionId;                                                      // 0-1 the invocation identification (2 bytes) used for transaction pairing; formerly called transaction identifier
   uint16_t protocolId;                                                         // 2-3 the protocol identifier (2 bytes), is 0 for Modbus by default; reserved for future extensions
   uint16_t length;                                                             // 4-5  the length (2 bytes), a byte count of all following bytes
   uint8_t unitId;                                                              // 6 the unit identifier (1 byte) used to identify a remote unit located on a non-TCP/IP network
   uint8_t pdu[];                                                               // 7 payload dynamically assigned may wish to make size of max_pdu or what you are sending
} ModbusEthHeader;                                                              // for modbus tcp or udp                                                        
#else
MODPACKED(
typedef struct
{
   uint16_t transactionId;                                                      // 0-1 the invocation identification (2 bytes) used for transaction pairing; formerly called transaction identifier
   uint16_t protocolId;                                                         // 2-3 the protocol identifier (2 bytes), is 0 for Modbus by default; reserved for future extensions
   uint16_t length;                                                             // 4-5  the length (2 bytes), a byte count of all following bytes
   uint8_t unitId;                                                              // 6 the unit identifier (1 byte) used to identify a remote unit located on a non-TCP/IP network
   uint8_t pdu[];                                                               // 7 payload dynamically assigned may wish to make size of max_pdu or what you are sending
}) ModbusEthHeader;                                                             // for modbus tcp or udp
#endif

#define ENRON_ADDR_BOOL_START 0x03E9U                                           // Address boundaries as defined by ENRON
#define ENRON_ADDR_BOOL_STOP 0x07CF
#define ENRON_ADDR_UINT16_START 0x0BB9U
#define ENRON_ADDR_UINT16_STOP 0x0F9FU
#define ENRON_ADDR_UINT32_START 0x1389U
#define ENRON_ADDR_UINT32_STOP 0x176FU
#define ENRON_ADDR_FLOAT32_START 0x1B59U
#define ENRON_ADDR_FLOAT32_STOP 0x1F3FU

//-------------------- Application Addresses -----------------------------------
#if defined(NOVUS_DR2A_USED)
// %%%%%%%%%%% NOVUS DigiRail-2A Universal Analog Input Module %%%%%%%%%%%%%%%
#define NOVUS_DR2A_DEF_ADDR 247u                                                // baud 1200 parity even
//READ INPUT REGISTERS - 04H
#define NOVUS_DR2A_PV_CH1 0u                                                    // PV of Channel 1 in percentage. Range from 0 to 62000.
#define NOVUS_DR2A_PV_CH2 1u                                                    // PV of Channel 2 in percentage. Range from 0 to 62000.
#define NOVUS_DR2A_ENGV_CH1 5u                                                  // PV of Channel 1 in engineering unit. Range: defined by the limits of the temperature sensor, or by the indication limits as defined by the parameters 42 and 43 of the Holding Registers.
#define NOVUS_DR2A_ENGV_CH2 6u

//READ HOLDING REGISTERS – 03H
//WRITE SINGLE REGISTER – 06H
#define NOVUS_DR2A_CH1_LOW 41u                                                  //Lower indication limit of channel 1 when input type is linear (RW)
#define NOVUS_DR2A_CH1_HIGH 42u                                                 //upper indication limit of channel 1 when input type is linear (RW)
#define NOVUS_DR2A_CH2_LOW 43u                                                  //Lower indication limit of channel 2 when input type is linear (RW)
#define NOVUS_DR2A_CH2_HIGH 44u                                                 //upper indication limit of channel 2 when input type is linear (RW)
#define NOVUS_DR2A_MOD_ADDR 3u                                                  // module address (RW)
#define NOVUS_DR2A_FILTER 8u                                                    // 0-20 filter (RW)
#define NOVUS_DR2A_CHSTAT 7u                                                    // channel status (RO)
#define NOVUS_DR2A_SQRT1 36u                                                    // Square root fitness for channel 1 (0 or 1) (RW)
#define NOVUS_DR2A_SQRT2 37u                                                    // Square root fitness for channel 2 (0 or 1) (RW)
#define NOVUS_DR2A_TYPE1 21u                                                    // Type for channel 1 (enum Novus_Dr2A_Typ_e ) (RW)
#define NOVUS_DR2A_TYPE2 22u                                                    // Type for channel 2 (enum Novus_Dr2A_Typ_e ) (RW)

typedef enum {NDR2A_DISABLED = -1, NDR2A_TCJ = 0, HDR2A_TCK=1, NDR2A_TC=2, NDR2A_TCE=3, NDR2A_TCN=4, NDR2A_TCR=5, NDR2A_TCS=6, NDR2A_TCB=7, NDR2A_Pt100=8, NDR2A_50mV=9, NDR2A_20mV=10, NDR2A_minus10to20mV=11, NDR2A_5V=12, NDR2A_10V=13, NDR2A_0to20mA=18, NDR2A_4to20mA=19 } Novus_Dr2A_Typ_e;
typedef enum {NDR2A_ulDISABLED = -1, NDR2A_ulTCJ = 940, HDR2A_ulTCK=1370, NDR2A_ulTCT=400, NDR2A_ulTCE=720, NDR2A_ulTCN=1300, NDR2A_ulTCR=1760, NDR2A_ulTCS=1760, NDR2A_ulTCB=1800, NDR2A_ulPt100=650, NDR2A_ul50mV=31000, NDR2A_ul20mV=31000, NDR2A_ulminus10to20mV=31000, NDR2A_ul5V=31000, NDR2A_ul10V=31000, NDR2A_ul0to20mA=31000, NDR2A_ul4to20mA=31000 } Novus_Dr2A_UpLim_e;
typedef enum {NDR2A_loDISABLED = -1, NDR2A_loTCJ = -130, HDR2A_loTCK=-200, NDR2A_loTC=-200, NDR2A_loTCE=-100, NDR2A_loTCN=-200, NDR2A_loTCR=0, NDR2A_loTCS=0, NDR2A_loTCB=500, NDR2A_loPt100=-200, NDR2A_lo50mV=0, NDR2A_lo20mV=0, NDR2A_lominus10to20mV=0, NDR2A_loul5V=0, NDR2A_lo10V=0, NDR2A_lo0to20mA=0, NDR2A_lo4to20mA=0 } Novus_Dr2A_LoLim_e;

#endif /* DR2A */

#if defined(NOVUS_DR2R_USED)
// %%%%%%%%%%% NOVUS DigiRail-2R Universal 2 relay out Module %%%%%%%%%%%%%%%
//WRITE SINGLE COIL - 05H
#define NOVUS_DR2R_ROT1_STAT 0u
#define NOVUS_DR2R_ROT2_STAT 1u

//WRITE MULTIPLE COILS – 0FH
#define NOVUS_DR2R_ROT_BEGIN 0u
#define NOVUS_DR2R_ROT_END 1u

//READ HOLDING REGISTERS – 03H  PLC 40001 = 0 WRITE SINGLE REGISTER – 06H
#define NOVUS_DR2R_ROT_STAT 7u                                                  // bit 1 = out 2 bit 0 = out1
#define NOVUS_DR2R_ROT1_TH 12u                                                  // Value of timing for digital output 1, in hundredths of seconds - word high
#define NOVUS_DR2R_ROT1_LH 13u                                                  // Value of timing for digital output 1, in hundredths of seconds - word low
#define NOVUS_DR2R_ROT2_TH 14u                                                  // Value of timing for digital output 2, in hundredths of seconds - word high
#define NOVUS_DR2R_ROT2_LH 15u                                                  // Value of timing for digital output 2, in hundredths of seconds - word low
#define NOVUS_DR2R_ROT1_CurTH 20u                                               // for deactivating digital output 1, in hundredths of seconds - word high
#define NOVUS_DR2R_ROT1_CurLH 21u                                               // for deactivating digital output 1, in hundredths of seconds - word low
#define NOVUS_DR2R_ROT2_CurTH 22u                                               // for deactivating digital output 2, in hundredths of seconds - word high
#define NOVUS_DR2R_ROT2_CurLH 23u                                               // for deactivating digital output 2, in hundredths of seconds - word low
#endif  /* DR2R */

#if defined(NOVUS_DR4C_USED)
// %%%%%%%%%%% NOVUS DigiRail-4C Universal counter Module %%%%%%%%%%%%%%%
//READ HOLDING REGISTERS – 03H  PLC 40001 = 0 WRITE SINGLE REGISTER – 06H
#define NOVUS_DR4C_DIN_STAT 7u                                                  // State of the digital inputs, where bit 0 represents input 1, bit 1 represents input 2 (0=off; 1=on),  etc
#define NOVUS_DR4C_CNT1_TH 18u                                                  // Value of the input 1 counts - most significant word
#define NOVUS_DR4C_CNT1_LH 19u                                                  // Value of the input 1 counts - least significant word
#define NOVUS_DR4C_CNT2_TH 20u                                                  // Value of the input 2 counts - most significant word
#define NOVUS_DR4C_CNT2_LH 21u                                                  // Value of the input 2 counts - least significant word
#define NOVUS_DR4C_CNT3_TH 22u                                                  // Value of the input 3 counts - most significant word
#define NOVUS_DR4C_CNT3_LH 23u                                                  // Value of the input 3 counts - least significant word
#define NOVUS_DR4C_CNT4_TH 24u                                                  // Value of the input 4 counts - most significant word
#define NOVUS_DR4C_CNT4_LH 25u                                                  // Value of the input 4 counts - least significant word
#define NOVUS_DR4C_inCFG 16u                                                    // Configuration of the digital inputs Bit 0 in 1 - Input 1 operates as fast counter input. Debounce for input 1 is ignored
#define NOVUS_DR4C_pResetDir 17u                                                // set-up preset counter (e.g. distance tacho) and direction of pulse rising edge for counter as per D4C_Config_e
#define NOVUS_DR4C_DB1_TH 12u                                                   // Debounce of the digital input 1 (in ms) 0-10000
#define NOVUS_DR4C_DB2_TH 13u                                                   // Debounce of the digital input 2 (in ms) 0-10000
#define NOVUS_DR4C_DB3_TH 14u                                                   // Debounce of the digital input 3 (in ms) 0-10000
#define NOVUS_DR4C_DB4_TH 15u                                                   // Debounce of the digital input 4 (in ms) 0-10000
#define NOVUS_DR4C_PulsInt 59u                                                  // Pulse counting interval time (in 0.1 seconds) 0-36000 a value of 10 means a 1 second interval. When we configure a 1 second interval, the counting will be in Hertz
#define NOVUS_DR4C_PulsPInt 60u                                                 // Peak pulse counting interval time (in 0.1 seconds) 0-36000 Time interval on which input pulses are totalized and, in case they are bigger than the current value, they are stored on registers 71 to 78
#define NOVUS_DR4C_PulsRt1H 63u                                                 // Counted pulses in the last interval for input 1 - most significant word
#define NOVUS_DR4C_PulsRt1L 64u                                                 // Counted pulses in the last interval for input 1 - least significant word
#define NOVUS_DR4C_PulsRt2H 65u                                                 // Counted pulses in the last interval for input 2 - most significant word
#define NOVUS_DR4C_PulsRt2L 66u                                                 // Counted pulses in the last interval for input 2 - least significant word
#define NOVUS_DR4C_PulsRt3H 67u                                                 // Counted pulses in the last interval for input 3 - most significant word
#define NOVUS_DR4C_PulsRt3L 68u                                                 // Counted pulses in the last interval for input 3 - least significant word
#define NOVUS_DR4C_PulsRt4H 69u                                                 // Counted pulses in the last interval for input 4 - most significant word
#define NOVUS_DR4C_PulsRt4L 70u                                                 // Counted pulses in the last interval for input 4 - least significant word
#define NOVUS_DR4C_PulsPk1H 71u                                                 // Maximum of pulses counted in the last interval for input 1 - most significant word
#define NOVUS_DR4C_PulsPk1L 72u                                                 // Maximum of pulses counted in the last interval for input 1 - least significant word
#define NOVUS_DR4C_PulsPk2H 73u                                                 // Maximum of pulses counted in the last interval for input 2 - most significant word
#define NOVUS_DR4C_PulsPk2L 74u                                                 // Maximum of pulses counted in the last interval for input 2 - least significant word
#define NOVUS_DR4C_PulsPk3H 75u                                                 // Maximum of pulses counted in the last interval for input 3 - most significant word
#define NOVUS_DR4C_PulsPk3L 76u                                                 // Maximum of pulses counted in the last interval for input 3 - least significant word
#define NOVUS_DR4C_PulsPk4H 77u                                                 // Maximum of pulses counted in the last interval for input 4 - most significant word
#define NOVUS_DR4C_PulsPk4L 78u                                                 // Maximum of pulses counted in the last interval for input 4 - least significant word
// states of NOVUS_DR4C_pResetDir1
#define NOVUS_DR4C_enabPre1 (1u<<0u)                                            // enable preset no.1
#define NOVUS_DR4C_enabPre2 (1u<<1u)                                            // enable preset no.2
#define NOVUS_DR4C_enabPre3 (1u<<2u)                                            // enable preset no.3
#define NOVUS_DR4C_enabPre4 (1u<<3u)                                            // enable preset no.4
#define NOVUS_DR4C_inhib1 (1u<<4u)                                              // inhibit preset no.1
#define NOVUS_DR4C_inhib2 (1u<<5u)                                              // inhibit preset no.2
#define NOVUS_DR4C_inhib3 (1u<<6u)                                              // inhibit preset no.3
#define NOVUS_DR4C_inhib4 (1u<<7u)                                              // inhibit preset no.4
#define NOVUS_DR4C_edge1 (1u<<8u)                                               // rising edge = 0 falling edge = 1 to count i.e Bit in 1 - Count at the negative margin (1 for 0)
#define NOVUS_DR4C_edge2 (1u<<9u)                                               // rising edge = 0 falling edge = 1 to count
#define NOVUS_DR4C_edge3 (1u<<10u)                                              // rising edge = 0 falling edge = 1 to count
#define NOVUS_DR4C_edge4 (1u<<12u)                                              // rising edge = 0 falling edge = 1 to count

#define NOVUS_DR4C_Preset1H 26u                                                 // Preset value of the input 1 counts - most significant word
#define NOVUS_DR4C_Preset1L 27u                                                 // Preset value of the input 1 counts - least significant word
#define NOVUS_DR4C_Preset2H 28u                                                 // Preset value of the input 2 counts - most significant word
#define NOVUS_DR4C_Preset2L 29u                                                 // Preset value of the input 2 counts - least significant word
#define NOVUS_DR4C_Preset3H 30u                                                 // Preset value of the input 3 counts - most significant word
#define NOVUS_DR4C_Preset3L 31u                                                 // Preset value of the input 3 counts - least significant word
#define NOVUS_DR4C_Preset4H 32u                                                 // Preset value of the input 4 counts - most significant word
#define NOVUS_DR4C_Preset4L 33u                                                 // Preset value of the input 4 counts - least significant word

#endif /* novus DR4c */

#if defined(PROCON_TCP_16DI_USED)
// %%%%%%% Procon ELectronic Modbus TCP 16DI logic and counter up to 1KHz %%%%%
// remove 10 30 or 40 prefix once tested with the command

#define PROCON_TCP16DI_DEF_IP "169.254.111.11"                                  // Default IP Address

// READ HOLDING REGISTERS – 03H
// The specified addresses correspond to the low level physical addresses, 
// where zero (0) corresponds to the address of PLC 40001
#define PROCON_PT16DI_Mode 100u                                                 // 40101 0=Disable, 1=Up Counting, 2=Up/Down Count
#define PROCON_PT16DI_Filter 101u                                                 // 40102 Filter in 0.1s units
// MODBUS_FUNCTION_READ_DISCRETE_INPUTS
#define PROCON_PT16DI_DIStart 0u                                                // 10001 Status of Digital Inputs. (RO)
#define PROCON_PT16DI_DIEnd 15u                                                 // 10016 Status of Digital Inputs. (RO)
// READ INPUT REGISTERS - 04H
#define PROCON_PT16DI_DIWord 1u                                                 // 30002 Status of Digital Inputs. (RO)
// MODBUS_FUNCTION_READ_DISCRETE_INPUTS
#define PROCON_PT8DIO_DIStart 0u                                                // 10001 Status of Digital Inputs. (RO)
#define PROCON_PT8DIO_DIEnd 7u                                                  // 10008 Status of Digital Inputs. (RO)
// MODBUS_FUNCTION_WRITE_SINGLE_COIL   05H
// WRITE MULTIPLE COILS – 0FH
// READ COILS – 01H
// The specified addresses correspond to the low level physical addresses, where zero (0) corresponds to the address of PLC 00001
#define PROCON_PT8DIO_DOStart 17u                                               // Status of Digital Outputs. (RW)
#define PROCON_PT8DIO_DOEnd 24u                                                 // Status of Digital Outputs. (RW)
// MODBUS_FUNCTION_READ_INPUT_REGS
#define PROCON_PT8DIO_DIWord 1u                                                 // 30002 Status of Digital Inputs. (RO)
// MODBUS_FUNCTION_READ_HOLDING_REGS
#define PROCON_PT8DIO_DOWord 2u                                                 // 40003 Status of Digital Outputs. (RW)
// MODBUS_FUNCTION_READ_HOLDING_REGS
#define PROCON_PT4RO_DOWord 1u                                                  // 40002 4 Relay Ouputs (RW) X,X,X,X,X,X,X,X, X,X,X,X,4,3,2,1
// MODBUS_FUNCTION_READ_HOLDING_REGS
#define PROCON_PT8AVOI_AO1Start 1u                                              // 40002 either PT8AO or PT8VO volts or current
#define PROCON_PT8AVOI_AO8Stop 8u                                               // 40009
#endif /* 16DI TCP */

#if defined(PL101_USED)
// --------- Procon PL101 PLC Webserver or Logic Interface ---------------------

// func3or4 Reads a range of registers from RAM, EEPROM and BBRAM Range M0-M1219 max number 100
// 5 Writes a single Bit to any part of RAM M0-M1219 max number 1
// func6 Writes a single register to RAM, EEPROM and BBRAM Range M0-M1219 max number 1
// func15 Writes a range of bits to RAM. range M9-M999 max number 1600
// func16 Writes a range of registers to RAM, EEPROM and BBRAM. range M9-M1219 max number 100

// user RAM stored and not saved
// M161 – M199 start 40162 161 end 40200 199 
// shown in realtime in webpage where referenced as %M0000161 to %M0000199
// where %Mfwdxxxx 
// f= Format Field 0 Unsigned Single 1 Signed Single 2 Unsigned Double 3 Signed Double 4 Float
// w – Width Field  This field is used to specify the minimum number of characters to generate 
// for the conversion. A value of zero (0) will let it the function generate an unrestricted number of characters
// d – Decimal Places Field  This field is used to specify the number of fraction characters to generate for the conversion after the decimal point
#define PROCON_PL101_URAM1_START 161u
#define PROCON_PL101_URAM1_STOP 199u

// M401 – M999  start 40402 401 end 41000 999
#define PROCON_PL101_URAM2_START 401u
#define PROCON_PL101_URAM2_STOP 999u

// user EEPROM stored and not often
// M1000 – M1199 start 41001 1000 end 41170 1169
#define PROCON_PL101_EEPROM_START 1000u
#define PROCON_PL101_EEPROM_STOP 1169u

// user BBRAM stored saved and often
// M1200 – M1219 start 41208 1207 end 41220 1219
#define PROCON_PL101_BBRAM_START 1207u
#define PROCON_PL101_BBRAM_STOP 1219u

#if defined(D_FT900)
typedef struct MODPACKED {
   uint16_t Seconds;
   uint16_t Minutes;
   uint16_t Hours;
   uint16_t Day;
   uint16_t Date;
   uint16_t Month;
   uint16_t Year;
} Procon_PL101_rtc_t;                                                          // PL101 realtime clock                                                       
#else
MODPACKED(
typedef struct
{
   uint16_t Seconds;
   uint16_t Minutes;
   uint16_t Hours;
   uint16_t Day;
   uint16_t Date;
   uint16_t Month;
   uint16_t Year;
}) Procon_PL101_rtc_t;                                                          // PL101 realtime clock
#endif

// RTC read
#define PROCON_RTC_DATA_START 1200u
#define PROCON_RTC_DATA_END 1206u

#if defined(D_FT900)
typedef struct MODPACKED {
   uint16_t di1 : 1u;                                                           // digital input 1
   uint16_t di2 : 1u;                                                           // digital input
   uint16_t di3 : 1u;                                                           // digital input
   uint16_t di4 : 1u;                                                           // digital input
   uint16_t di5 : 1u;                                                           // digital input
   uint16_t di6 : 1u;                                                           // digital input
   uint16_t di7 : 1u;                                                           // digital input
   uint16_t di8 : 1u;                                                           // digital input
   uint16_t di9 : 1u;                                                           // digital input
   uint16_t di10 : 1u;                                                          // digital input
   uint16_t di11 : 1u;                                                          // digital input
   uint16_t di12 : 1u;                                                          // digital input
   uint16_t di13 : 1u;                                                          // digital input
   uint16_t di14 : 1u;                                                          // digital input
   uint16_t di15 : 1u;                                                          // digital input
   uint16_t di16 : 1u;                                                          // digital input
} Procon_PT16DI_t;                                                              // status of each input                                                       
#else
MODPACKED(
typedef struct
{
   uint16_t di1 : 1u;                                                           // digital input 1
   uint16_t di2 : 1u;                                                           // digital input
   uint16_t di3 : 1u;                                                           // digital input
   uint16_t di4 : 1u;                                                           // digital input
   uint16_t di5 : 1u;                                                           // digital input
   uint16_t di6 : 1u;                                                           // digital input
   uint16_t di7 : 1u;                                                           // digital input
   uint16_t di8 : 1u;                                                           // digital input
   uint16_t di9 : 1u;                                                           // digital input
   uint16_t di10 : 1u;                                                          // digital input
   uint16_t di11 : 1u;                                                          // digital input
   uint16_t di12 : 1u;                                                          // digital input
   uint16_t di13 : 1u;                                                          // digital input
   uint16_t di14 : 1u;                                                          // digital input
   uint16_t di15 : 1u;                                                          // digital input
   uint16_t di16 : 1u;                                                          // digital input
}) Procon_PT16DI_t;                                                             // status of each input
#endif

#define PROCON_PT16DI_Cnt1MSBStrt 2u                                            // 40003 Counter Register Start MSB+LSB = 32bit (RO)
#define PROCON_PT16DI_Cnt16LSBStop 33u                                          // 40034 Counter Register Stop MSB+LSB = 32bit (RO)
#if defined(D_FT900)
typedef struct MODPACKED {
   uint32_t cnt1;                                                               // counter channel
   uint32_t cnt2;                                                               // counter channel
   uint32_t cnt3;                                                               // counter channel
   uint32_t cnt4;                                                               // counter channel
   uint32_t cnt5;                                                               // counter channel
   uint32_t cnt6;                                                               // counter channel
   uint32_t cnt7;                                                               // counter channel
   uint32_t cnt8;                                                               // counter channel
   uint32_t cnt9;                                                               // counter channel
   uint32_t cnt10;                                                              // counter channel
   uint32_t cnt11;                                                              // counter channel
   uint32_t cnt12;                                                              // counter channel
   uint32_t cnt13;                                                              // counter channel
   uint32_t cnt14;                                                              // counter channel
   uint32_t cnt15;                                                              // counter channel
   uint32_t cnt16;                                                              // counter channel
} Procon_PT16DI_cnt_t;                                                          // high speed counters for each input                                                       
#else
MODPACKED(
typedef struct
{
   uint32_t cnt1;                                                               // counter channel
   uint32_t cnt2;                                                               // counter channel
   uint32_t cnt3;                                                               // counter channel
   uint32_t cnt4;                                                               // counter channel
   uint32_t cnt5;                                                               // counter channel
   uint32_t cnt6;                                                               // counter channel
   uint32_t cnt7;                                                               // counter channel
   uint32_t cnt8;                                                               // counter channel
   uint32_t cnt9;                                                               // counter channel
   uint32_t cnt10;                                                              // counter channel
   uint32_t cnt11;                                                              // counter channel
   uint32_t cnt12;                                                              // counter channel
   uint32_t cnt13;                                                              // counter channel
   uint32_t cnt14;                                                              // counter channel
   uint32_t cnt15;                                                              // counter channel
   uint32_t cnt16;                                                              // counter channel
}) Procon_PT16DI_cnt_t;                                                         // high speed counters for each input
#endif

#define PROCON_PT16DI_CntCapDef 34u                                             // 40035 set each bit to capture the counter captures below (RW)
#define PROCON_PT16DI_CCnt1MSBStrt 35u                                          // 40036 Counter Capture Register Start MSB+LSB = 32bit (RO)
#define PROCON_PT16DI_CCnt16LSBStop 66u                                         // 40067 Counter Capture Register Stop MSB+LSB = 32bit (RO)

#endif /* end procon PL101 */

// These functions can be used as alternative to structs by sending a string also useful for modbus ascii

/*******************************************************************************
* Function Name: modbus_rtu_build_request
********************************************************************************
* Summary:
*  modbus_rtu_build_request : function performs following functions:
*   1. Builds a RTU master request header
*
* Parameters:
*  uint8_t slave, uint8_t function, uint16_t addr, uint16_t nb,  uint8_t *req
*
* Return:
*  int8_t 1 : success
*
*******************************************************************************/
static int8_t modbus_rtu_build_request(uint8_t slave, uint8_t function, uint16_t addr, uint16_t nb,  uint8_t *req)
{
    if ((((slave==NULL) || (function==NULL)) || (addr==NULL)) || (nb==NULL))
       return 0U;                                                               // return error
    req[0U] = slave;                                                            // create the message
    req[1U] = function;
    req[2U] = addr >> 8U;
    req[3U] = addr & 0x00ffU;
    req[4U] = nb >> 8U;
    req[5U] = nb & 0x00ffU;
    return 1U;                                                                   // return success
}

/*******************************************************************************
* Function Name: modbus_rtu_build_response
********************************************************************************
* Summary:
*  modbus_rtu_build_response : function performs following functions:
*   1. Builds a RTU slave response header
*
* Parameters:
*  uint8_t slave, uint8_t function, uint8_t *rsp
*
* Return:
*  int8_t 1 : success
*
*******************************************************************************/
static int8_t modbus_rtu_build_response(uint8_t slave, uint8_t function, uint8_t *rsp)
{
    if ((slave==NULL) || (function==NULL))
       return 0U;
    rsp[0] = slave;
    rsp[1] = function;
    return 1U;
}

/*******************************************************************************
* Function Name: modbus_rtu_tag_crc
********************************************************************************
* Summary:
*  modbus_rtu_tag_crc : function performs following functions:
*   1. tag crc for RTU
*
* Parameters:
*  uint8_t *req, uint16_t req_length
*
* Return:
*  void
*
*******************************************************************************/
static void modbus_rtu_tag_crc(uint8_t *req, uint16_t req_length)
{
    uint16_t crc;
    crc=usMBCRC16((unsigned char *) req, req_length);                       // included in crc.h
    req[req_length++] = crc >> 8U;
    req[req_length++] = crc & 0x00FFU;
}

/*******************************************************************************
* Function Name: modbus_ascii_tag_crc
********************************************************************************
* Summary:
*  modbus_ascii_tag_crc : function performs following functions:
*   1. tag crc for ASCII
*
* Parameters:
*  uint8_t *req, uint16_t req_length
*
* Return:
*  void
*
*******************************************************************************/
static void modbus_ascii_tag_crc(uint8_t *req, uint16_t req_length)
{
    uint8_t crc;
    crc=crc=usMBAsciiLRC((unsigned char *) req, req_length);                    // included in crc.h
    req[req_length++] = crc & 0x00FFU;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
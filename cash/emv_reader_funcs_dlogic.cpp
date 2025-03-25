// -------------------------------------                       d-logic μFR Nano EMV Credit Card Reader                        -----------------------------------------------
//
// This is a library for use with EMV card reader by d-logic the manufacturer provides a program with a GUI made in wx and can also be used for testing
//
// I have ported the api functions for use without the HMI (wx libraries) by vending systems etc as below (untested)
//
// Wx Reader and libraries can be downloaded here https://www.d-logic.com/ja/knowledge_base/emv-%E3%82%AF%E3%83%AC%E3%82%B8%E3%83%83%E3%83%88%E3%82%AB%E3%83%BC%E3%83%89-ufr-%E3%83%8A%E3%83%8E-%E3%82%AA%E3%83%B3%E3%83%A9%E3%82%A4%E3%83%B3/
// Author: Aleksandar Krstic (aleksandar.krstic@d-logic.rs)
//
// -------------------------------------                       d-logic μFR Nano EMV Credit Card Reader                        -----------------------------------------------
#include "apdu_credit_card_exampleMain.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include <string.h>
#if _WIN32 || _WIN64
#include <conio.h>
#include <windows.h>
#elif linux || linux || APPLE
#define __USE_MISC 1
#include <unistd.h>
#include <termios.h>
#undef __USE_MISC
#include "conio_gnu.h"
#else
#error "Unknown build platform."
#endif
#include "ufr-lib/include/uFCoder.h"                                                          // get these from the manufacturer url as above
#include "ini.h"
#include "iso3166.h"
#include "iso4217.h"
#include "emv.h"
#include "uFR.h"
#include "utils.h"
#include <iostream>
#include <sstream>
#include <iomanip>
// command line parser library
#include "cxxopts.hpp"

#define RAW_RES_MAX_LEN 258
#define MAX_AID_LEN 16
//(*InternalHeaders(apdu_credit_card_exampleFrame)
//*)
int checkEmvPse(const char *df_name, const char *szTitlePse);
int tryEmvPseCardRead(const char *df_name, const char *szTitlePse);
void dec_to_hex(uint32_t dec, char HEXA[1000]);
uint32_t hex2int(char *hex);
int open_connection(uint32_t reader_type, uint32_t port_interface, c_string port_name, c_string additional, bool checkAdvanced)
void reset_connection();
int close_conenction();
int GetLastTranasction(int pse);
int GetPan(int pse, uint64_t *pan_rcv);
int ReadTheCard(int pse);
int CheckEmv(int pse);

uint32_t sw_int[2];
char sw_string1[4];
char sw_string2[4];
char EMV_STATUS_error[255] = "";
uint32_t cnt_integer, record_integer, sfi_integer, sfi_hex32;
char SFI_HEX[255];
uint32_t response_integer[RAW_RES_MAX_LEN];
char RESPONSE[RAW_RES_MAX_LEN] = "";

//  hexadecimal to decimal
uint32_t hex2int(char *hex)
{
    uint32_t val = 0;

    while (*hex)
    {
        uint8_t byte = *hex++;

        if (byte >= '0' && byte <= '9')
            byte = byte - '0';
        else if (byte >= 'a' && byte <= 'f')
            byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <= 'F')
            byte = byte - 'A' + 10;

        val = (val << 4) | (byte & 0xF);
    }

    return val;
}

// decimal to hexadecimal
void dec_to_hex(uint32_t dec, char HEXA[1000])
{

    char hex[1000] = "";
    char hex1[1000] = "";

    switch (dec / 16)
    {

    case 0:
        strcpy(hex, "0");
        break;
    case 1:
        strcpy(hex, "1");
        break;
    case 2:
        strcpy(hex, "2");
        break;
    case 3:
        strcpy(hex, "3");
        break;
    case 4:
        strcpy(hex, "4");
        break;
    case 5:
        strcpy(hex, "5");
        break;
    case 6:
        strcpy(hex, "6");
        break;
    case 7:
        strcpy(hex, "7");
        break;
    case 8:
        strcpy(hex, "8");
        break;
    case 9:
        strcpy(hex, "9");
        break;
    case 10:
        strcpy(hex, "A");
        break;
    case 11:
        strcpy(hex, "B");
        break;
    case 12:
        strcpy(hex, "C");
        break;
    case 13:
        strcpy(hex, "D");
        break;
    case 14:
        strcpy(hex, "E");
        break;
    case 15:
        strcpy(hex, "F");
        break;
    }

    switch (dec % 16)
    {

    case 0:
        strcpy(hex1, "0");
        break;
    case 1:
        strcpy(hex1, "1");
        break;
    case 2:
        strcpy(hex1, "2");
        break;
    case 3:
        strcpy(hex1, "3");
        break;
    case 4:
        strcpy(hex1, "4");
        break;
    case 5:
        strcpy(hex1, "5");
        break;
    case 6:
        strcpy(hex1, "6");
        break;
    case 7:
        strcpy(hex1, "7");
        break;
    case 8:
        strcpy(hex1, "8");
        break;
    case 9:
        strcpy(hex1, "9");
        break;
    case 10:
        strcpy(hex1, "A");
        break;
    case 11:
        strcpy(hex1, "B");
        break;
    case 12:
        strcpy(hex1, "C");
        break;
    case 13:
        strcpy(hex1, "D");
        break;
    case 14:
        strcpy(hex1, "E");
        break;
    case 15:
        strcpy(hex1, "F");
        break;
    }

    strcat(HEXA, hex);
    strcat(HEXA, hex1);
}

// open connection
int open_connection(uint32_t reader_type, uint32_t port_interface, c_string port_name, c_string additional, bool checkAdvanced)
{
    UFR_STATUS status;                               
    int ret = 0;
	
    if (checkAdvanced == true)
    {
        status = ReaderOpenEx(reader_type, port_name, port_interface, (void*)additional);
    }
    else
    {
        status = ReaderOpen();
    }

    if (status == UFR_OK)
    {
        ReaderUISignal(1, 1);
        std::cout << "Port opened" << std::endl;
		ret = 1;
    }
    else if (status != UFR_OK)
    {
        std::cout << "Port not opened!" << std::endl;
    }
    return ret;
}

// reset connection
void reset_connection()
{

    UFR_STATUS status;
    status = ReaderReset();

    if (status == UFR_OK)
    {
        ReaderUISignal(1, 1);
        std::cout <<  "Reader reset" << std::endl;
    }
    else if (status != UFR_OK)
    {
        std::cout <<  "Reader not reset" << std::endl;
    }
}

// close connection
int close_conenction()
{

    UFR_STATUS status;
    status = ReaderClose();
    int ret = 0;
    if (status == UFR_OK)
    {
        std::cout << "Reader closed" << std::endl;
	ret = 1;
    }
    else if (status != UFR_OK)
    {
        std::cout << "Reader not closed" << std::endl;
    }
    return ret;
}

// read card
int tryEmvPseCardRead(const char *df_name, const char *szTitlePse)
{

    //std:cerr<<"1";

    UFR_STATUS emv_status;
    UFR_STATUS status;
    bool head_attached = false;
    emv_tree_node_t *head = NULL, *tail = NULL, *temp = NULL;
    afl_list_item_t *afl_list = NULL, *afl_list_item = NULL;
    uint8_t afl_list_count;
    uint8_t r_apdu[RAW_RES_MAX_LEN];
    uint32_t Ne; // without SW
    uint8_t sw[2], aid[MAX_AID_LEN];
    uint8_t sfi, record, cnt = 1, aid_len;
    uint16_t *sw16_ptr = (uint16_t *)&sw;
    uint8_t *gpo_data_field = NULL;
    uint16_t gpo_data_field_size;
    char empty_string[RAW_RES_MAX_LEN] = "";

    cnt = 1;
    std::cout << "=====================================================================\n";
    std::cout << "  Read and parse EMV card supporting " << szTitlePse << "\n";
    std::cout << " --------------------------------------------------------------------------------------------------------------------\n";

    do
    {

        status = SetISO14443_4_Mode();

        std::cout << "THIS IS SIMPLY A DEBUG LINE HERE" << std::endl;

        Status = UFR_Status2String(status);

        if (status != UFR_OK)
        {
            std::cout << " Error while switching into ISO 14443-4 mode, uFR status is: " << Status << "\n";
            return 0;
            //break;
        }

        //std:cerr<<"2";

        std::cout << (int)cnt++ << ". Issuing Select PSE command: " << df_name << "\n";
        std::cout << "  [C] 00 A4 04 00 \n";

        Ne = 256;
        status = APDUTransceive(0x00, 0xA4, 0x04, 0x00, (uint8_t *)df_name, strlen(df_name), r_apdu, &Ne, 1, sw);
        Status = UFR_Status2String(status);

        if (status != UFR_OK)
        {
            std::cout << " Error while executing APDU command, uFR status is: " << Status;

            return 0;
            //break;
        }
        else
        {

            if (*sw16_ptr != 0x90)
            {
                sw_int[0] = sw[0];
                sw_int[1] = sw[1];

                sprintf(sw_string1, "%X", sw_int[0]);
                sprintf(sw_string2, "%X", sw_int[1]);

                std::cout << "\nCould not continue execution due to an APDU error" << "\n[SW] : " << sw_string1 << " " << sw_string2;
            }

            if (Ne)
            {
                std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                strcpy(RESPONSE, empty_string);

                for (uint32_t i = 0; i < Ne; i++)
                {
                    response_integer[i] = r_apdu[i];
                }

                for (uint32_t j = 0; j < Ne; j++)
                {

                    dec_to_hex(response_integer[j], RESPONSE);
                }
                std::cout << "\n[R] : " << RESPONSE;
            }

            sw_int[0] = sw[0];
            sw_int[1] = sw[1];

            sprintf(sw_string1, "%02X", sw_int[0]);
            sprintf(sw_string2, "%02X", sw_int[1]);

            std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;

            //std:cerr<<"3";
        }

        emv_status = newEmvTag(&head, r_apdu, Ne, false);

        sprintf(EMV_STATUS_error, "%08X", emv_status);

        if (emv_status)
        {
            std::cout << "Card does not support Payment System Enviroment " << std::endl;
            return 0;
            //break;
        }

        emv_status = getSfi(head, &sfi);

        if (emv_status == UFR_OK)
        {

            cnt = 2;
            record = 1;

            cnt_integer = cnt;
            record_integer = record;
            sfi_integer = sfi;
            sfi_hex32 = (sfi << 3) | 4;

            sprintf(SFI_HEX, "%02X", sfi_hex32);

            do
            {
                std::cout << cnt_integer << ". Issuing Read Record command (record = " << record_integer << ", sfi = " << sfi_integer << " )";
                std::cout << "\n[C] 00 B2 " << SFI_HEX;

                emv_status = emvReadRecord(r_apdu, &Ne, sfi, record, sw);

                if (emv_status == UFR_OK)
                {

                    emv_status = newEmvTag(&temp, r_apdu, Ne, false);

                    if (record == 1)
                        head->next = tail = temp;
                    else
                    {
                        tail->next = temp;
                        tail = tail->next;
                    }

                    if (Ne)
                    {

                        std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                        strcpy(RESPONSE, empty_string);

                        for (uint32_t i = 0; i < Ne; i++)
                        {
                            response_integer[i] = r_apdu[i];
                        }

                        for (uint32_t j = 0; j < Ne; j++)
                        {
                            dec_to_hex(response_integer[j], RESPONSE);
                        }

                        std::cout << "\n[R] : " << RESPONSE;
                    }

                    sw_int[0] = sw[0];
                    sw_int[1] = sw[1];

                    sprintf(sw_string1, "%02X", sw_int[0]);
                    sprintf(sw_string2, "%02X", sw_int[1]);

                    std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;

                    //std:cerr<<"4";
                }
                else
                {

                    if (*sw16_ptr != 0x90)
                    {

                        sw_int[0] = sw[0];
                        sw_int[1] = sw[1];

                        sprintf(sw_string1, "%X", sw_int[0]);
                        sprintf(sw_string2, "%X", sw_int[1]);

                        std::cout << "\nThere is no records.\n"  << "\n[SW] : " << sw_string1 << " " << sw_string2;
                    }
                }

                record++;
                cnt++;

                //std:cerr<<"5";

            }
            while (emv_status == UFR_OK);
        }

        emv_status = getAid(head, aid, &aid_len);

        if (emv_status == UFR_OK)
        {
            std::cout << " \n--------------------------------------------------------------------------------------------------------------------\n";
            std::cout << (int)cnt++ << ". Issuing Select the application command:\n";
            std::cout << "  [C] 00 A4 04 00 ";

            Ne = 256;
            status = APDUTransceive(0x00, 0xA4, 0x04, 0x00, aid, aid_len, r_apdu, &Ne, 1, sw);
            Status = UFR_Status2String(status);

            //std:cerr<<"6";

            if (status != UFR_OK)
            {
                std::cout << " Error while executing APDU command, uFR status is: " << Status;

                return 0;
               // break;
            }
            else
            {

                if (*sw16_ptr != 0x90)
                {
                    sw_int[0] = sw[0];
                    sw_int[1] = sw[1];

                    sprintf(sw_string1, "%X", sw_int[0]);
                    sprintf(sw_string2, "%X", sw_int[1]);

                    std::cout << "\nCould not continue execution due to an APDU error" << "\n[SW] : " << sw_string1 << " " << sw_string2;
                }

                //std:cerr<<"7";

                if (Ne)
                {
                    std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                    strcpy(RESPONSE, empty_string);

                    for (uint32_t i = 0; i < Ne; i++)
                    {

                        response_integer[i] = r_apdu[i];
                    }

                    for (uint32_t j = 0; j < Ne; j++)
                    {

                        dec_to_hex(response_integer[j], RESPONSE);
                    }

                    std::cout << "\n[R] : " << RESPONSE;
                }

                sw_int[0] = sw[0];
                sw_int[1] = sw[1];

                sprintf(sw_string1, "%02X", sw_int[0]);
                sprintf(sw_string2, "%02X", sw_int[1]);

                std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;

                //std:cerr<<"8";
            }
        }

        emv_status = newEmvTag(&temp, r_apdu, Ne, false);

        sprintf(EMV_STATUS_error, "%08X", emv_status);

        if (emv_status)
        {

            std::cout << " EMV parsing error code: " << EMV_STATUS_error;
            std::cout << "Card does not support Payment System Enviroment " << std::endl;
        }

        if (!head_attached)
        {
            head->next = tail = temp;
            head_attached = true;
        }
        else
        {
            tail->next = temp;
            tail = tail->next;
        }

        std::cout << " \n--------------------------------------------------------------------------------------------------------------------\n";
        std::cout << (int)cnt++ << ". Formating Get Processing Options instruction (checking PDOL).\n";
        std::cout << " --------------------------------------------------------------------------------------------------------------------\n";

        emv_status = formatGetProcessingOptionsDataField(temp, &gpo_data_field, &gpo_data_field_size);

        sprintf(EMV_STATUS_error, "%08X", emv_status);

        //std:cerr<<"9";

        if (emv_status)
        {
            std::cout << " EMV parsing error code: " << EMV_STATUS_error;
        }

        std::cout << (int)cnt++ << ". Issuing Get Processing Options command:\n";
        std::cout << "  [C] 80 A8 00 00\n";

        Ne = 256;
        status = APDUTransceive(0x80, 0xA8, 0x00, 0x00, gpo_data_field, gpo_data_field_size, r_apdu, &Ne, 1, sw);

        //std::cerr<<"10";

        if (status != UFR_OK)
        {
            std::cout << " Error while executing APDU command, uFR status is: " << Status;
            return 0;
            //break;
        }
        else
        {

            if (*sw16_ptr != 0x90)
            {

                sw_int[0] = sw[0];
                sw_int[1] = sw[1];

                sprintf(sw_string1, "%X", sw_int[0]);
                sprintf(sw_string2, "%X", sw_int[1]);

                std::cout << "\nCould not continue execution due to an APDU error"
                       << "\n[SW] : " << sw_string1 << " " << sw_string2;

                //std:cerr<<"11";
            }

            if (Ne)
            {
                std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                strcpy(RESPONSE, empty_string);

                for (uint32_t i = 0; i < Ne; i++)
                {
                    response_integer[i] = r_apdu[i];
                }

                for (uint32_t j = 0; j < Ne; j++)
                {
                    dec_to_hex(response_integer[j], RESPONSE);
                }

                std::cout << "\n[R] : " << RESPONSE;
            }

            sw_int[0] = sw[0];
            sw_int[1] = sw[1];

            sprintf(sw_string1, "%02X", sw_int[0]);
            sprintf(sw_string2, "%02X", sw_int[1]);

            std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;
        }
        emv_status = newEmvTag(&temp, r_apdu, Ne, false);

        sprintf(EMV_STATUS_error, "%08X", emv_status);

        if (emv_status)
        {

            std::cout << " EMV parsing error code: " << EMV_STATUS_error;
        }

        tail->next = temp;
        tail = tail->next;

        std::cout << " \n--------------------------------------------------------------------------------------------------------------------\n";
        std::cout << " GET AFL :\n";

        emv_status = getAfl(temp, &afl_list, &afl_list_count);

        if (emv_status == EMV_ERR_TAG_NOT_FOUND)

            emv_status = getAflFromResponseMessageTemplateFormat1(temp, &afl_list, &afl_list_count);

        sprintf(EMV_STATUS_error, "%08X", emv_status);

        if (emv_status)
        {

            std::cout << " EMV parsing error code: " << EMV_STATUS_error;
            return 0;
            //break;
        }

        afl_list_item = afl_list;

        while (afl_list_item)
        {

            for (int r = afl_list_item->record_first; r <= afl_list_item->record_last; r++)
            {

                sfi_integer = afl_list_item->sfi;

                std::cout << " \n--------------------------------------------------------------------------------------------------------------------\n";
                std::cout << (int)cnt << ". Issuing Read Record command (record = " << r << ", sfi = " << sfi_integer << ")\n";
                std::cout << "  [C] 00 B2 \n";

                emv_status = emvReadRecord(r_apdu, &Ne, afl_list_item->sfi, r, sw);

                if (emv_status == UFR_OK)
                {

                    emv_status = newEmvTag(&temp, r_apdu, Ne, false);

                    if (emv_status == UFR_OK)
                    {

                        tail->next = temp;
                        tail = tail->next;
                    }

                    if (Ne)
                    {

                        std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                        strcpy(RESPONSE, empty_string);

                        for (uint32_t i = 0; i < Ne; i++)
                        {

                            response_integer[i] = r_apdu[i];
                        }

                        for (uint32_t j = 0; j < Ne; j++)
                        {

                            dec_to_hex(response_integer[j], RESPONSE);
                        }

                        std::cout << "\n[R] : " << RESPONSE;

                        if (r == 1 && afl_list_item->sfi == 2)
                        {
                            std::cout << RESPONSE[10] << RESPONSE[11] << RESPONSE[12] << RESPONSE[13];
                            std::cout << RESPONSE[14] << RESPONSE[15] << RESPONSE[16] << RESPONSE[17];
                            std::cout << RESPONSE[18] << RESPONSE[19] << RESPONSE[20] << RESPONSE[21];
                            std::cout << RESPONSE[22] << RESPONSE[23] << RESPONSE[24] << RESPONSE[25];
                        }
                    }

                    sw_int[0] = sw[0];
                    sw_int[1] = sw[1];

                    sprintf(sw_string1, "%02X", sw_int[0]);
                    sprintf(sw_string2, "%02X", sw_int[1]);

                    std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;

                    //std:cerr<<"14";
                }
                else
                {
                    if (*sw16_ptr != 0x90)
                    {

                        sw_int[0] = sw[0];
                        sw_int[1] = sw[1];

                        sprintf(sw_string1, "%X", sw_int[0]);
                        sprintf(sw_string2, "%X", sw_int[1]);

                        std::cout << "\nCould not continue execution due to an APDU error"
                               << "\n[SW] : " << sw_string1 << " " << sw_string2;
                    }
                }

                cnt++;
            }

            //std:cerr<<"15";

            afl_list_item = afl_list_item->next;
        }
    }
    while (0);

    std::cout << "\n--------------------------------------------------------------------------------------------------------------------\n";

    if (afl_list)
        emvAflListCleanup(afl_list);
    if (gpo_data_field)
        free(gpo_data_field);
    if (head)
        emvTreeCleanup(head);

    std::cout << "=====================================================================\n";

    //std:cerr<<"16";

    return 1;
}

// check emv
int checkEmvPse(const char *df_name, const char *szTitlePse)
{

    emv_tree_node_t *head = NULL, *tail = NULL, *temp = NULL;
    uint8_t r_apdu[RAW_RES_MAX_LEN];
    uint32_t Ne; // without SW
    uint8_t sw[2];
    uint8_t sfi, record, cnt;
    uint16_t *sw16_ptr = (uint16_t *)&sw;
    UFR_STATUS emv_status;
    UFR_STATUS status;
    std::cout = "";

    std::cout << "=====================================================================\n";
    std::cout << "  Checking if card support Payment System Environment " << szTitlePse << "\n";
    std::cout << " --------------------------------------------------------------------------------------------------------------------\n";

    do
    {
        status = SetISO14443_4_Mode();

        Status = UFR_Status2String(status);

        if (status != UFR_OK)
        {
            std::cout << "Error while switching into ISO 14443-4 mode, uFR status is: " << Status;
	    return 0;
        }
        else
        {

            std::cout << "1. Issuing Select PSE command " << df_name;
            std::cout << "\n[C] 00 A4 04 00 32 50 41 59 2E 53 59 53 2E 44 44 46 30 31 00 ";

            Ne = 256;

            status = APDUTransceive(0x00, 0xA4, 0x04, 0x00, (const uint8_t *)df_name, strlen(df_name), r_apdu, &Ne, 1, sw);

            Status = UFR_Status2String(status);

            if (status != UFR_OK)
            {

                std::cout << " Error while executing APDU command, uFR status is: " << Status;

                return 0;
                //break;
            }
            else
            {

                if (*sw16_ptr != 0x90)
                {

                    sw_int[0] = sw[0];
                    sw_int[1] = sw[1];

                    sprintf(sw_string1, "%X", sw_int[0]);
                    sprintf(sw_string2, "%X", sw_int[1]);

                    std::cout << "\nCould not continue execution due to an APDU error"
                              << "\n[SW] : " << sw_string1 << " " << sw_string2;
                }

                std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                uint32_t response_integer[RAW_RES_MAX_LEN];

                for (uint32_t i = 0; i < Ne; i++)
                {

                    response_integer[i] = r_apdu[i];
                }

                char RESPONSE[RAW_RES_MAX_LEN] = "";

                for (uint32_t j = 0; j < Ne; j++)
                {

                    dec_to_hex(response_integer[j], RESPONSE);
                }

                sw_int[0] = sw[0];
                sw_int[1] = sw[1];

                sprintf(sw_string1, "%02X", sw_int[0]);
                sprintf(sw_string2, "%02X", sw_int[1]);

                std::cout << "\n[R] : " << RESPONSE << "\n [SW] : " << sw_string1 << " " << sw_string2;
            }

            emv_status = newEmvTag(&head, r_apdu, Ne, false);

            sprintf(EMV_STATUS_error, "%08X", emv_status);

            if (emv_status)
            {

                std::cout << "\nEMV parsing error code: " << EMV_STATUS_error;
                std::cout << "\n--------------------------------------------------------------------------------------------------------------------\n";
                std::cout << "Card does not support Payment System Environment " << std::cout;
                std::cout << "\n=====================================================================\n";
                return 0;
                //break;
            }

            emv_status = getSfi(head, &sfi);

            if (emv_status == UFR_OK)
            {

                cnt = 2;
                record = 1;

                cnt_integer = cnt;
                record_integer = record;
                sfi_integer = sfi;
                sfi_hex32 = (sfi << 3) | 4;

                sprintf(SFI_HEX, "%02X", sfi_hex32);

                do
                {

                    std::cout << cnt_integer << ". Issuing Read Record command (record = " << record_integer << ", sfi = " << sfi_integer << " )";
                    std::cout << "\n[C] 00 B2 " << SFI_HEX;

                    emv_status = emvReadRecord(r_apdu, &Ne, sfi, record, sw);

                    if (emv_status == UFR_OK)
                    {

                        emv_status = newEmvTag(&temp, r_apdu, Ne, false);

                        if (record == 1)
                            head->next = tail = temp;
                        else
                        {
                            tail->next = temp;
                            tail = tail->next;
                        }

                        if (Ne)
                        {

                            std::cout << "\nAPDU command executed: response data length = " << (int)Ne;

                            for (uint32_t i = 0; i < Ne; i++)
                            {

                                response_integer[i] = r_apdu[i];
                            }

                            char RESPONSE[RAW_RES_MAX_LEN] = "";

                            for (uint32_t j = 0; j < Ne; j++)
                            {

                                dec_to_hex(response_integer[j], RESPONSE);
                            }

                            std::cout << "\n[R] : " << RESPONSE;
                        }

                        sw_int[0] = sw[0];
                        sw_int[1] = sw[1];

                        sprintf(sw_string1, "%02X", sw_int[0]);
                        sprintf(sw_string2, "%02X", sw_int[1]);

                        std::cout << "\n [SW] : " << sw_string1 << " " << sw_string2;
                    }
                    else
                    {

                        if (*sw16_ptr != 0x90)
                        {

                            sw_int[0] = sw[0];
                            sw_int[1] = sw[1];

                            sprintf(sw_string1, "%X", sw_int[0]);
                            sprintf(sw_string2, "%X", sw_int[1]);

                            std::cout << "\nThere is no records.\n" << "\n[SW] : " << sw_string1 << " " << sw_string2;
                        }
                    }

                    record++;
                    cnt++;

                }
                while (emv_status == UFR_OK);
            }
            std::cout << "\n--------------------------------------------------------------------------------------------------------------------\n";
            std::cout << "Card support Payment System Environment " << szTitlePse;
            std::cout << "\n=====================================================================\n";
        }

    }
    while (0);

    emvTreeCleanup(head);
    return 1;
}

// check Emv
int CheckEmv(int pse)
{
    int ret = 0;
    if (pse == 1)
    {
        ret = checkEmvPse("1PAY.SYS.DDF01", "PSE1");
    }
    else if (pse == 2)
    {
        ret = checkEmvPse("2PAY.SYS.DDF01", "PSE2");
    }
    else
    {
       std::cout << "read error \n";
    }
    return ret;
}

// read the card
//
int ReadTheCard(int pse)
{
    int ret = 1;
    if (pse == 1)
    {
        if(tryEmvPseCardRead("1PAY.SYS.DDF01", "PSE1")==0)
        {
            std::cout << "read error \n";
	    ret = 0;
        }
    }
    else if (pse == 2)
    {
        if(tryEmvPseCardRead("2PAY.SYS.DDF01", "PSE2")==0)
        {
            std::cout << "read error \n";
	    ret = 0;
        }
    }
    else
    {
       std::cout << "read error \n";
       ret = 0;
    }
    return ret;
}

// get pan
int GetPan(int pse, uint64_t *pan_rcv)
{
    UFR_STATUS status;
	
    char pan_str[128];
    c_string df_name = "";

    if (pse == 1)
    {
        df_name = "1PAY.SYS.DDF01";
    } else if (pse == 2)
    {
        df_name = "2PAY.SYS.DDF01";
    }
    else
    {
       std::cout << "read error \n";
       return 0;
    }
	
    status = SetISO14443_4_Mode();
    if (status == UFR_OK)
    {
        status = EMV_GetPAN(df_name, pan_str);
        if (status == UFR_OK)
        {
            std::string pan = std::string(pan_str);
            std::cout << "pan " << pan << "\n";
	    *pan_rcv = std::stoull(pan);
        } else
        {
            std::cout << "read error \n";
	    return 0;
        }

    } else
    {
        std::cout << "Error while switching into ISO 14443-3 mode: " << UFR_Status2String(status);
	return 0;
    }
    return 1;
}

// get last transaction
int GetLastTranasction(int pse)
{
    UFR_STATUS status;
    int ret = 1;
	
    char last_transaction_info[256];
    c_string df_name = "";

    if (pse == 1)
    {
        df_name = "1PAY.SYS.DDF01";
    } else if (pse == 2)
    {
        df_name = "2PAY.SYS.DDF01";
    }
    else
    {
       std::cout << "read error \n";
	   return 0;
    }
	
    status = SetISO14443_4_Mode();
    if (status == UFR_OK)
    {
        status = EMV_GetLastTransaction(df_name, last_transaction_info);
        if (status == UFR_OK)
        {
            std::string transaction_info = std::string(last_transaction_info);

            std::string arr[12];
            int i = 0;
            std::stringstream ssin(transaction_info);
            while (ssin.good()){
                ssin >> arr[i];
                ++i;
            }
            std::cout << arr[1] << std::endl;
            std::cout << arr[3] << std::endl;
            std::cout << arr[5] << std::endl;
            std::cout << arr[7] << std::endl;
            std::cout << arr[9] << std::endl;

        } else
        {
            std::cout << "EMV_GetLastTransaction() error occurred: " << UFR_Status2String(status) << std::endl;
	    ret = 0;
        }

    } else
    {
        std::cout << "Error while switching into ISO 14443-3 mode: " << UFR_Status2String(status) << std::endl;
	ret = 0;
    }
    return ret;
}


int main(int argc, char** argv)
{
    uint32_t reader_type = 0;
    uint32_t port_interface = 0;
    c_string port_name = " ";
    c_string additional = " ";
    bool checkAdvanced = false;                                                  // open default connection	
    int ret = 1;
    int pse = 1;                                                                 // default PSE1		
    cxxopts::Options options("emv_reader");
	
     try {
		
		options.add_options()
			("port_name", "Port name", cxxopts::value<std::string>())
			("reader_type", "Reader type", cxxopts::value<uint32_t>())
			("port_interface", "Port Interface 84/85", cxxopts::value<uint32_t>())
			("pse", "Select Payment System Enviroment", cxxopts::value<int>(pse))
			("additional", "additional", cxxopts::value<std::string>())
			("checkAdvanced", "use those parameters", cxxopts::value<bool>(checkAdvanced))
			("d,debug", "Enable debugging")
			("h,help", "pass the following commandline args pse(i)1/2 port_name(s) reader_type(i) port_interface(i)84/5 additional(s) checkAdvanced(b)true use these")
			;
		options.parse_positional({ "port_name", "reader_type", "port_interface", "additional", "checkAdvanced"});

		auto result = options.parse(argc, argv);

		if (result.count("help"))
		{
		    std::cout << options.help({}) << std::endl;
		    return 0;
		}
		std::stringstream ss;                                                               // i think we need this ? if not use std::string for them 
                if (result.count("port_name")) {
		    std::string port_names = result["port_name"].as<std::string>();
		    ss << port_names;
		    ss >> port_name;
		}
                if (result.count("additional")) {
	           std::string additionals = result["additional"].as<std::string>();                                                
	           ss << additionals;
	           ss >> additional;
                }
                if (result.count("reader_type")) {		
	           reader_type = result["reader_type"].as<uint32_t>();
                }
                if (result.count("port_interface")) {	
	            port_interface = result["port_interface"].as<uint32_t>();
                }
	   }
    catch (cxxopts::OptionException &e) {
		std::cout << options.usage() << std::endl;
		std::cerr << e.what() << std::endl;
		std::cerr << "defaulting to use standard connection" << std::endl;
    }
	
    if (open_connection(reader_type, port_interface, port_name, additional, checkAdvanced) == 1) {
	if (CheckEmv(pse) == 1) {
            reset_connection();	
	    int fret = 0;
            fret = ReadTheCard(pse);
            if (fret == 0) {
                std::cout << "Error in reading card !" << std::endl;
                ret = 2;
            } else {
		std::cout << "Read Ok DataPrinted above and put into gloabl variables \n [SW] read for example was : " << sw_string1 << " " << sw_string2;	
            }				
            reset_connection();	
	    fret = 0;
	    uint64_t pan_read = 0;                                                           // primary account number
            fret = GetPan(pse, &pan_read);
            if (fret == 0) {
                std::cout << "Error in reading pan from card !" << std::endl;
                ret = 3;
            } else {
		std::cout << "pan read returned = " << pan_read << std::endl;
            }
            reset_connection();	
	    fret = 0;
            fret = GetLastTranasction(pse);
            if (fret == 0) {
                std::cout << "Error in reading last transaction from card !" << std::endl;
                ret = 4;
            }	
        } else {
            ret = 0;	
        }			
    } else {
        ret = 0;
    }
    return ret;	
}

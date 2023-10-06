#include <stdio.h>
#include "libusb.h"
#pragma comment(lib,"libusb-1.0.lib")

#define VENDERID 0x04D8
#define PRODUCTID 0x0053

#define EP1 0x1

int main()
{
    libusb_device_handle *devh = NULL;
    unsigned char    send_data[64] = {0}; //マイコン側の受信配列と同じか以下の大きさに合わせること
    unsigned char receive_data[64] = {0}; //マイコン側の送信配列と同じか以下の大きさに合わせること
    int data_len;

    int r;

    try
    {
        //初期化
        r = libusb_init(NULL);
        if ( r < 0 )
            throw(libusb_error_name(r));

        //デバッグ設定
        libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);

        //デバイスハンドル取得
        devh = libusb_open_device_with_vid_pid(NULL, VENDERID, PRODUCTID);
        if ( devh == NULL )
            throw("Device Not Found\n");

        //使用権要求
        r = libusb_claim_interface(devh, 0);
        if ( r != 0 )
            throw(libusb_error_name(r));

        //送信
        int dummy = 0;
        send_data[0] = 0x81;
        r = libusb_bulk_transfer(devh, LIBUSB_ENDPOINT_OUT | EP1, send_data, sizeof(send_data), &dummy, 1000);
        if ( r != 0 )
            throw(libusb_error_name(r));

        //受信
        r = libusb_bulk_transfer(devh, LIBUSB_ENDPOINT_IN | EP1, receive_data, sizeof(receive_data), &data_len, 1000);
        if ( r != 0 )
            throw(libusb_error_name(r));

        //受信データの表示
        printf("Received : %d Bytes\n", data_len);
        for ( int i = 0; i < data_len; i++ )
            printf("%02X ", receive_data[i]);

        printf("\n");
    } catch ( const char* e )
    {
        //例外処理
        printf(e);
    }

    //開放処理
    if ( devh != NULL )
        libusb_close(devh);
    libusb_exit(NULL);
    return 0;
}
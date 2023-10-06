// ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
//
// Desc :: THis is an example of reading and writing using libusb 1.0.0-dev
//
// compile gcc -o windows_usb_example windows_usb_example.c -Wall -lusb-1.0
// sudo apt update
// sudo apt-get install libusb-1.0-0-dev
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••

#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <stdlib.h>
#pragma comment(lib,"libusb-1.0.lib")

// This is defined by the target device hardware 
// (in this test its the Sony Alpha Camera)
//
#define MY_VID 0x054C    // Vendor ID
#define MY_PID 0x0D9F    // Product ID

#define TIMEOUT 1000

// endpoints defined by the hardware chip
// find them with this command under linux
// sudo lsusb -d 054c: -v where 054 is the vendor ID. 
#define EP_DATA_IN  0x81
#define EP_DATA_OUT 0x02
#define EP_DATA_IN2 0x83

#define DEVICE_CONFIGURATION 0
#define USB_CONFIG_NUM  1

/*
    windows library example for USB
*/
void die(char* s, libusb_device_handle *devh)
{
    printf("Error: ");
    puts(s);
	if (devh != NULL) {
        libusb_close(devh);
	}
    libusb_exit(NULL);
	exit(EXIT_FAILURE);
}

int main()
{
    int ret;
    libusb_device_handle *devh = NULL;
	
    // Initialization
    if ( libusb_init(NULL) < 0 )
        die("libusb_init FAILED", devh);
    printf("Init\n");
	
    // Setting the debug level (messages will appear when communication fails, etc.)
    libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);
    printf("Set debug\n");

    // Open the device with VID and PID
    devh = libusb_open_device_with_vid_pid(NULL, MY_VID, MY_PID);
    if ( devh == NULL)
        die("libusb_open_device_with_vid_pid", devh);
    printf("libusb_open_device_with_vid_pid\n");

    if(libusb_kernel_driver_active(devh, DEVICE_CONFIGURATION) == 1) {
        printf("Kernel Driver Active");
        if(libusb_detach_kernel_driver(devh, DEVICE_CONFIGURATION) == 0)
            printf("Kernel Driver Detached!");
    }

    // if you have an alt interface uncomment and use itoa
	//
    //ret = libusb_set_interface_alt_setting(devh, DEVICE_CONFIGURATION, 1);
    //if(ret != 0) {
    //     printf("Cannot configure alternate setting");
    //     die("libusb_set_interface_alt_setting : CANT_CONFIGURE", devh);
    //}
	
    if (LIBUSB_SUCCESS != libusb_set_configuration(devh, USB_CONFIG_NUM)) {
        printf("Failed to set-configuration on HIL interface");
        die("libusb_set_configuration : CANT_SET_CONFIG", devh);
    }
	
    // Request permission to use the device's interface
    // If you do not do this
    // libusb: error [winusbx_submit_bulk_transfer] unable to match endpoint to an open interface - cancelling transfer
    // 
    ret = libusb_claim_interface(devh,0);
    if ( ret == 0 )
    {
        printf("OK\n");
    } else if ( ret == LIBUSB_ERROR_NOT_FOUND )
        die("libusb_claim_interface : LIBUSB_ERROR_NOT_FOUND", devh);
    else if ( ret == LIBUSB_ERROR_BUSY )
        die("libusb_claim_interface : LIBUSB_ERROR_BUSY", devh);
    else if ( ret == LIBUSB_ERROR_OVERFLOW )
        die("llibusb_claim_interface : LIBUSB_ERROR_NO_DEVICE", devh);
    else
        die("libusb_claim_interface : UNKNOWN %d", devh);
    printf("libusb_claim_interface\n");

    //---

    unsigned char dataV[64]={0x81};
    int data_len=64;

    //---

    // EP=1 to bulk send. Timeout 1000ms writing
    //ret = libusb_bulk_transfer(devh, LIBUSB_ENDPOINT_OUT | 1, dataV, sizeof(dataV), &data_len, TIMEOUT);

    //ret = libusb_bulk_transfer(devh, EP_DATA_OUT | 1, dataV, sizeof(dataV), &data_len, TIMEOUT);
    ret = libusb_bulk_transfer(devh, EP_DATA_OUT, dataV, sizeof(dataV), &data_len, TIMEOUT);
	
    if ( ret == 0 )
    {
        printf("Received : %d Bytes\n", data_len);
        for ( int i = 0; i<data_len; i++ )
        {
            printf("%02X ", dataV[i]);
        }
    } else if ( ret == LIBUSB_ERROR_TIMEOUT )
        die("libusb_bulk_transfer : LIBUSB_ERROR_TIMEOUT", devh);
    else if ( ret == LIBUSB_ERROR_PIPE )
        die("libusb_bulk_transfer : LIBUSB_ERROR_PIPE", devh);
    else if ( ret == LIBUSB_ERROR_OVERFLOW )
        die("libusb_bulk_transfer : LIBUSB_ERROR_OVERFLOW", devh);
    else if ( ret == LIBUSB_ERROR_NO_DEVICE )
        die("libusb_bulk_transfer : LIBUSB_ERROR_NO_DEVICE", devh);
    else {
		printf("libusb_control_transfer error: %s", libusb_error_name(ret));
        die("libusb_bulk_transfer : UNKNOWN", devh);
	}

    printf("\nSent\n");

    //---

    // EP=1 from bulk reception. Timeout 1000ms reading
    //ret = libusb_bulk_transfer(devh, LIBUSB_ENDPOINT_IN|1, dataV, sizeof(dataV), &data_len, TIMEOUT);
	ret = libusb_bulk_transfer(devh, EP_DATA_IN | 1, dataV, sizeof(dataV), &data_len, TIMEOUT);
	//ret = libusb_bulk_transfer(devh, EP_DATA_IN, dataV, sizeof(dataV), &data_len, TIMEOUT);
	
    if( ret == 0 )
    {
        printf("Received : %d Bytes\n",data_len);
        for(int i=0;i<data_len;i++ )
        {
            printf("%02X ",dataV[i]);
        }
    } else if ( ret == LIBUSB_ERROR_TIMEOUT )
        die("libusb_bulk_transfer : LIBUSB_ERROR_TIMEOUT", devh);
    else if ( ret == LIBUSB_ERROR_PIPE )
        die("libusb_bulk_transfer : LIBUSB_ERROR_PIPE", devh);
    else if ( ret == LIBUSB_ERROR_OVERFLOW )
        die("libusb_bulk_transfer : LIBUSB_ERROR_OVERFLOW", devh);
    else if ( ret == LIBUSB_ERROR_NO_DEVICE )
        die("libusb_bulk_transfer : LIBUSB_ERROR_NO_DEVICE", devh);
    else
        die("libusb_bulk_transfer : UNKNOWN", devh);


    printf("\nReceived\n");

    // clean up.
    libusb_close(devh);
    printf("Done\n");
    libusb_exit(NULL);
    exit(EXIT_SUCCESS);
}
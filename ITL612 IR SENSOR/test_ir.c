// ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
// Desc :: THis is an example of reading and writing using libusb-dev
// 
// Compiles as : gcc -o test_libusb0 test_libusb0.c -lusb
// /opt/intel/oneapi/compiler/2022.2.1/linux/bin/icx -o test_ir test_ir.c -Wall -lusb -O2 -qopt-report
// clang -o test_ir test_ir.c -Wall -lusb -O2
//
// to install sudo apt-get install libusb-dev
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <usb.h>
#include <stdlib.h>

#include "itl612.h"

//Define
//#define USB_VENDOR 0x04B4                 // <--- we can only find these when we have the device
//#define USB_PRODUCT 0x8613
//#define USB_VENDOR 0x1f00
//#define USB_PRODUCT 0x2012
#define USB_VENDOR 0x054C                  // <--- we can only find these when we have the device this has been set to the Sony Camera for testing purpose
#define USB_PRODUCT 0x0D9F
#define TIMEOUT (5*1000)
// define the ENDPOINT
//#define EP6	0x86
// other endpoints from command
// sudo lsusb -d 054c: -v
//
#define ENDPOINT_OUT            0x02                                                           //  WRITE OUT
#define ENDPOINT_IN             0x81                                                           //  READ IN
#define ENDPOINT_INTERRUPT_IN   0x83                                                           //  interrupt read
#define USB_RT_PORT	(USB_TYPE_CLASS | USB_RECIP_OTHER)

#define QUERY_MSG_LEN 12
#define RCV_MSG_LEN 64u

#include <unistd.h>

#include "itl612.c"

/* Init USB */
struct usb_bus *USB_init()
{
	usb_init();
	usb_find_busses();
	usb_find_devices();
	return(usb_get_busses());
}

/* Find USB device  */
struct usb_device *USB_find(struct usb_bus *busses, struct usb_device *dev)
{
	struct usb_bus *bus;
	if (busses == NULL)  {
		return ( NULL );
	}
	for(bus=busses; bus; bus=bus->next){
		for(dev=bus->devices; dev; dev=dev->next) {
			if( (dev->descriptor.idVendor==USB_VENDOR) && (dev->descriptor.idProduct==USB_PRODUCT) ){
				printf("\033[32m DEVICE=%x PRODUCT=%x\n \033[0m",dev->descriptor.idVendor, dev->descriptor.idProduct);
				return( dev );
			}
			else {
				printf("\033[31m Non matching device found :: dev=%x pro=%x\n \033[0m",dev->descriptor.idVendor, dev->descriptor.idProduct);
			}
		}
	}
	return( NULL );
}

/* USB Open */
struct usb_dev_handle *USB_open(struct usb_device *dev)
{
	struct usb_dev_handle *udev = NULL;

	udev=usb_open(dev);
	if( (udev=usb_open(dev))==NULL ){
		fprintf(stderr,"usb_open Error.(%s)\n",usb_strerror());
		exit(1);
	}

	if( usb_set_configuration(udev,dev->config->bConfigurationValue)<0 ){
		if( usb_detach_kernel_driver_np(udev,dev->config->interface->altsetting->bInterfaceNumber)<0 ){
			fprintf(stderr,"usb_set_configuration Error.\n");
			fprintf(stderr,"usb_detach_kernel_driver_np Error.(%s)\n",usb_strerror());
 		}
	}

	if( usb_claim_interface(udev,dev->config->interface->altsetting->bInterfaceNumber)<0 ){
		if( usb_detach_kernel_driver_np(udev,dev->config->interface->altsetting->bInterfaceNumber)<0 ){
			fprintf(stderr,"usb_claim_interface Error.\n");
			fprintf(stderr,"usb_detach_kernel_driver_np Error.(%s)\n",usb_strerror());
		}
	}

	if( usb_claim_interface(udev,dev->config->interface->altsetting->bInterfaceNumber)<0 ){
		fprintf(stderr,"usb_claim_interface Error.(%s)\n",usb_strerror());
	}

	return(udev);
}

/* USB Close */
void USB_close(struct usb_dev_handle *dh)
{
	if (dh == NULL) {
		return;
	}
	if(usb_release_interface(dh, 0)){
		fprintf(stderr,"usb_release_interface() failed. (%s)\n",usb_strerror());
	}
	if( usb_close(dh)<0 ){
		fprintf(stderr,"usb_close Error.(%s)\n",usb_strerror());
	}
}

/* USB altinterface */
void USB_altinterface(struct usb_dev_handle *dh,int tyep)
{
	if (dh == NULL) {
		return;
	}
	if(usb_set_altinterface(dh,tyep)<0)
	{
		fprintf(stderr,"Failed to set altinterface %d: %s\n", 1,usb_strerror());
		USB_close(dh);
	}
}

/* send error message then exit this program */
void die(char* s, struct usb_dev_handle *dh)
{
    printf("Error: ");
    puts(s);
    USB_close(dh);
    exit (EXIT_FAILURE);
    //libusb_exit(NULL);
}

int main(void)
{
	struct usb_bus *bus;
	struct usb_device *dev = NULL;
	usb_dev_handle *dh;
    const int timeout1 = 1000;
	
	/* Initialize */
	bus=USB_init();
	dev=USB_find(bus,dev);
	if( dev==NULL ){
		fprintf(stderr,"Device not found\n");
        exit (EXIT_FAILURE); 
	}
	printf("Initialize OK\n");
	
	/*-------------*/
	/* Device Open */
	/*-------------*/
	dh=USB_open(dev);
	if( dh==NULL ){ exit(2); }
	printf("Device Open OK\n");

	/*
		Read Write Buffers 
	*/
    unsigned char dataV[RCV_MSG_LEN+1]={0x81};               // This is the data received message
	unsigned char ctl[QUERY_MSG_LEN+1]= {0x55, 0xAA, 0x07, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x04, 0xF0, 0x00};             // THis is the control message to send out
    int data_len=1;
    int ret = 0;
	
    //---

    // Write the data the device

    // •••••••••••••••••••••••••••••••••••••••••••••••• write using the control message ••••••••••••••••••••••••••••••••••••••••••••••••
    //	
    // usb_control_msg(dh, request_type, request, value, index, msg, sizeof(msg), timeout);
	//               read in (dh, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 0x03/RQ_IO_READ, 
	//               or write out USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, RQ_IO_WRITE
	// NB: may require pipe This value is created by calling eitherusb_sndctrlpipe orusb_rcvctrlpipe as variable after dh
	//      request - requests are documented in Section 9.4 of the USB 2.0 specification
	//      <bit7>=host to device - 0, bit<6-5> = 0=std, 1=class, 2=vendor, 3=reserved bit<4-0> 1=interface 0=device 2=endpoint
	//      other examples being power req_type=USB_REQ_SET_FEATURE or CLEAR_FEATURE and request USB_PORT_FEAT_POWER or USB_PORT_FEAT_INDICATOR
	//      
	//      ret=usb_control_msg(dh, USB_REQ_SET_FEATURE, USB_PORT_FEAT_POWER, 0x01, port, NULL, 0, TIMEOUT);
	//      ret=usb_control_msg(dh, USB_REQ_CLEAR_FEATURE, USB_PORT_FEAT_POWER, 0x00, port, NULL, 0, TIMEOUT);
	//      ret=usb_control_msg(dh, USB_REQ_SET_FEATURE, USB_PORT_FEAT_INDICATOR, 0x01, ((1<<8) | port), NULL, 0, TIMEOUT);
	//      ret=usb_control_msg(dh, USB_REQ_SET_FEATURE, USB_PORT_FEAT_INDICATOR, 0x00, port, NULL, 0, TIMEOUT);
	//
	//      value = Standard types are documented in Section 9.3.1 of the USB 2.0 specification
	//      9=SET_CONFIGURATION, 0=GET_STATUS, 1=CLEAR_FEATURE, 3-SET_FEATURE, 5=SET_ADDRESS, 6=GET_DESCRIPTION 7=SET_DESCRIPTION
	//      8=GET_CONFIGURATION, GET_INTERFACE=10, SET_INTERFACE=11 this is defined in example USB_REQ_GET_STATUS
	//      ---------------------------------------------------------------------------------------------------------------
	//      USB_REQ_SET_CONFIGURATION = 0x09 USB_REQ_SET_CONFIGURATION ....> 0xc4, 0x02
	//      Request type bits of the bmRequestType field in control transfers.
    //      Enumerator
    //      LIBUSB_REQUEST_TYPE_STANDARD 	
    //      Standard.
    //
    //      LIBUSB_REQUEST_TYPE_CLASS 	
    //      Class.
    //
    //      LIBUSB_REQUEST_TYPE_VENDOR 	
    //      Vendor.
    //
    //      LIBUSB_REQUEST_TYPE_RESERVED 	
    //      Reserved.
	//      ------------------------------------------------------------------------------------------------------------------
    //      Recipient bits of the bmRequestType field in control transfers.
    //      Enumerator
    //      LIBUSB_RECIPIENT_DEVICE 	 
    //      Device.
    //      LIBUSB_RECIPIENT_INTERFACE 	
    //      Interface.
    //      LIBUSB_RECIPIENT_ENDPOINT 	
    //      Endpoint.
    //      LIBUSB_RECIPIENT_OTHER 	
    //      Other.
	
    ret=usb_control_msg(dh, 0x21, 0x09, 0x00, 0x00, (char*) ctl, QUERY_MSG_LEN, TIMEOUT);
    if (ret < 0)
	{
	  fprintf (stderr, "cannot write data using usb_control or no message returned, %s (%d) ret=%d\n", strerror(errno), errno, ret);
	  //die("usb_control_msg : FAILED TO WRITE",dh);
	}

    // example to use this type of write to set-up the video algorythms
    //
	uint8_t cmd = ITL612_CC_CW4_ASS_ON;
	ITL612_Video_Option_p3_CC_e vid_algo_option = Anti_striation_switch;
	ITL612_DataFrameQuery_t video_algo_setup[8u];                                          // make a structure array for each video algorithm option and set them to your requirement
	compose_video_page3_msg( &video_algo_setup[0u], vid_algo_option, cmd );
	cmd = ITL612_CC_CW4_IMS_EM;
	vid_algo_option = image_mode;
	compose_video_page3_msg( &video_algo_setup[1u], vid_algo_option, cmd );
	vid_algo_option = brightness;
	uint8_t set_val = 0;
	SET_BRIGHT(set_val,6);
	compose_video_page3_msg( &video_algo_setup[2u], vid_algo_option, set_val );
	vid_algo_option = contrast;
	SET_CONTRAST(set_val,200);
	compose_video_page3_msg( &video_algo_setup[3u], vid_algo_option, set_val );
	vid_algo_option = enhanced_detail;
	SET_EDG(set_val,200);
	compose_video_page3_msg( &video_algo_setup[4u], vid_algo_option, set_val );
	vid_algo_option = dimming_mode;
	cmd = ITL612_CC_CW4_DIM_3;
	compose_video_page3_msg( &video_algo_setup[5u], vid_algo_option, cmd );
	vid_algo_option = hue;
	cmd = ITL612_CC_CW4_HUE_COOL;
	compose_video_page3_msg( &video_algo_setup[6u], vid_algo_option, cmd );	
	vid_algo_option = image_obs_mode;
	cmd = ITL612_CC_CW4_OBS_TEMP;
	compose_video_page3_msg( &video_algo_setup[6u], vid_algo_option, cmd );	
	
	for ( int ii=9; 0<ii; ) {                                                                 // bulk send 8 frames as composed in previous section
		ii--;
	    copy_query_to_send_buffer( (char*) ctl, &video_algo_setup[ii-1], ITL612_DATA_FRAME_QUERY_LEN );
        ret=usb_control_msg(dh, 0x21, 0x09, 0x00, 0x00, (char*) ctl, QUERY_MSG_LEN, TIMEOUT);
        if (ret < 0)
	    {
	       fprintf (stderr, "cannot write data using usb_control or no message returned, %s (%d) ret=%d\n", strerror(errno), errno, ret);
	       //die("usb_control_msg : FAILED TO WRITE",dh);
	    }
    }
	
    // •••••••••••••••••••••••••••••••••••••••••••••••• bulk write ••••••••••••••••••••••••••••••••••••••••••••••••
	//char array[2]; <-- dummy data test (removed for now)
	//array[0]='a';
    //array[1]='b';
	//int num=usb_bulk_write(dh, ENDPOINT_OUT, array, 2, timeout1);
	
	// compose a set-up message to reset and restore to default settings
	//
	uint8_t cw4 = ITL612_CC_CW4_FACT_DEF;
	ITL612_SetUp_Option_CC_e selected_option = factory_restore;
	ITL612_DataFrameQuery_t msg_strut;
	compose_setup_msg( &msg_strut, selected_option, cw4 );
	copy_query_to_send_buffer( (char*) ctl, &msg_strut, ITL612_DATA_FRAME_QUERY_LEN );
	
	// send it with bulk send
	//
    int num=usb_bulk_write(dh, ENDPOINT_OUT, (char*) ctl, ITL612_DATA_FRAME_QUERY_LEN, timeout1);
	if (num < 0) {
		printf ("Failed to bulk write, got %s\n", usb_strerror());
		//die("libusb_bulk_write : FAILED TO WRITE",dh);
	} else {
		printf ("\033[33m bulk write wrote %d bytes\n", num);		
	}
	
	// compose a set-up message to save the data.
	//
	cw4 = ITL612_CC_CW4_SAVE_SET;
	selected_option = save;
	compose_setup_msg( &msg_strut, selected_option, cw4 );
	copy_query_to_send_buffer( (char*) ctl, &msg_strut, ITL612_DATA_FRAME_QUERY_LEN );
	usleep(50);
	// send it with bulk send
	//
    num=usb_bulk_write(dh, ENDPOINT_OUT, (char*) ctl, ITL612_DATA_FRAME_QUERY_LEN, timeout1);
	if (num < 0) {
		printf ("Failed to bulk write, got %s\n", usb_strerror());
		//die("libusb_bulk_write : FAILED TO WRITE",dh);
	} else {
		printf ("\033[33m bulk write wrote %d bytes\n", num);		
	}
	
    // •••••••••••••••••••••••••••••••••••••••••••••••• interrupt write ••••••••••••••••••••••••••••••••••••••••••••••••
    int rc;
	unsigned char buffer[QUERY_MSG_LEN+1] = {0x55, 0xAA, 0x07, 0x01, 0x00, 0x05, 0x00, 0x00, 0x00, 0x01, 0x02, 0xF0, 0x00};
	//memset(buffer, 0x00, sizeof (buffer));
	//buffer[0] = 0x4C;
	// this is where you would copy the message formulated
	//if (in != NULL)
	//	memcpy (buffer + 1, in, in_size);
	//rc = usb_interrupt_write(dh, 0x01, (char*) buffer, QUERY_MSG_LEN, TIMEOUT);
	rc = usb_interrupt_write(dh, ENDPOINT_INTERRUPT_IN, (char*) buffer, QUERY_MSG_LEN, TIMEOUT);
	if (rc < 0) {
		printf ("Failed to interrupt write, got %s\n", usb_strerror ());
		//die("usb_interrupt_write : FAILED TO WRITE",dh);
	}
	else if (rc < QUERY_MSG_LEN) {
		printf ("Failed to interrupt write stream only wrote %d bytes\n", rc);
		//die("usb_interrupt_write : FAILED TO WRITE STREAM",dh);
	}
	
    // •••••••••••••••••••••••••••••••••••••••••••••••• data read ••••••••••••••••••••••••••••••••••••••••••••••••
    ret=usb_bulk_read(dh, ENDPOINT_IN, (char*)dataV, data_len, TIMEOUT);
    if(ret < data_len){printf ("bulk read error. (%s)\n", usb_strerror());}
	
	printf("USB Close\n");
	USB_close(dh);
	printf("USB End\n");
    exit (EXIT_SUCCESS);
}



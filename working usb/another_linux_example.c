//gcc -o test_ez_read test_ez_read.c -lusb

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <usb.h>

//Define
#define USB_VENDOR 0x1f00
#define USB_PRODUCT 0x2012
#define TIMEOUT (5*1000)
#define EP6	0x86

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
	for(bus=busses; bus; bus=bus->next){
		for(dev=bus->devices; dev; dev=dev->next) {
			if( (dev->descriptor.idVendor==USB_VENDOR) && (dev->descriptor.idProduct==USB_PRODUCT) ){
				return( dev );
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
	if(usb_release_interface(dh, 0)){
		fprintf(stderr,"usb_release_interface() failed. (%s)\n",usb_strerror());
		usb_close(dh);
	}
	if( usb_close(dh)<0 ){
		fprintf(stderr,"usb_close Error.(%s)\n",usb_strerror());
	}
}

/* USB altinterface */
void USB_altinterface(struct usb_dev_handle *dh,int tyep)
{
	if(usb_set_altinterface(dh,tyep)<0)
	{
		fprintf(stderr,"Failed to set altinterface %d: %s\n", 1,usb_strerror());
		USB_close(dh);
	}
}

main()
{
	struct usb_bus *bus;
	struct usb_device *dev;
	usb_dev_handle *dh;
	unsigned char buf[512];
	int ret;
	/* Initialize */

	printf("starting!\n");
	bus=USB_init();
	dev=USB_find(bus,dev);
	if( dev==NULL ){ 




               printf("device not found\n");




		exit(1); }
	printf("Initialize OK\n");
	/*-------------*/
	/* Device Open */
	/*-------------*/
	dh=USB_open(dev);
	if( dh==NULL ){ exit(2); }
	printf("Device Open OK\n");

	/*
		Put command here
	*/
	printf("Start Read data from EP6\n");
	USB_altinterface(dh,1);
	ret = usb_bulk_read(dh,EP6, buf, sizeof(buf), TIMEOUT);
	if(ret<0){
		fprintf(stderr,"usb_bulk_read() failed? buf(%d) usb(%d)\n",sizeof(buf), ret);
		fprintf(stderr,"usb_bulk_read error.(%s)\n",usb_strerror());
		USB_close(dh);
		return 1;
	}
	printf("usb_bulk_read() finished\n");
	fwrite(buf, 1, sizeof(buf), stdout);

	USB_close(dh);
	printf("USB End\n");
	return 0;
}
/* gcc -lusb -o hoge hoge.c */

#include <usb.h>

struct usb_bus *bus;
struct usb_device *dev;
usb_dev_handle *dh;

usb_init();
usb_find_busses();
usb_find_devices();

for (bus = usb_get_busses(); bus; bus = bus->next) {
   for (dev = bus->devices; dev; dev = dev->next) {
	if (dev->descriptor.idVendor == VENDOR_ID &&
	    dev->descriptor.idProduct == PRODUCT_ID) {
	    goto device_found;
	}
   }
}

/* デバイスが見つからなかった場合 */
fprintf(stderr, "Device not found.\n");
exit(1);
/* デバイスが見つかった場合 */
device_found:
printf("Device Found!!\n"); /* 確認用*

if( (dh=usb_open(dev))==NULL ){
  printf("usb_open Error.(%s)\n",usb_strerror());
  exit(1);
}

/* read */
result=usb_bulk_read(dh, end_point, readbuf, bytes, timeout);
if(result < bytes){printf("bulk read error. (%s)\n", usb_strerror());}


/* if you dont close you may have issues with further use of the usb */
usb_close(dh);
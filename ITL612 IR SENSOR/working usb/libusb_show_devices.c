// compile gcc -O -Wall libusb-sample.c -lusb-1.0
// sudo apt update
// sudo apt-get install libusb-1.0-0-dev
//
#include <stdio.h>
#include <libusb-1.0/libusb.h>

int main (void) {
  libusb_device **list;
  struct libusb_device_descriptor desc;
  libusb_device_handle *handle;
  int i, ret;
  unsigned char text[512];

  /* Initialization */
  libusb_init (NULL); 
  
  /* Get a list of devices */
  int cnt = libusb_get_device_list(NULL, &list); //
  
  /* Scan the list to list the devices  */
  for (i = 0; i < cnt; i++) {
    libusb_device *dev = list[i];
    libusb_get_device_descriptor(dev, &desc);
    ret = libusb_open(dev, &handle);
  
    /* print description vendor_id and product_id */
    if (ret == 0) {
      libusb_get_string_descriptor_ascii(handle, desc.iProduct, text, sizeof(text));
      printf ("desc=%s vendor=%x product=%x\n", text, desc.idVendor, desc.idProduct);
    }
  }

  /* close */
  libusb_free_device_list(list, 1);
  libusb_exit(NULL);
  return 0;
}
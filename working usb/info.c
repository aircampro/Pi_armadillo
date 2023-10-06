

typedef struct usbRequest{
    uchar       bmRequestType;
    uchar       bRequest;
    usbWord_t   wValue;
    usbWord_t   wIndex;
    usbWord_t   wLength;
}usbRequest_t;

static int usbOpenDevice(usb_dev_handle **device, int idvendor, int idproduct)
{
  struct usb_bus *bus;
  struct usb_device *dev;
  usb_dev_handle *udh=NULL;
  int ret,retp, retm,errors;
  char string[256];
  usb_init();
  usb_find_busses();
  ret=usb_find_devices();
  if(ret==0){return errors=1;}
  for (bus = usb_busses; bus; bus = bus->next)
  {
    for (dev = bus->devices; dev; dev = dev->next)
    {
      udh=usb_open(dev);
      retp = usb_get_string_simple(udh, dev->descriptor.iProduct, string, sizeof(string));
      retm=usb_get_string_simple(udh, dev->descriptor.iManufacturer, string, sizeof(string));
      if (retp > 0 && retm > 0)
        if (idvendor==dev->descriptor.idVendor && idproduct==dev->descriptor.idProduct){ 
          *device=udh;return errors=0;}
    }
  }
         usb_close(udh);return errors=1;
}

int main(int argc, char **argv)
{
  usb_dev_handle *d=NULL;
  unsigned char buffer[16];
  unsigned char i=3,j=4,k=5,l=6,m=7,n=8,o=9,p=0,ret;
  char string[256];
//if(argc<2){return 0;}
//j=atoi(argv[1]);
ret=usbOpenDevice(&d, IDVendor,IDProduct);
if(ret!=0){printf("usbOpenDevice failed\n"); return 0;}

  ret=usb_control_msg(d, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
           i, j+256*k, l+256*m,(char *)buffer, n+256*o,5000);
printf("ret=%d \n",ret);
for(p=0;p<ret;p++){printf("buffer[%d]=%d \n",p, buffer[p]);}

  return 0;
}
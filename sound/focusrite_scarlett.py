#
# Example of using usb backend 
# ref:- https://qiita.com/c_kuwata/items/2dcf59f7e8e16a79b8d9
# Focusrite Scarlett 18i20 audio interface https://store.shimamura.co.jp/ec/pro/disp/1/mt0169696
#
import usb.core
import usb.backend.libusb1
from ctypes import c_void_p, c_int
backend = usb.backend.libusb1.get_backend(find_library=lambda x: "libusb-1.0.dll")
backend.lib.libusb_set_option.argtypes = [c_void_p, c_int]
backend.lib.libusb_set_debug(backend.ctx, 5)
# The following are options for using usbdk
backend.lib.libusb_set_option(backend.ctx, 1)
# Scarlett 18i20 vendor and product id numbers
VENDOR_ID = 0x1235
PRODUCT_ID = 0x8215
device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=backend)
device.set_configuration()
# The setup sequence runs automatically because of usbdk, and the next serial number is usually 0x77.
device.ctrl_transfer(0x21, 0x02, 0x00, 0x03,
    [0x00, 0x00, 0x80, 0x00, 0x08, 0x00, 0x77, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x7C, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00])

# device read with timeout 100ms
ret = device.read(0x83, 8, 100)
print(' '.join(map(lambda x: '{0:0{1}x}'.format(x, 2), ret)))
ret = bytearray(device.ctrl_transfer(0xA1, 0x03, 0x00, 0x03, 0x0028))
print(' '.join(map(lambda x: '{0:0{1}x}'.format(x, 2), ret)))

#RECV Control 00 00 80 00 18 00 77 00 00 00 00 00 00 00 00 00 
#             01 00 00 00 00 00 00 00 01 00 00 00 01 00 00 00
#             01 00 00 01 00 00 00 01
#             
device.ctrl_transfer(0x21, 0x02, 0x00, 0x03,
    [0x00, 0x00, 0x80, 0x00, 0x08, 0x00, 0xFF, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x7C, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00])

#RECV Control 00 00 80 00 00 00 77 00 03 00 00 00 00 00 00 00
     
#1..8 bytes: Setup packet for control transfer.
#9..16 bytes: Is it a response status, including the error case described below?
#17..24 bytes: Analog input LINE/INST
#25..32 bytes: Analog input PAD
#33..40 bytes: Analog input AIR

#RECV Interrupt 00 00 80 00 00 00 00 00
#SEND Control   21 02 00 00 03 00 18 00 00 00 80 00 08 00 EE 99
#               00 00 00 00 00 00 00 00 7C 00 00 00 18 00 00 00
#RECV Control
#SEND Interrupt 
#RECV Interrupt 01 00 00 00 00 00 00 00
#SEND Control   A1 03 00 00 03 00 28 00
#RECV Control   00 00 80 00 18 00 EE 99 00 00 00 00 00 00 00 00 
#               01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
#               00 00 00 00 00 00 00 00
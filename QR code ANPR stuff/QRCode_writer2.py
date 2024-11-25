#!/usr/bin/env python
#
# QRCode_writer2.py
#
# examples shown using both libraries as below
#
# pip install opencv-python	
# pip install pyqrcode
# pip install pypng
#
# pip install qrcode
# pip install pillow
import qrcode
import pyqrcode
import cv2

import glob
import sys

# for printing out decide on the o/s
#
import platform
syst=platform.system()
GEN_ONLY=0
if not syst.find("Win") == -1:
    # win32
    import win32api
    import win32print
    
    def PrintOut(f_arg):
        filepath =f_arg
        win32api.ShellExecute(
            0,
            "print",
            filepath,
            "/c:""%s" % win32print.GetDefaultPrinter(),
            ".",
            0
        )        
elif not syst.find("Lin") == -1:
    #linux
    from subprocess import run, Popen, PIPE

    def PrintOut(f_arg):
        cpl_proc = run(["lp", f_arg], capture_output=True, text=True)  
        print("print code ", cpl_proc.returncode)
        print(cpl_proc.stdout)
else:
    print("os not supported for printing !", syst)    
    GEN_ONLY=1
    
# write QR code file names
FILE_PNG_A = 'qrcode_A.png'
FILE_PNG_B = 'qrcode_B.png'
FILE_PNG_C = "grcode_C.png"
FILE_PNG_D = "grcode_D.png"
FILE_PNG_AB = 'qrcode_AB.png'

if __name__ == "__main__": 
   
    # QR code 1
    code = pyqrcode.create('http://www.github.com/aircampro', error='L', version=3, mode='binary')
    code.png(FILE_PNG_A, scale=5, module_color=[0, 0, 0, 128], background=[55, 255, 255])

    # QR code 2
    code = pyqrcode.create('http://www.github.com/mark-nick-o', error='L', version=3, mode='binary')
    code.png(FILE_PNG_B, scale=5, module_color=[0, 0, 0, 128], background=[255, 255, 55])

    # QR code 3
    code = pyqrcode.create(b'\x04\x21\x22\x0a\x30\x31\x65\x29\x67\x76', mode='binary')
    code.png(FILE_PNG_C, scale=10)
    
    # QR code 4
    qr = qrcode.QRCode(
        version=3,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=20,
        border=3
    )
    qr.add_data('QR code Test data 12$@')
    qr.make()
    img = qr.make_image()
    img.save(FILE_PNG_D)

    # concatenate the files A+B to a single QR code
    im1 = cv2.imread(FILE_PNG_A)
    im2 = cv2.imread(FILE_PNG_B)

    im_h = cv2.hconcat([im1, im2])
    cv2.imwrite(FILE_PNG_AB, im_h)

    # show the image and wait for feedback
    cv2.imshow('QR A+B combined image', im_h)
    cv2.waitKey(0)

    # print the QR Codes 
    if GEN_ONLY == 0:
        PrintOut(FILE_PNG_A)
        PrintOut(FILE_PNG_B)
        PrintOut(FILE_PNG_AB)
        PrintOut(FILE_PNG_C)
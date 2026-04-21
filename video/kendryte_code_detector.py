#
# Code for kendryte https://www.kendryte.com/en/products
# ref:- example https://www.kendryte.com/en/appDetail?id=9&title=%E8%BD%A6%E7%89%8C%E8%AF%86%E5%88%AB
#
import sensor, image, time, lcd
from maix import KPU, utils
import gc

class ExtreactionMode:
    barcode = 1
    qrcode = 2
    april = 3

class CodeReader:
    def __init__(self, hmim=False, vf=False, mode=ExtreactionMode.barcode):
        lcd.init()
        sensor.reset()                      # Reset and initialize the sensor. It will                                   # run automatically, call sensor.run(0) to stop
        sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
        sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
        sensor.skip_frames(time = 1000)     # Wait for settings take effect.
        if hmim == True:
            sensor.set_hmirror(1)
        if vf == True:
            sensor.set_vflip(1)
        self.clock = time.clock()                # Create a clock object to track the FPS.
        self.mode = mode

    def check_video():
        self.clock.tick()                    # Update the FPS clock.
        img = sensor.snapshot()
        fps = self.clock.fps()
        if self.mode == ExtreactionMode.barcode:
            barcode = img.find_barcodes()
            if barcode is not None:		
                pay = barcode.payload()
                a=img.draw_string(10, 10, "%2.payload" %(pay), color=(255, 255, 0), scale=2.0)
        elif self.mode == ExtreactionMode.qrcode:
            qr = img.find_qrcodes()	
            if qr is not None:		
                pay = qr.payload()			
                a=img.draw_string(10, 10, "%2.payload" %(pay), color=(255, 255, 0), scale=2.0)	
        elif self.mode == ExtreactionMode.april:
            apriltag = img.find_apriltags()	
            if apriltag is not None:		
                pay = apriltag.id()			
                a=img.draw_string(10, 10, "%2.payload" %(pay), color=(255, 255, 0), scale=2.0)				
        a=img.draw_string(0, 0, "%2.1ffps" %(fps), color=(255, 255, 0), scale=2.0)

        lcd.display(img)
        print("mem free:",gc.mem_free())
        print("heap free:",utils.heap_free())
        gc.collect()

        def change_mode(m=ExtreactionMode.qrcode):
            self.mode = m

if __name__ == "__main__":

    cr = CodeReader()
    for i in range(0,200):              # look for barcode
        cr.check_video()
    cr.change_mode(ExtreactionMode.april)
    for i in range(0,200):              # look for april tag
        cr.check_video()	
    cr.change_mode()
    for i in range(0,200):              # look for qr tag
        cr.check_video()
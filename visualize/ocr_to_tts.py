#!/usr/bin/env python
#
# Example of performing OCR using PyTessBaseAPI 
#
# usage sys.argv[0] <file to process> <language> <optional arg to only show ROI> <option amount of blur on the grayscale image converted>
#
import sys
import locale
locale.setlocale(locale.LC_ALL, 'C')
import tesserocr
from tesserocr import PyTessBaseAPI, RIL, PSM
from PIL import Image
import cv2
import pyttsx3

# define compression level on output results
C_LVL=9                                                     # compression level on output png
def main():
    lang='eng'                                              # default use english
    pre_prop_param = 0
    args = sys.argv
    if len(args) > 1:                                       # 1st arg is file name to look for text in
        img_file_name = str(args[1])
    else:
        img_file_name = 'text_to_read.png'
    if len(args) > 2:                                       # specified language
        lang = str(args[2])    
    if len(args) > 3:                                       # if 2nd argument then use the roi rather than image
        show_roi = True
    else:
        show_roi = False
    if len(args) > 4:                                       # amount of median blur for pre-process
        pre_prop_param = int(args[4])
    roi_file = "roi.png"
    # text to speach engine
    engine = pyttsx3.init()
    # Get the available voices
    voices = engine.getProperty('voices')
    # Set the desired voice (index 0 represents the first voice in the list)
    print(voices)
    idx=int(input("select voice number - index 0 represents the first voice in the list"))
    engine.setProperty('voice', voices[idx].id)
    # Set the speech rate (words per minute)
    engine.setProperty('rate', 150)
    # Set the volume (0.0 to 1.0)
    engine.setProperty('volume', 0.7)
    draw_img = cv2.imread(img_file_name)                    # open the photo file as a byte array
    if show_roi == True:
        ROI = cv2.selectROI('Select ROIs', draw_img, fromCenter = False, showCrosshair = False)
        x1 = ROI[0]
        y1 = ROI[1]
        x2 = ROI[2]
        y2 = ROI[3]   
        img_crop = draw_img[int(y1):int(y1+y2),int(x1):int(x1+x2)] 
        if pre_prop_param != 0:                                                                # option to perform a pre-process on the roi image first
            img_crop = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)   
            img_ctop = cv2.medianBlur(img_crop, pre_prop_param)
            img_crop = cv2.threshold(img_crop, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
        cv2.imwrite(roi_file, img_crop)
        image = Image.open(roi_file)                                                           # use the roi instead of the whole file
        draw_img = img_crop    
    else:
        image = Image.open(img_file_name)                                                      # open the entire file  
    font = cv2.FONT_HERSHEY_SIMPLEX

    with PyTessBaseAPI(lang=lang, psm=PSM.SINGLE_BLOCK) as api:
        api.SetImage(image)
        boxes = api.GetComponentImages(RIL.TEXTLINE, True)
        print('Found {} textline image components.'.format(len(boxes)))

        for i, (im, box, _, _) in enumerate(boxes):
            x, y, w, h = box['x'], box['y'], box['w'], box['h']
            api.SetRectangle(x, y, w, h)
            ocrResult = api.GetUTF8Text().strip()
            if ocrResult is not None:
                conf = api.MeanTextConf()
                if conf >= 90:
                    cv2.rectangle(draw_img, (x, y), (x+w, y+h), (255, 150, 0), 1)
                elif conf < 80 and conf > 20:
                    cv2.rectangle(draw_img, (x, y), (x+w, y+h), (0, 150, 255), 1)
                else:
                    cv2.rectangle(draw_img, (x, y), (x+w, y+h), (50, 255, 255), 1)    
                cv2.putText(draw_img, str(conf) + ': ' + ocrResult, (x, y), font, 0.6, (0, 255, 255), 1)

                print ((u"Box[{0}]: x={x}, y={y}, w={w}, h={h}, confidence: {1}, text: {2}").format(i, conf, ocrResult, **box))
                engine.say(ocrResult)
                engine.runAndWait()
        # Write the original image to a new file in PNG format with compression level 9 using cv2.imwrite function
        # cv2.IMWRITE_PNG_COMPRESSION is a flag for the cv2.imwrite function that specifies the compression level of the output image
        # The compression level can range from 0 (no compression) to 9 (highest compression)
        cv2.imwrite('ocr_output.png', draw_img, [cv2.IMWRITE_PNG_COMPRESSION, C_LVL])

if __name__ == "__main__":
    main()

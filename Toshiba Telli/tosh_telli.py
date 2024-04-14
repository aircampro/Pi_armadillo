from teli import *
from PekatVisionSDK import Instance as Pekat

import cv2

initialize()

cameras = getNumOfCameras()
print("Number of cameras: %d" % cameras)

if (cameras):
    # At least one camera
    for i in range(cameras):
        info = getCameraInfo(i)
        print("  Camera id = %d" % i)
        print("  Camera type: %s" % info.camType)
        print("  Manufacturer: %s" % info.manufacturer)
        print("  Model: %s" % info.modelName)
        print("  Serial no: %s" % info.serialNumber)
        print("  User name: %s" % info.userDefinedName)

    # Connect to Pekat
    pekat = Pekat(host = "127.0.0.1", port = 8100, already_running = True)

    z = input("which camera id do you want to choose ? ")
    
    with Camera(int(z)) as cam:
        # How to get and set properties
        # String
        print(cam.getStringValue("DeviceUserID"))
        cam.setStringValue("DeviceUserID", "My camera")
        print(cam.getStringValue("DeviceUserID"))
        # Enum
        print(cam.getEnumStringValue("TestPattern"))
        cam.setEnumStringValue("TestPattern", "ColorBar")
        print(cam.getEnumStringValue("TestPattern"))
        # Int
        print(cam.getIntValue("AcquisitionFrameCount"))
        cam.setIntValue("AcquisitionFrameCount", 2)
        print(cam.getIntValue("AcquisitionFrameCount"))
        # Float
        print(cam.getFloatValue("ExposureTime"))
        cam.setFloatValue("ExposureTime", 500)
        print(cam.getFloatValue("ExposureTime"))
        print(cam.getFloatValue("AntiGlitch"))
        cam.setFloatValue("AntiGlitch", 1)
        print(cam.getFloatValue("AntiGlitch"))
        print(cam.getFloatValue("AntiChattering"))
        cam.setFloatValue("AntiChattering", 1)
        print(cam.getFloatValue("AntiChattering"))
        
        # Start streaming
        cam.startStream()
        # Wait for image 2000ms
        w = cam.waitForImage(2000)
        if (w):
            # Image received
            img = cam.getCurrentImage()
            # Analyze in Pekat for annotated_image
            img_res, _ = pekat.analyze(img, response_type = 'annotated_image')
            # Analyze in Pekat for response - heatmap and context
            img_heatmap, context = pekat.analyze(img, response_type='heatmap')
            # stop pekat instance
            pekat.stop()
            # Save original and result to disk
            cv2.imwrite('pict-orig.png', img)
            cv2.imwrite('pict-annotated.png', img_res)
            cv2.imwrite('pict-heatmap.png', img_heatmap)
        else:
            print("Image not received in time")
        # Stop streaming
        cam.stopStream()

terminate()
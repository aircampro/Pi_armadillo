#
# Optris thermal imaging camera connected on usb
# pythonic interface to the Evocortex libirimager
# ref:- https://github.com/FiloCara/pyOptris?tab=readme-ov-file
#
import pyOptris as optris
import sys
import cv2

# choose relevant library
if not sys.platform.find("linux") == -1:
    LIB_path = "/usr/lib/libirdirectsdk.so"
elif not sys.platform.find("win32") == -1:
    LIB_path = "../irDirectSDK/sdk/x64/libirimager.dll"
else:
    print("unsupported os")
    sys.exit()

# temperature range
TMIN = 300
TMAX = 900

# video thermal text properties
PRINT_THERMAL = 1              # set to 1 to print the thermal result in the video else show without it.
# font
font = cv2.FONT_HERSHEY_SIMPLEX
# org
org = (10, 10)
# fontScale
fontScale = 1
# Blue color in BGR
color = (255, 0, 0)
# Line thickness of 2 px
thickness = 2

if __name__ == "__main__":

    # load library
    optris.load_DLL(LIB_path)
    # USB connection initialisation
    optris.usb_init('config_file.xml')

    w, h = optris.get_thermal_image_size()
    print('{} x {}'.format(w, h))

    # set up the camera 
	if optris.set_shutter_mode(optris.ShutterMode.AUTO) == 0:
        if optris.set_palette.(optris.ColouringPalette.IRON_HI) == 0:
            if optris.set_palette_scale(optris.PaletteScalingMethod.SIGMA3) == 0:
			    if optris.set_temperature_range(TMIN, TMAX) == 0:
                    emissivity, transmissivity, ambientTemperature = 0.9, 0.5, 20.0
                    if optris.set_radiation_parameters(emissivity, transmissivity, ambientTemperature) == 0:
                        print("optris camera set-up correctly")
	
    # Get the thermal frame as (numpy array)
    thermal_frame = optris.get_thermal_image(w, h)
    # Conversion to temperature values are to be performed as follows:
    # t = ((double)data[x] - 1000.0) / 10.0;
    processed_thermal_frame = (thermal_frame - 1000.0) / 10.0 
    print("processed frame : ", processed_thermal_frame)

    while True:
        # Get the palette image (RGB image) and display it
        frame = optris.get_palette_image(w, h)
        if PRINT_THERMAL == 1:
            thermal_frame = optris.get_thermal_image(w, h)
            processed_thermal_frame = (thermal_frame - 1000.0) / 10.0 
            frame = cv2.putText(frame, str(processed_thermal_frame), org, font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.imshow("IR streaming", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        elif cv2.waitKey(1( & 0xFF == ord("f"):
            f = optris.get_focus_motor_position()
            f += 0.1
            if not optris.set_focus_motor_position(f) == 0:
                print("error setting focus")
        elif cv2.waitKey(1( & 0xFF == ord("b"):
            f = optris.get_focus_motor_position()
            f -= 0.1
            if not optris.set_focus_motor_position(f) == 0:
                print("error setting focus")
    optris.terminate()
    cv2.destroyAllWindows()

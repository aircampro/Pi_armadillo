# adapted from the "Getting started" code snippet found at https://github.com/basler/pypylon
# git clone https://github.com/basler/pypylon.git
# cd pypylon
# pip install .
#
# read here ......> https://resonon.gitlab.io/programming-docs/content/overview.html#pika-l-xc2-and-uv
#
from pypylon import pylon

# Pika L Parameters, from table on the Camera Setup and Windowing Page
ROI_WIDTH = 900
ROI_HEIGHT = 600
Y_BINNING = 2

# Device-specific parameters obtained from Resonon Camera Configuration Report
X_OFFSET = 540
Y_OFFSET = 312
A = 0.00010350000229664147
B = 0.9359210133552551
C = 83.2490005493164

# obtain the camera instance from the Pylon API
camera_instance = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
camera_instance.Open()

# Configure the camera window
# We make sure the camera is not binned before configuring the window, then add our desired binning
camera_instance.BinningHorizontal.SetValue(1)
camera_instance.BinningVertical.SetValue(1)
camera_instance.Width.SetValue(ROI_WIDTH)
camera_instance.Height.SetValue(ROI_HEIGHT)
camera_instance.OffsetX.SetValue(X_OFFSET)
camera_instance.OffsetY.SetValue(Y_OFFSET)
camera_instance.BinningVertical.SetValue(Y_BINNING)


def get_wavelength_for_channel(band_number: int) -> float:
    # The wavelength calibration equation is defined in terms of un-binned and un-windowed sensor coordinates.
    # here, we convert the band number of the binned and windowed region to its equivalent un-binned location
    # and apply the calibration equation.
    #
    # The term Y_BINNING / 2.0 - 0.5 is a correction to ensure the wavelength is calculated from the center of the
    # binned region, instead of the edge (using 0-based, C style indexing)
    camera_pixel = Y_OFFSET + band_number * Y_BINNING + Y_BINNING / 2.0 - 0.5
    return A * camera_pixel**2 + B * camera_pixel + C


print('Wavelengths:')
wavelengths = [get_wavelength_for_channel(pixel) for pixel in range(camera_instance.Height.GetValue())]
print(', '.join(str(round(w, 2)) for w in wavelengths))


# Capture 100 frames
FRAME_COUNT = 100
camera_instance.StartGrabbingMax(FRAME_COUNT)
frame_number = 0
while camera_instance.IsGrabbing():
    grab_result = camera_instance.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    if grab_result.GrabSucceeded():
        print (f'Frame {frame_number}')
        frame_number += 1
        if frame_number == FRAME_COUNT:
            # Access the image data for the last frame, as an example
            print("Frame size X (spatial samples): ", grab_result.Width)
            print("Frame size Y (spectral bands): ", grab_result.Height)
            frame = grab_result.Array
            print("Gray value of first pixel: ", frame[0, 0])

    grab_result.Release()
camera_instance.Close()
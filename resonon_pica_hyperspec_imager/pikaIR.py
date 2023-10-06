# Adapted from VimbaPython synchronous grab example, found here:
# https://github.com/alliedvision/VimbaPython/blob/master/Examples/synchronous_grab.py
# 
# # git clone https://github.com/alliedvision/VimbaPython.git
# cd VimbaPython
# pip install .
from vimba import Vimba

# Pika IR Parameters, from table on the Camera Setup and Windowing Page
ROI_WIDTH = 320
ROI_HEIGHT = 168
Y_BINNING = 1

# Device-specific parameters obtained from Resonon Camera Configuration Report
X_OFFSET = 0
Y_OFFSET = 40
A = 0.0007239
B = 4.832482
C = 691.154


def get_wavelength_for_channel(band_number: int) -> float:
    # The wavelength calibration equation is defined in terms of un-binned and un-windowed sensor coordinates.
    # here, we convert the band number of the binned and windowed region to its equivalent un-binned location
    # and apply the calibration equation.
    #
    # The term Y_BINNING / 2.0 - 0.5 is a correction to ensure the wavelength is calculated from the center of the
    # binned region, instead of the edge (using 0-based, C style indexing)
    camera_pixel = Y_OFFSET + band_number * Y_BINNING + Y_BINNING / 2.0 - 0.5
    return A * camera_pixel**2 + B * camera_pixel + C


with Vimba.get_instance() as vimba:
    with vimba.get_all_cameras()[0] as camera_instance:
        camera_instance.BinningHorizontal.set(1)
        camera_instance.BinningVertical.set(1)
        camera_instance.Width.set(ROI_WIDTH)
        camera_instance.Height.set(ROI_HEIGHT)
        camera_instance.OffsetX.set(X_OFFSET)
        camera_instance.OffsetY.set(Y_OFFSET)
        camera_instance.BinningVertical.set(Y_BINNING)

        print('Wavelengths:')
        wavelengths = [get_wavelength_for_channel(pixel) for pixel in range(camera_instance.Height.get())]
        print(', '.join(str(round(w, 2)) for w in wavelengths))

        # Capture 100 frames
        for frame in camera_instance.get_frame_generator(limit=100, timeout_ms=3000):
            print('Got {}'.format(frame), flush=True)

        # Access the image data for the last frame, as an example
        print("Frame size X (spatial samples): ", frame.get_width())
        print("Frame size Y (spectral bands): ", frame.get_height())
        print("Gray value of first pixel: ", frame.as_numpy_ndarray()[0, 0])



Quick search

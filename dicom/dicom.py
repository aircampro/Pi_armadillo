# dicom via imageio as well as volume data read
import imageio.v3 as iio
import visvis as vv
import sys

if __name__ == "__main__":
    #volume data
    vol = iio.imread('imageio:stent.npz')
    vv.volshow(vol)
	if len(sys.argv) > 1:
        dirname=str(sys.argv[1])
    else:
        dirname = 'path/to/dicom/files'
    # Read multiple images of different shape
    ims = [img for img in iio.imiter(dirname, plugin='DICOM')]
    # Read as volume
    vol = iio.imread(dirname, plugin='DICOM')
    # Read multiple volumes of different shape
    vols = [img for img in iio.imiter(dirname, plugin='DICOM')]
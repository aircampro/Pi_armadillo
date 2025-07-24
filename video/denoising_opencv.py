# denoising example for a picture file - could be used to unblur a photo 
#
import numpy as np
import cv2
from matplotlib import pyplot as plt.

# convert color to gray for a given CvMat frame 
def gray_img(img_bgr):
    img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    return img_gray
 
if __name__ == '__main__':

    # specify the mode and file name from the script arguments
    args_len = len(sys.argv)
    if args_len == 1 :
        file_nm = 'die.png'                                # default
        mode = 1
    elif args_len == 2:	
        file_nm = str(argv[1])
        mode = 1
    else:
        file_nm = str(argv[1])
        mode = int(sys.argv[2])

    if file_nm.split(".").find("avi") == 0:                                        # its an avi (video) file 
        cap = cv2.VideoCapture(file_nm)
        # create a list of first 5 frames
        img = [cap.read()[1] for i in xrange(5)]
        # cv2.fastNlMeansDenoisingMulti() - works with image sequence captured in short period of time (grayscale images)
        # cv2.fastNlMeansDenoisingColoredMulti() - same as above, but for color images
        #
        if mode == 1:
            # convert all to grayscale
            gray = [cv2.cvtColor(i, cv2.COLOR_BGR2GRAY) for i in img]
            # convert all to float64
            gray = [np.float64(i) for i in gray]
            # create a noise of variance 25
            noise = np.random.randn(*gray[1].shape)*10
            # Add this noise to images
            noisy = [i+noise for i in gray]
            #  Convert back to uint8
            noisy = [np.uint8(np.clip(i,0,255)) for i in noisy]
            # Denoise 3rd frame considering all the 5 frames
            dst = cv2.fastNlMeansDenoisingMulti(noisy, 2, 5, None, 4, 7, 35)
            plt.subplot(131),plt.imshow(gray[2],'gray')
            plt.subplot(132),plt.imshow(noisy[2],'gray')
            plt.subplot(133),plt.imshow(dst,'gray')
            plt.show()
        elif mode == 2:
            # convert all to float64
            im = [np.float64(i) for i in img]
            # create a noise of variance 25
            noise = np.random.randn(*im[1].shape)*10
            # Add this noise to images
            noisy = [i+noise for i in im]
            #  Convert back to uint8
            noisy = [np.uint8(np.clip(i,0,255)) for i in noisy]
            # Denoise 3rd frame considering all the 5 frames
            dst = cv2.fastNlMeansDenoisingColoredMulti(noisy, 2, 5, None, 4, 7, 35)
            plt.subplot(131),plt.imshow(im[2],'image')
            plt.subplot(132),plt.imshow(noisy[2],'image')
            plt.subplot(133),plt.imshow(dst,'img')        
    else:
        img = cv2.imread(file_nm)
        # cv2.fastNlMeansDenoising() - works with a single grayscale images
        # cv2.fastNlMeansDenoisingColored() - works with a color image.
        #
        if mode == 1:
            img_g = gray_img(img)
            dst = cv2.fastNlMeansDenoising(img_g,None,10,10,7,21)
        elif mode == 2:
            dst = cv2.fastNlMeansDenoisingColored(img,None,10,10,7,21)
        plt.subplot(121),plt.imshow(img)
        plt.subplot(122),plt.imshow(dst)
        plt.show()
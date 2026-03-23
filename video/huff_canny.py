#! /usr/bin/env python3
#
# Edge detect without openCV
#
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser

# huff transdorm
def huff_trans(image_name):
    original_image = plt.imread(image_name)
    if np.issubdtype(original_image.dtype, np.integer):
        original_image = original_image / np.iinfo(original_image.dtype).max
    gray_image = 0.2116 * original_image[:,:,0] + 0.7152 * original_image[:,:,1] + 0.0722 * original_image[:,:,2]
    plt.imshow(gray_image, cmap='gray')
    return gray_image

def _convolve2d(image, kernel):
    shape = (image.shape[0] - kernel.shape[0] + 1, image.shape[1] - kernel.shape[1] + 1) + kernel.shape
    strides = image.strides * 2
    strided_image = np.lib.stride_tricks.as_strided(image, shape, strides)
    return np.einsum('kl,ijkl->ij', kernel, strided_image)

def _convolve2d_multichannel(image, kernel):
    convolved_image = np.empty((image.shape[0] - kernel.shape[0] + 1, image.shape[1] - kernel.shape[1] + 1, image.shape[2]))
    for i in range(image.shape[2]):
        convolved_image[:,:,i] = _convolve2d(image[:,:,i], kernel)
    return convolved_image

def _pad_singlechannel_image(image, kernel_shape, boundary):
    return np.pad(image, ((int(kernel_shape[0] / 2),), (int(kernel_shape[1] / 2),)), boundary)

def _pad_multichannel_image(image, kernel_shape, boundary):
    return  np.pad(image, ((int(kernel_shape[0] / 2),), (int(kernel_shape[1] / 2),), (0,)), boundary)

def convolve2d(image, kernel, boundary='edge'):
    if image.ndim == 2:
        pad_image = _pad_singlechannel_image(image, kernel.shape, boundary) if boundary is not None else image
        return _convolve2d(pad_image, kernel)
    elif image.ndim == 3:
        pad_image = _pad_multichannel_image(image, kernel.shape, boundary) if boundary is not None else image
        return _convolve2d_multichannel(pad_image, kernel)

def create_gaussian_kernel(size=(3, 3), sigma=1):
    center = ((size[0] - 1) / 2, (size[1] - 1) / 2)
    sigma2 = 2 * sigma * sigma
    kernel = np.fromfunction(lambda y, x: np.exp(-((x - center[1]) ** 2 + (y - center[0]) ** 2) / sigma2), size)
    kernel = kernel / np.sum(kernel)
    return kernel 

def _suppress_nonmaximum(array):
    direction = array[9]
    if direction < 0:
        direction += np.pi
    if direction < 0.125 * np.pi:
        v0, v1, v2 = (array[3], array[4], array[5])
    elif direction < 0.375 * np.pi:
        v0, v1, v2 = (array[2], array[4], array[6])
    elif direction < 0.625 * np.pi:
        v0, v1, v2 = (array[1], array[4], array[7])
    elif direction < 0.875 * np.pi:
        v0, v1, v2 = (array[0], array[4], array[8])
    else:
        v0, v1, v2 = (array[3], array[4], array[5])
    return v1 if v1 >= v0 and v1 >= v2 else 0

def suppress_nonmaximum(intensity_image, direction_image):
    pad_image = np.pad(intensity_image, 1, 'edge')
    shape = intensity_image.shape + (3, 3)
    strides = pad_image.strides * 2
    strided_image = np.lib.stride_tricks.as_strided(pad_image, shape, strides).reshape(shape[0], shape[1], 9)
    return np.apply_along_axis(_suppress_nonmaximum, 2, np.dstack((strided_image, direction_image)))

def histeresis_thresholding(image, threshold):
    edge_image = np.where(image >= threshold[1], 1.0, 0.0)
    while True:
        pad_edge_image = np.pad(edge_image, 1, 'edge')
        shape = image.shape + (3, 3)
        strides = pad_edge_image.strides * 2
        neighbour_edge_image = np.lib.stride_tricks.as_strided(pad_edge_image, shape, strides)
        edge_candidate_index = np.where((image >= threshold[0]) & (edge_image != 1.0))

        changed = False
        for index in list(zip(*edge_candidate_index)):
            if np.any(neighbour_edge_image[index] == 1.0):
                edge_image[index] = 1.0
                changed = True
        if not changed:
            break;
    return edge_image

def canny(image, gaussian_size=(3, 3), gaussian_sigma=1, threshold=(0.02, 0.05)):
    gaussian_kernel = create_gaussian_kernel(size=gaussian_size, sigma=gaussian_sigma)
    gaussian_image = convolve2d(gray_image, gaussian_kernel)
    sobel_x_kernel = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    sobel_x_image = convolve2d(gaussian_image, sobel_x_kernel) / 8
    sobel_y_kernel = np.array(([[1, 2, 1], [0, 0, 0], [-1, -2, -1]]))
    sobel_y_image = convolve2d(gaussian_image, sobel_y_kernel) / 8
    sobel_intensity_image = np.sqrt(sobel_x_image ** 2 + sobel_y_image ** 2)
    sobel_direction_image = np.arctan(np.divide(sobel_y_image, sobel_x_image, out=np.zeros_like(sobel_y_image), where=sobel_x_image!=0))
    nonmaximum_suppression_image = suppress_nonmaximum(sobel_intensity_image, sobel_direction_image)
    return histeresis_thresholding(nonmaximum_suppression_image, threshold)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('input_file', help='input file')
    args = parser.parse_args()	
    gray_image = huff_trans(args.input_file)
    edge_image = canny(gray_image, gaussian_size=(3, 3), gaussian_sigma=0.2, threshold=(0.05, 0.2))
    plt.imshow(edge_image, cmap='gray')

import numpy as np
import cv2
import sys

np.random.seed(0)
def add_noise(img, mu=0, sigma=100):
    # Generate pixel x RGB noise
    noise = np.random.normal(mu, sigma, img.shape)
    # Add noise and clip to 8bit range
    noisy_img = img.astype(np.float64) + noise
    noisy_img = np.clip(noisy_img, 0, 255).astype(np.uint8)   
    return noisy_img

if __name__ == "__main__":

    if len(sys.argv) == 2:
        vid_in  = str(sys.argv[1]) 
    else:
        vid_in = 'Input.mp4'   
    cap = cv2.VideoCapture(vid_in)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    # Preparing the output
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out = cv2.VideoWriter('output.mp4',fourcc, fps, (w*2, h))  # The width is set to double because the image before and after noise is added to concat.
    while True:
        # Loading frames sequentially and adding noise
        ret, frame = cap.read()
        if not ret:
            break
        # Add noise to concat the original image
        noisy_frame = add_noise(frame)
        out_frame = cv2.hconcat((frame, noisy_frame))
        out.write(out_frame)
    # Finish the process when everything is done
    cap.release()
    out.release()
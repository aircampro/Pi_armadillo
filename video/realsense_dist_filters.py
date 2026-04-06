# implement filters on the measurement of distance @ a given co-ordinate
#
#
import pyrealsense2 as rs
import numpy as np
import cv2
import time

class Kalman():
    #
    # Reference: http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
    #

    def __init__(self, procvar, measvar, db=False):
        self.ctr = 0
        self.Q = procvar  # process variance
        self.P_k_minus_1 = 0.0  # a posteri error estimate
        self.R = measvar  # estimate of measurement variance, change to see effect
        self.DEBUG = db

    def guess(self, input):

        if self.ctr == 0:
            self.xhat_k_minus_1 = input  # a posteri estimate of x
            self.K = 1.0  # Kalman gain
            self.ctr = self.ctr + 1
            return input
        else:
            # time update
            xhat_k = self.xhat_k_minus_1 # a prior estimate of x, transition matrix is identity, no control
            Phat_k = self.P_k_minus_1 + self.Q # a prior estimate of error
            # measurement update
            self.K = Phat_k / (Phat_k + self.R) # Kalman gain
            estimate = xhat_k + self.K * (input - xhat_k) # a posteri estimate of x
            self.xhat_k_minus_1 = estimate # a posteri estimate of x
            self.P_k_minus_1 = (1 - self.K) * Phat_k # a posteri error estimate
            # error variance and kalman gain should become stable soon
            if self.DEBUG: print("Kalman:","Input",input,"Estimate",int(estimate), "ErrorVar {:.2}".format(self.P_k_minus_1), "KalmanGain {:.2}".format(self.K))
            return estimate
 
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# picsel position for distance measurement
x_pos = 200
y_pos = 200
first = true
# lpf filter co-eff
a = 0.8
try:
    while True:
        frames = pipeline.wait_for_frames()  
        depth_frame = flames.get_depth_frame 

        if not depth_frame:
            continue

        # apply temporal filter
        temporal = rs.temporal_filter()
        depth_frame = temporal.process(depth_frame)
        depth_frame = depth_frame.as_depth_frame()　

        # get distance from point
        distance = depth_frame.get_distance(x_pos, y_pos)

        # Implement LPF Filter on measurement
        # y[i] = a*y[i-1] + (1-a)*x[i]
        if first:
            period = 0.01
            y0 = (1-a) * distance 
            time.sleep(period)
            k = Kalman(0.1, distance)                     # create kalman filter           
            first = false
        else:
            out = a*y0 + ((1-a) * distance )
            y0 = out
            kalm = k.guess(distance)
            print(f" distance = {distance} output from LPF Filter : {out} Kalman {kalm}")
            time.sleep(period)	

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        color_frame = frames.get_color_frame()

        if not color_frame or not depth_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        combined_image = np.hstack((color_image, depth_colormap))
        cv2.imshow('RealSense Software Trigger Capture', combined_image)
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

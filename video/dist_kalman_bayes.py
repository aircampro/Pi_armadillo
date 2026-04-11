# Program Name: dist_kalman_bayes.py
# Overview: High-accuracy Kalman Filter + Bayesian update for noisy sine wave estimation
# Usage: Run in Python environment with required libraries installed
#
# In real mode we estimate the distance measured at the co-ordinate by the realsense camera by kalman and bayes
# In test mode we show the methods on a noise sine wave input.
#

# -------------------- Install Required Libraries --------------------
# !pip install numpy matplotlib scipy

# -------------------- Import Libraries --------------------
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
#import pandas
import pyrealsense2 as rs
import cv2
import time

TEST_IT=True                                                                      # true means use the test data to check prog false means use the camera

# -------------------- Parameters  --------------------
np.random.seed(42)                                                                # Random seed
n_steps = 100                                                                     # Number of time steps
true_freq = 0.2                                                                   # Frequency of sine wave
omega = 2 * np.pi * true_freq                                                     # Angular frequency
noise_std = 0.5                                                                   # Observation noise std
process_var = 1e-6                                                                # Process noise variance
obs_var = noise_std**2                                                            # Observation noise variance
dt = 1.0                                                                          # Time interval
no_samples = 50                                                                   # number of real time measurements to make

# -------------------- Generate Noisy Sine Wave / This is your actual data (e.g. from sensor) --------------------
if TEST_IT == True:
    t = np.arange(n_steps)
    true_signal = np.sin(omega * t)                                                   # test input is True sine wave
    observations = true_signal + np.random.normal(0, noise_std, n_steps)              # Generated example is sine wave plus Noisy observations
else:                                                                                 # read the samples from the camera 
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    # picsel position for distance measurement
    x_pos = 500
    y_pos = 500
    obs = []
    for z in range(0,no_samples):
        try:
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
        except:
            print("error reading distance")
         obs.append(distance)
    #df = pd.array(obs, dtype="float")                                           # if you want to use pandas for cleaning first
    #observations = df.to_numpy(np.float64)
    observations = np.array(obs)
    pipeline.stop()
  
# -------------------- Kalman Filter Initialization --------------------
x_est = np.array([0.0, omega])                                                    # Initial state: [position, velocity]
P = np.eye(2) * 1.0                                                               # Initial covariance matrix

# State transition matrix (harmonic oscillator)
A = np.array([
    [np.cos(omega * dt), np.sin(omega * dt) / omega],
    [-omega * np.sin(omega * dt), np.cos(omega * dt)]
])

H = np.array([[1, 0]])                                                           # Observation model
Q = process_var * np.eye(2)                                                      # Process noise covariance
R = np.array([[obs_var]])                                                        # Observation noise covariance

# -------------------- Storage Arrays  --------------------
kalman_estimates = []                                                            # Kalman estimates
bayes_means = []                                                                 # Bayesian posterior means
bayes_vars = []                                                                  # Bayesian posterior variances

# -------------------- Kalman Filter + Bayesian Estimation Loop --------------------
for z in observations:
    # ---- Prediction Step ----
    x_pred = A @ x_est
    P_pred = A @ P @ A.T + Q

    # ---- Kalman Update Step ----
    y = z - float(H @ x_pred)                                                    # Innovation
    S = float(H @ P_pred @ H.T + R)                                              # Innovation covariance
    K = P_pred @ H.T / S                                                         # Kalman gain
    x_est = x_pred + K.flatten() * y                                             # Updated state estimate
    P = (np.eye(2) - K @ H) @ P_pred                                             # Updated covariance
    kalman_estimates.append(float(x_est[0]))                                     # Store position

    # ---- Bayesian Update Step  ----
    prior_mean = float(x_pred[0])
    prior_var = float(P_pred[0, 0])
    post_var = 1 / (1 / prior_var + 1 / obs_var)                                 # Posterior variance
    post_mean = post_var * (prior_mean / prior_var + z / obs_var)                # Posterior mean

    bayes_means.append(post_mean)
    bayes_vars.append(post_var)

# -------------------- Plot Results --------------------
plt.figure(figsize=(12, 6))
plt.plot(t, true_signal, label='True Sine Wave')
plt.plot(t, observations, label='Noisy Observations', alpha=0.4)
plt.plot(t, kalman_estimates, label='Kalman Filter Estimates')
plt.plot(t, bayes_means, label='Bayesian Posterior Means', linestyle='--')
plt.fill_between(t, np.array(bayes_means) - np.sqrt(bayes_vars), np.array(bayes_means) + np.sqrt(bayes_vars), color='gray', alpha=0.3, label='Bayesian 1σ Interval')
plt.xlabel('Time step')
plt.ylabel('Signal')
plt.title('Kalman + Bayesian Estimation of Noisy Sine Wave / Distance measurement')
plt.legend()
plt.grid(True)
plt.show()
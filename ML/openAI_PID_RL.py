#
# This example is using openAI gym library with either PID or RL doing the action 
#
# apt-get install -y python-numpy python-dev cmake zlib1g-dev libjpeg-dev xvfb libav-tools xorg-dev python-opengl libboost-all-dev libsdl2-dev swig
# $ sudo apt install python3-pip
# $ sudo apt install g++
# $ pip3 install gym tensorflow==1.14.0 stable-baselines
#
import numpy as np
import gym
import sys
import imageio

if len(sys.argv) > 1:
    mode=str(sys.argv[1])
else:
    mode="pid"                                                 # choose the pid mode

if mode == "rlppo":                                            # chose google RL PPO method
    import stable_baselines3
    from stable_baselines3 import PPO
elif mode == "rldqn" or mode == "rldqnlstm":                                          # chose openAI RL DQN method
    from stable_baselines.bench import Monitor
    from stable_baselines.deepq.policies import MlpPolicy
    from stable_baselines.deepq.policies import MlpLstmPolicy
    from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv
    from stable_baselines.common.cmd_util import make_vec_env
    from stable_baselines.common.evaluation import evaluate_policy
    from stable_baselines import DQN
    import os
elif mode == "rlppo2":                                            # chose google RL PPO method
    import stable_baselines3
    from stable_baselines import PPO2

def make_gif(frames, filename, duration=1./30.):
    imageio.mimsave(filename, frames, 'GIF', **{'duration': duration})

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))

if __name__ == "__main__":

    if len(sys.argv) > 2:
        env = gym.make(str(sys.argv[2]))                                                                                     # change to CartPole-v0 Pendulum-v0 
    else:
        env = gym.make('CartPole-v1')                                                                                        # choose gym model 
    if mode == "pid":
        if len(sys.argv) > 3:
            P = int(sys.argv[3])
            I = int(sys.argv[4])
            D = int(sys.argv[5]) 
            if len(sys.argv) > 5:
                desired_state = np.array([int(sys.argv[6]), int(sys.argv[7]), int(sys.argv[8]), int(sys.argv[9])])        # this is the setpoint for the PID when in that mode
                desired_mask = np.array([int(sys.argv[10]), int(sys.argv[11]), int(sys.argv[12]), int(sys.argv[13])])            
            else:
                desired_state = np.array([0, 0, 0, 0])                                                                    # this is the setpoint for the PID when in that mode
                desired_mask = np.array([0, 0, 1, 0])            
        else:            
            P, I, D = 0.1, 0.01, 0.5                                                                                      # default estimated parameters
            desired_state = np.array([0, 0, 0, 0])                                                                        # this is the setpoint for the PID when in that mode
            desired_mask = np.array([0, 0, 1, 0])
    elif mode == "rlppo":
        model = PPO("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=10000)
    elif mode == "rlppo2":
        model = PPO2("MlpPolicy", env, verbose=1)
        model.learn(total_timesteps=10000)
    elif mode == "rldqn":                                                                                                 # https://stable-baselines.readthedocs.io/en/master/modules/policies.html#mlp-policies
        log_dir = './logs/'
        os.makedirs(log_dir, exist_ok=True)
        model_name = 'cartpoleDQN'
        model = DQN(MlpPolicy, env, verbose=1, tensorboard_log=log_dir)
        model.learn(total_timesteps=10000)
    elif mode == "rldqnlstm":                                                                                                 # https://stable-baselines.readthedocs.io/en/master/modules/policies.html#mlp-policies
        log_dir = './logs/'
        os.makedirs(log_dir, exist_ok=True)
        model_name = 'cartpoleDQNLstm'
        model = DQN(MlpLstmPolicy, env, verbose=1, tensorboard_log=log_dir)
        model.learn(total_timesteps=10000)
    frame = []
    env = gym.wrappers.Monitor(env, 'video/',video_callable=lambda episode_id: True,force = True)
    for i_episode in range(20):
        state = env.reset()
        integral = 0
        derivative = 0
        prev_error = 0
        for t in range(1000):
            env.render()
            if mode == "pid":
                error = state - desired_state
                integral += error
                derivative = error - prev_error
                prev_error = error
                pid = np.dot(P * error + I * integral + D * derivative, desired_mask)           # get pid prediction
                action = sigmoid(pid)
                action = np.round(action).astype(np.int32)
            elif mode == "rlppo" or mode == "rldqn" or mode == "rldqnlstm" or mode == "rlppo2":
                action, _states = model.predict(state)                                          # use RL model chosen
            state, reward, done, info = env.step(action)                                        # perfrom action 
            frame.append(state)
            if done:
                print("Episode finished after {} timesteps".format(t+1))
                break
        make_gif(frame, 'out.gif')                                                              # save output as gif

    env.close()

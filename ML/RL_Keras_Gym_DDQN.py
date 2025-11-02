# dqnddqn.py
# example of RL using DQN DDQN 
# coding:utf-8
# [0] Importing required libraries
import sys
import gym                                                                                  # Cartpole execution environment
import numpy as np
import time
import keras
from keras.models import Sequential
from keras.layers import Dense
KV = keras.__version__
# explanation of various optimisation teqniques we default it to Adam as below
#
# ADAM An improved version of RMSprop. One of the most popular algorithms.
# In RMSprop, the modified learning rate is directly multiplied by th#e gradient, but in Adam, the slope also uses an exponential moving average.
# Adagrad MomenumSGD accelerated learning by introducing inertial terms.
# Adagrad and its derivative algorithms basically adaptively change the learning rate (the coefficient multiplied by the gradient) to speed up learning and reduce vibration.
# RMSprop aims to alleviate this problem by "forgetting past gradient information".
# The difference from Adagrad is that it is not simply added in the accumlator calculation, but the moving root mean square (moving RMS). 
# This prevents the learning rate after correction from unilaterally decreasing as learning progresses.
# Adadelta
# Like RMSprop, use exponential moving averages to forget past slopes. The difference is that the adaptive learning rate is calculated by multiplying the 
# exponential moving average of the past step size to reduce the difference in the effect of the learning rate when the slope is large and small
# amsgrad the algorithm is called AMSGrad. By using long-term memory of past gradients, it is possible to improve performance in optimal solutions in shapes that cannot be converged by Adam.
# AdamaxA variant version of Adam.
# Regardless of the theory, if the square slope disappears and becomes the absolute slope value, and the slope is larger than the exponential moving average of the past slope, it 
# seems that the average held is cleared and updated to the latest absolute slope value.
# Nadam Adam was applied to the Nestrov acceleration method used in SGD.
#
if KV <= 2.5:
    from keras.optimizers import Adam,SGD,RMSprop,AdamW,Adadelta,Adagrad,Adamax,Adafactor,Nadam,Ftrl,Lion,Lamb,Muon
else:
    from keras.optimizers import adam_v2
# check the keras model for compatibility with this
#import h5py
#with h5py.File("model.hdf5", "r") as fp:
#    print(fp.attrs.get("keras_version"))   
from keras.utils import plot_model
from collections import deque
from gym import wrappers                                                                     # gym from openAI
from keras import backend as K
import tensorflow as tf

# [1] Defining the loss function
# We use the huber function for the loss function. See https://github.com/jaara/AI-blog/blob/master/CartPole-DQN.py
def huberloss(y_true, y_pred):
    err = y_true - y_pred
    cond = K.abs(err) < 1.0
    L2 = 0.5 * K.square(err)
    L1 = (K.abs(err) - 0.5)
    loss = tf.where(cond, L2, L1)  # Keras does not cover where function in tensorflow :-(
    return K.mean(loss)

# [2] Define the Q function as a class for deep learning networks
class QNetwork:
    def __init__(self, learning_rate=0.01, state_size=4, action_size=2, hidden_size=10, o=0, lf=0):
        self.model = Sequential()
        self.model.add(Dense(hidden_size, activation='relu', input_dim=state_size))
        self.model.add(Dense(hidden_size, activation='relu'))
        self.model.add(Dense(action_size, activation='linear'))
        if KV <= 2.5:
            self.optim=[Adam, SGD, RMSprop, AdamW, Adadelta, Adagrad, Adamax, Adafactor, Nadam, Ftrl, Lion, Lamb, Muon]
        else:
            self.optim=[adam_v2.Adam]
        self.optimizer = self.optim[o](lr=learning_rate)                                         # Adam is the learning method to reduce errors
        self.lossf = [huberloss, 'categorical_crossentropy', 'mse']
        self.model.compile(loss=self.lossf[lf], optimizer=self.optimizer)

    # Weight training
    def replay(self, memory, batch_size, gamma, targetQN):
        inputs = np.zeros((batch_size, 4))
        targets = np.zeros((batch_size, 2))
        mini_batch = memory.sample(batch_size)
 
        for i, (state_b, action_b, reward_b, next_state_b) in enumerate(mini_batch):
            inputs[i:i + 1] = state_b
            target = reward_b
 
            if not (next_state_b == np.zeros(state_b.shape)).all(axis=1):
                # Value calculation (The Q network for action decisions and the Q network for value numbers are separated to accommodate DDQN)
                retmainQs = self.model.predict(next_state_b)[0]
                next_action = np.argmax(retmainQs)                                      # Select the action that returns the greatest reward
                target = reward_b + gamma * targetQN.model.predict(next_state_b)[0][next_action]
                
            targets[i] = self.model.predict(state_b)                                    # Q network output
            targets[i][action_b] = target                                               # teacher signal

        # I received advice from shiglay and modified it to be outside the for statement.
        self.model.fit(inputs, targets, epochs=1, verbose=0)                            # epochs is the number of iterations of the training data, verbose=0 is the setting to not display

# [3]A memory class for Experience Replay and Fixed Target Q-Network
class Memory:
    def __init__(self, max_size=1000):
        self.buffer = deque(maxlen=max_size)

    def add(self, experience):
        self.buffer.append(experience)

    def sample(self, batch_size):
        idx = np.random.choice(np.arange(len(self.buffer)), size=batch_size, replace=False)
        return [self.buffer[ii] for ii in idx]

    def len(self):
        return len(self.buffer)

# [4] Class that determines actions based on the cart state
# Based on your advice, I changed the argument from targetQN to mainQN.
class Actor:
    def get_action(self, state, episode, mainQN):                                          # [C]Return the action at t+1
        # Gradually taking only optimal actions, the ε-greedy method
        epsilon = 0.001 + 0.9 / (1.0+episode)

        if epsilon <= np.random.uniform(0, 1):
            retTargetQs = mainQN.model.predict(state)[0]
            action = np.argmax(retTargetQs)                                                # Select the action that returns the greatest reward

        else:
            action = np.random.choice([0, 1])                                              # Act randomly

        return action

# [5] Start main function----------------------------------------------------
# [5.1] Initialization--------------------------------------------------------
DQN_MODE = 1                                                                               # 1 is DQN, 0 is DDQN
LENDER_MODE = 1                                                                            # 0 is no drawing even after learning, 1 is drawing after learning
GYMM='CartPole-v0'
if len(sys.argv[0]) >= 1:
    GYMM=str(sys.argv[1])
env = gym.make(GYMM)
if len(sys.argv[0]) >= 2:
    DQN_MODE=int(sys.argv[2])
if len(sys.argv[0]) >= 3:
    LENDER_MODE=int(sys.argv[3])
num_episodes = 299                                                                         # Total number of trials
max_number_of_steps = 200                                                                  # Number of steps per trial
goal_average_reward = 195                                                                  # Learning will end if this reward is exceeded
num_consecutive_iterations = 10                                                            # Number of trials for which the average learning completion evaluation is calculated
total_reward_vec = np.zeros(num_consecutive_iterations)                                    # Stores the reward for each trial
gamma = 0.99                                                                               # Discount factor
islearned = 0                                                                              # Flag indicating that learning is complete
isrender = 0                                                                               # Rendering flag
# ---
hidden_size = 16                                                                           # Number of neurons in the hidden layer of the Q-network
learning_rate = 0.00001                                                                    # Learning rate of the Q-network
if len(sys.argv[0]) >= 4:
    learning_rate=float(sys.argv[4])
memory_size = 10000                                                                        # Buffer memory size
batch_size = 32                                                                            # Batch size for updating the Q-network

# [5.2] Q-Network, Memory, and Actor Creation--------------------------------------------------------
mainQN = QNetwork(hidden_size=hidden_size, learning_rate=learning_rate)                    # Main Q-Network
targetQN = QNetwork(hidden_size=hidden_size, learning_rate=learning_rate)                  # Q-Network for calculating value
# plot_model(mainQN.model, to_file='Qnetwork.png', show_shapes=True)                       # Visualizing the Q-Network
memory = Memory(max_size=memory_size)
actor = Actor()

# [5.3]Main routine--------------------------------------------------------
for episode in range(num_episodes):                                                       # Repeat for several trials
    env.reset()                                                                           # cartPole environment initialization
    state, reward, done, _ = env.step(env.action_space.sample())                          # The first step is to take appropriate action
    state = np.reshape(state, [1, 4])                                                     # Convert a list type state into a 1x4 matrix
    episode_reward = 0

    # 
    targetQN.model.set_weights(mainQN.model.get_weights())

    for t in range(max_number_of_steps + 1):  
        if (islearned == 1) and LENDER_MODE:                                             # Draw cartPole after learning is complete
            env.render()
            time.sleep(0.1)
            print(state[0, 0])                                                          

        action = actor.get_action(state, episode, mainQN)                                # Decide the action at time t
        next_state, reward, done, info = env.step(action)                                # Calculate s_{t+1}, _R{t} by performing action a_t
        next_state = np.reshape(next_state, [1, 4])                                      # Convert a list type state into a 1x4 matrix

        # Set and give rewards
        if done:
            next_state = np.zeros(state.shape)                                           # There is no next state s_{t+1}
            if t < 195:
                reward = -1                                                              # Reward clipping, reward is fixed to 1, 0, -1
            else:
                reward = 1                                                               # Reward when you finish 195 steps while standing
        else:
            reward = 0                                                                   # If the robot is standing at each step, a reward is added (reward is set to 1 from the beginning, but this is explicitly stated).

        episode_reward += 1 # reward                                                     # Update total rewards

        memory.add((state, action, reward, next_state))                                  # Update memory
        state = next_state                                                               # Status update

        # Replay: Learn and update the weights of the Q-network
        if (memory.len() > batch_size) and not islearned:
            mainQN.replay(memory, batch_size, gamma, targetQN)

        if DQN_MODE:
        # 2018.06.12
        # shiglay
        # targetQN = mainQN                                                              # Making the Q network for decision making and value calculation the same
        # ↓
            targetQN.model.set_weights(mainQN.model.get_weights())

        # 1. Processing at the end of implementation
        if done:
            total_reward_vec = np.hstack((total_reward_vec[1:], episode_reward))         # record rewards
            print('%d Episode finished after %f time steps / mean %f' % (episode, t + 1, total_reward_vec.mean()))
            break

    # Decide on the end based on the average reward from multiple trials
    if total_reward_vec.mean() >= goal_average_reward:
        print('Episode %d train agent successfuly!' % episode)
        islearned = 1
        if isrender == 0:                                                                 # Update learned flag
            isrender = 1

            # env = wrappers.Monitor(env, './movie/cartpoleDDQN')                         # When saving videos
            # If you want to see how it behaves after just 10 episodes, uncomment below.
            # if episode>10:
            #    if isrender == 0:
            #        env = wrappers.Monitor(env, './movie/cartpole-experiment-1')         # When saving videos
            #        isrender = 1

            #    islearned=1;

# -*- coding: utf-8 -*-
# ref: http://taotao54321.hatenablog.com/entry/2016/11/08/180245
#
import numpy as np
import gym
from gym import wrappers
import sys

# Q learning params
ALPHA = 0.1                                             # learning rate
GAMMA = 0.99                                            # reward discount
LEARNING_COUNT = 100000
TEST_COUNT = 1000

EPS = 0.1

TURN_LIMIT = 100
IS_MONITOR = True

class Agent:
    def __init__(self, env):
        self.env = env
        self.episode_reward = 0.0
        self.q_val = np.zeros(16 * 4).reshape(16, 4).astype(np.float32)

    def learn(self):
        # one episode learning
        state = self.env.reset()
        #self.env.render()
        
        for t in range(TURN_LIMIT):
            if np.random.rand() < EPS:               # explore
                act = self.env.action_space.sample() # random
            else:                                    # exploit
                act = np.argmax(self.q_val[state])
            next_state, reward, done, info = self.env.step(act)
            q_next_max = np.max(self.q_val[next_state])
            # Q <- Q + a(Q' - Q)
            # <=> Q <- (1-a)Q + a(Q')
            self.q_val[state][act] = (1 - ALPHA) * self.q_val[state][act]\
                                 + ALPHA * (reward + GAMMA * q_next_max)
            
            #self.env.render()
            if done:
                return reward
            else:
                state = next_state
        return 0.0 # over limit

    def test(self):
        state = self.env.reset()
        for t in range(TURN_LIMIT):
            act = np.argmax(self.q_val[state])
            next_state, reward, done, info = self.env.step(act)
            if done:
                return reward
            else:
                state = next_state
        return 0.0 # over limit

def main():
    global ALPHA
    global GAMMA
    global EPS
    global TURN_LIMIT
    if len(sys.argv[0]) >= 1:
        ALPHA = float(sys.argv[1])
    if len(sys.argv[0]) >= 2:
        GAMMA = float(sys.argv[2])
    if len(sys.argv[0]) >= 3:
        EPS = float(sys.argv[3]) 
    if len(sys.argv[0]) >= 4:
        TURN_LIMIT = float(sys.argv[4])        
    env = gym.make("FrozenLake-v0")
    if IS_MONITOR:
        env = wrappers.Monitor(env, './FrozenLake-v0', force=True)
    agent = Agent(env)

    print("###### LEARNING #####")
    reward_total = 0.0
    for i in range(LEARNING_COUNT):
        reward_total += agent.learn()
    print("episodes      : {}".format(LEARNING_COUNT))
    print("total reward  : {}".format(reward_total))
    print("average reward: {:.2f}".format(reward_total / LEARNING_COUNT))
    print("Q Value       :{}".format(agent.q_val))

    print("###### TEST #####")
    reward_total = 0.0
    for i in range(TEST_COUNT):
        reward_total += agent.test()
    print("episodes      : {}".format(TEST_COUNT))
    print("total reward  : {}".format(reward_total))
    print("average reward: {:.2f}".format(reward_total / TEST_COUNT))

if __name__ == "__main__":
    main()
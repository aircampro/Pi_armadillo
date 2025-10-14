# !/usr/bin/python
# coding:utf-8
# simple use of simpy
#
import random
import simpy

class Skelton:
    def __init__(self, env):
        self.env = env                                         # pointer to the SimPy environment
        self.count = 0                                         # an example state variable
    def update(self, e):
        self.count += 1                                        # increment the event counter
    def print_state(self):
        print('{} th event occurs at time of {}'.format(self.count, round(self.env.now)))
    def run(self, horizon):
        while True:
            e = simpy.Timeout(self.env, random.expovariate(1))  # create a random Timeout instance
            e.callbacks.append(self.update)                     # register update() method in e's callbacks
            if self.env.now > horizon:                          # if horizen is passed
                break                                           # stop simulation
            else:
                self.print_state()
                self.env.step()                                 # process the next event

env = simpy.Environment()
model = Skelton(env)
model.run(200)
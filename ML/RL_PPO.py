# RL example based on ref:- https://note.com/npaka/n/nb615fd590274
#
# to install follow these instructions
# $ pip install 'stable-baselines3[extra]'
# $ pip install 'gym[classic_control]'
#
import gym
from stable_baselines3 import PPO
import sys

# Preparing the learning environment
env = gym.make('CartPole-v1')

# Preparing the model
DEF_MODEL='MlpPolicy'
SAV_MODEL='your_model_saved'
if len(sys.argv[0]) >= 1:
    DEF_MODEL=str(sys.argv[1])
if len(sys.argv[0]) >= 2:
    SAV_MODEL=str(sys.argv[2])
model = PPO(DEF_MODEL, env, verbose=1)

# Execute learning
model.learn(total_timesteps=128000)

# Performing inference
state = env.reset()
try:
    while True:
        # Drawing the learning environment
        env.render()

        # Model inference
        action, _ = model.predict(state, deterministic=True)

        # 1 step execution
        state, rewards, done, info = env.step(action)

        # Episode completed
        if done:
            break
except KeyboardInterrupt:
    print("premature end requested")

# save model
model.save(SAV_MODEL)
# Open learning environment
env.close()
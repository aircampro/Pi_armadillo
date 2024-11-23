# re-inforcement learning example showing the movement of an object through a grid which is defined by the obstacles
#
import numpy as np
import sys

# Define the environment - a grid with obstacles
grid_size = 8
num_actions = 4
at_start_of_grid = (0, 0)
goal_state = (grid_size - 1, grid_size - 1)
# define your obstacles for example this is what your loading as an online current environment map or reading from lidar 
obstacles = [(1, 2), (2, 1), (3, 3), (4, 3), (5, 4), (6, 7), (7, 0)]
# you cant solve this puzzle obstacles = [(1, 2), (2, 1), (3, 3), (4, 3), (5, 4), (6, 7), (7, 6)] as last two block the ends

# Initialize the Q-table
q_table = np.zeros((grid_size, grid_size, num_actions))

# Set hyperparameters (tuning)
alpha = 0.1                              # Learning rate
gamma = 0.6                              # Discount factor
epsilon = 0.1                            # Exploration vs. exploitation factor

# Define reward and transition functions
def get_reward(state):
    if state == goal_state:
        return 10
    elif state in obstacles:
        return -10
    else:
        return -1

def get_next_state(state, action):
    x, y = state
    if action == 0:                                                 # Move up
        next_state = (max(x - 1, 0), y)
    elif action == 1:                                               # Move down
        next_state = (min(x + 1, grid_size - 1), y)
    elif action == 2:                                               # Move left
        next_state = (x, max(y - 1, 0))
    else:                                                           # Move right
        next_state = (x, min(y + 1, grid_size - 1))
    return next_state
	
# Training loop
num_episodes = 1000
for episode in range(num_episodes):
    state = at_start_of_grid
    done = False

    while not done:
        # Exploration vs. exploitation
        if np.random.uniform(0, 1) < epsilon:
            action = np.random.randint(num_actions)
        else:
            action = np.argmax(q_table[state])

        next_state = get_next_state(state, action)
        reward = get_reward(next_state)

        # Update Q-value
        q_table[state][action] += alpha * (reward + gamma * np.max(q_table[next_state]) - q_table[state][action])

        state = next_state

        if state == goal_state or state in obstacles:
            done = True
			
# Testing the trained agent
#
def test_main(start_state):
    state = start_state
    done = False

    itera = 0
    while not done:
        action = np.argmax(q_table[state])
        next_state = get_next_state(state, action)
        reward = get_reward(next_state)

        state = next_state
        print(f"Current state: {state}")
        itera = itera + 1
               
        if state == goal_state or state in obstacles:
            done = True
        if itera == 2000:
            print("max iterations reached try training it with a different starting point")
            break
            
    print(f"iterations: {itera}")	
    		
if __name__ == "__main__":

    if len(sys.argv) >= 2:
        test_main((int(sys.argv[1]),int(sys.argv[2])))
    else:
        test_main(at_start_of_grid)	
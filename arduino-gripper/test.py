

import numpy as np 
import random 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class SiloEnvironment:

    # initialise the silo, when its done and the team color int. 
    def __init__(self) :
        self.state = np.zeros((3,5), dtype=int)
        self.done = False
        self.team_color = 1 # red ball if we are red. 2 if we are blue 

    # reset the silo condition for a new episode
    def reset(self):
        self.state = np.zeros((3,5), dtype=int) 
        self.done = False 
        return self.state 
    
    # create steps with actions, 
    def step(self, action) :
        layer, col, ball = action 

        #if the current state is 0, place the ball. 
        if self.state[layer, col] == 0: 
            self.state[layer, col] = ball
        
        # each time mua vang is achieved, then plus 10. 
        reward = self.compute_reward() 
        # check if all space in the silos are achieved. 
        self.done = self.check_done()

        # return the current state, the reward and if the game is finished (all silo positions filled) 
        return self.state, reward, self.done
    

    def compute_reward(self):
        reward = 0

        # Check if the conditions for "Mùa Vàng" are met
        for col in range(5): # only 3 of the silos need to be filled by own team color, at 2 of team's ball in the silo, and team ball must be on the top.
            if self.check_silo_full(col) and self.check_team_paddy_rice(col):
                reward += 10  # Reward for achieving the "Mùa Vàng" goal

        return reward

    # check if all the silo is filled 
    def check_if_silo_filled(self, col):
        # return true / false if all all the silo is not 0 which is empty. 
        return np.all(self.state[:, col] != 0) 

    # check if the ball is our team ball color 
    def check_team_paddy_rice(self, col):
        return np.count_nonzero(self.state[:, col] == self.team_color) >= 2 and self.state[2, col] == self.team_color

    # check if all spots in the silos are filled. 
    def check_done(self):
        return np.all(self.state != 0)
    

# Define the Q-learning agent
class QLearningAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.q_table = np.zeros((state_size, action_size))
        self.alpha = 0.1
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01

    # choose the action based, on the current state of the blue ball (2), in the silos. 
    def choose_action(self, state): 
        if np.random.rand() <= self.epsilon:
            return random.choice(range(self.action_size))
        return np.argmax(self.q_table[state])

    # learn from the state, action, and reward, and get the next best action. 
    def learn(self, state, action, reward, next_state): 
        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.gamma * self.q_table[next_state, best_next_action]
        td_error = td_target - self.q_table[state, action]
        self.q_table[state, action] += self.alpha * td_error

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

# ROS node for subscribing to the image topic
class ImageSubscriber:
    def __init__(self, agent, env):
        self.bridge = CvBridge()
        self.agent = agent
        self.env = env
        self.state = self.env.reset()
        self.state_index = int(''.join(map(str, self.state.flatten())), 3)
        self.max_steps = 100
        # initialize the image listener node. 
        rospy.init_node('image_listener', anonymous=True)
        # subscribe to the color topic. 
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        try:
            # from the image, get the value, to 'red ball / blue ball' 
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Process the image to determine the action
        action = self.determine_action(cv_image)
        layer, col, ball = action
        next_state, reward, done = self.env.step((layer, col, ball))

        # Q-learning steps
        next_state_index = int(''.join(map(str, next_state.flatten())), 3)
        self.agent.learn(self.state_index, action, reward, next_state_index)
        self.state_index = next_state_index

        # Print current state and action for illustration
        print(f"Current State:\n{self.state}")
        print(f"Received Image Action: Layer {layer}, Column {col}, Ball {ball}")

        # Update state for next step
        self.state = next_state

        if done:
            print("Goal achieved!")
            self.state = self.env.reset()
            self.state_index = int(''.join(map(str, self.state.flatten())), 3)

    def determine_action(self, cv_image):
        # Dummy function to simulate action determination from image
        # Replace with actual image processing logic
        layer = random.randint(0, 2)
        col = random.randint(0, 4)
        ball = random.randint(1, 2)
        return (layer, col, ball)

# Main training loop
def train():
    env = SiloEnvironment()
    state_size = 3 * 5 * 3 ** (3 * 5)  # Number of possible states
    action_size = 3 * 5 * 2  # There are 3 layers, 5 columns, and 2 possible ball actions (1, 2)
    agent = QLearningAgent(state_size=state_size, action_size=action_size)

    episodes = 1000
    for e in range(episodes):
        state = env.reset()
        state_index = int(''.join(map(str, state.flatten())), 3)
        total_reward = 0

        for time in range(200):
            action = agent.choose_action(state_index)
            layer = action // (5 * 2) % 3
            col = action // 2 % 5
            ball = action % 2 + 1
            next_state, reward, done = env.step((layer, col, ball))
            next_state_index = int(''.join(map(str, next_state.flatten())), 3)
            agent.learn(state_index, action, reward, next_state_index)
            state_index = next_state_index
            total_reward += reward

            if done:
                break

        print(f"Episode {e+1}/{episodes}, Total Reward: {total_reward}")

    # Save the trained Q-table
    with open('q_table.pkl', 'wb') as f:
        pickle.dump(agent.q_table, f)

if __name__ == "__main__":
    # First train the agent
    train()

    # Load the trained Q-table
    with open('q_table.pkl', 'rb') as f:
        q_table = pickle.load(f)

    # Initialize the agent and environment
    state_size = 3 * 5 * 3 ** (3 * 5)
    action_size = 3 * 5 * 2
    agent = QLearningAgent(state_size=state_size, action_size=action_size)
    agent.q_table = q_table

    env = SiloEnvironment()

    # Start the ROS image subscriber node
    image_subscriber = ImageSubscriber(agent, env)

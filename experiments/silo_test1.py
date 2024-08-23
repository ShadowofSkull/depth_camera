import numpy as np
import random
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pickle
from geometry_msgs.msg import Point

class SiloEnvironment:
    def __init__(self):
        self.state = np.zeros((3, 5), dtype=int)
        self.done = False
        self.team_color = 1  # Red ball

    def reset(self):
        self.state = np.zeros((3, 5), dtype=int)
        self.done = False
        return self.state

    def step(self, action):
        layer, col = action
        ball = self.team_color
        if self.state[layer, col] == 0:
            self.state[layer, col] = ball

        reward = self.compute_reward()
        self.done = self.check_done()
        return self.state, reward, self.done

    def compute_reward(self):
        reward = 0
        for col in range(5):
            if self.check_if_silo_filled(col) and self.check_team_paddy_rice(col):
                reward += 10
        return reward

    def check_if_silo_filled(self, col):
        return np.all(self.state[:, col] != 0)

    def check_team_paddy_rice(self, col):
        return np.count_nonzero(self.state[:, col] == self.team_color) >= 2 and self.state[2, col] == self.team_color

    def check_done(self):
        return np.all(self.state != 0)

class QLearningAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.q_table = {}
        self.alpha = 0.1
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01

    def choose_action(self, state):
        if np.random.rand() <= self.epsilon:
            return (random.randint(0, 2), random.randint(0, 4))
        return max(self.q_table.get(state, {}), key=self.q_table.get(state, {}).get, default=(random.randint(0, 2), random.randint(0, 4)))

    def learn(self, state, action, reward, next_state):
        if state not in self.q_table:
            self.q_table[state] = {}
        if action not in self.q_table[state]:
            self.q_table[state][action] = 0

        best_next_action = max(self.q_table.get(next_state, {}).values(), default=0)
        td_target = reward + self.gamma * best_next_action
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.alpha * td_error

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

class ImageSubscriber:
    def __init__(self, agent, env):
        self.bridge = CvBridge()
        self.agent = agent
        self.env = env
        self.state = self.env.reset()
        self.max_steps = 100

        rospy.init_node('image_listener', anonymous=True)
        
        # Publisher for the action (position and ball type)
        self.action_pub = rospy.Publisher('/robot/place_ball', Point, queue_size=10)
        
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        action = self.determine_action(cv_image)
        layer, col = action
        next_state, reward, done = self.env.step((layer, col))

        state_tuple = tuple(map(tuple, self.state))
        next_state_tuple = tuple(map(tuple, next_state))
        self.agent.learn(state_tuple, action, reward, next_state_tuple)

        self.state = next_state
        if done:
            print("Goal achieved!")
            self.state = self.env.reset()

        # Publish the action
        self.publish_action(action)

    def determine_action(self, cv_image):
        layer = random.randint(0, 2)
        col = random.randint(0, 4)
        return (layer, col)

    def publish_action(self, action):
        layer, col = action
        position_msg = Point()
        position_msg.x = layer
        position_msg.y = col
        position_msg.z = self.env.team_color  # Red ball
        self.action_pub.publish(position_msg)

def train():
    env = SiloEnvironment()
    state_size = 3 * 5
    action_size = 3 * 5
    agent = QLearningAgent(state_size=state_size, action_size=action_size)

    episodes = 1000
    for e in range(episodes):
        state = env.reset()
        total_reward = 0

        for time in range(200):
            state_tuple = tuple(map(tuple, state))
            action = agent.choose_action(state_tuple)
            next_state, reward, done = env.step(action)
            next_state_tuple = tuple(map(tuple, next_state))
            agent.learn(state_tuple, action, reward, next_state_tuple)
            state = next_state
            total_reward += reward

            if done:
                break

        print(f"Episode {e+1}/{episodes}, Total Reward: {total_reward}")

    with open('q_table.pkl', 'wb') as f:
        pickle.dump(agent.q_table, f)

if __name__ == "__main__":
    train()
    
    with open('q_table.pkl', 'rb') as f:
        q_table = pickle.load(f)

    env = SiloEnvironment()
    agent = QLearningAgent(state_size=15, action_size=15)
    agent.q_table = q_table

    image_subscriber = ImageSubscriber(agent, env)


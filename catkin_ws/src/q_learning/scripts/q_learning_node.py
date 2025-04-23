#!/usr/bin/env python3
import rospy
import random
import numpy as np
from std_msgs.msg import Int32, String

class QLearningAgent:
    def __init__(self):
        #=== Node init ===
        rospy.init_node("q_learning_node") # Initialize new ROS node: q_learning_agent

        #=== Params ===
        self.alpha = 0.1 # Learning Rates/Step size
        self.gamma = 0.9 # How much feature rewards are valued (closer to 0 immediate reward, closer to 1 delayed gratification)
        self.epsilon = 1.0 # Controls randomness of decisions(exploration vs sticking to what looks best always)
        self.epsilon_decay = 0.995 # Rate of randomness decrease over time
        self.min_epsilon = 0.05 # minimum randomness value

        #=== State/Action vars ===
        self.num_states = 16 # 16 states with our input of 0000 -> 1111 binary numbers 0-15
        self.actions = ["forward", "left", "right", "backward", "stop"] # Possible actions in relations to inputs
        self.num_actions = len(self.actions) # Count of actiosn 

        #=== Q-Learning Vars ===
        self.q_table = np.zeros((self.num_states, self.num_actions)) # rewards for doing actions in certain states for now makes a 16x5 matrix with an action, reward, and input for each cell
        self.current_state = 0 # initialize starting state
        self.previous_state = None # What rover detected last step
        self.previous_action = None # What rover did last step

        #=== ROS Setup of sub and pub ===
        rospy.Subscriber("/rl_state", Int32, self.state_change_handler) # Get state values from rl_state
        self.action_pub = rospy.Publisher("/q_action", String, queue_size=10) # Setup publisher to send actions out to q_action

        self.rate = rospy.Rate(2) # run rate of node 2Hz
        rospy.loginfo("Q-Learning Node Started...")
        self.run() # Run ros node

    def choose_action(self, state):
        if random.uniform(0, 1) < self.epsilon: # Pick a random a random action
            action_idx = random.randint(0, self.num_actions - 1) # Get random action index
        else:
            action_idx = np.argmax(self.q_table[state]) # Pick action with highest reward
        return action_idx
    
    def get_reward(self, state, action_idx):
        bits = list(f"{state:04b}") # turn state int into binary list of numbers
        thermal = int(bits[3]) # Get thermal bit
        obstacles = any(int(b) for b in bits[0:3]) # Get all obstacles states

        if thermal: # Punishment of -10
            return -10
        if obstacles: # Punishment of -10
            return -10
        if self.actions[action_idx] == "stop": # Punishment of -10
            return -5
        return 20 # Reward for safe/good actions
    
    def update_q_table(self, reward, new_state):
        """
        Bellman equation: Q(S,a) = Q(S,a) + a(r + y * max_a * Q'(S', a') - Q(S, a))
        OR
        updated_q = old_q + alpha * (reward + gamma * future_q - old_q)
        """
        old_q = self.q_table[self.previous_state, self.previous_action] # Determine old q: Q of previous action
        future_q = np.max(self.q_table[new_state]) # Determine next q: Q of next action(max of table with state value) 
        # DO the thing that gives the best reward
        updated_q = old_q + self.alpha * (reward + self.gamma * future_q - old_q) # Determine Q with Bellman equation
        self.q_table[self.prev_state, self.prev_action] = updated_q # Update Q table state space
    
    def state_change_handler(self, msg):
        self.current_state = msg.data # Save current state sent over from rl_state
    
    def run(self):
        while not rospy.is_shutdown(): # While ros is running run Q-Learning node
            action_idx = self.choose_action(self.current_state) # Get action index
            action = self.actions[action_idx] # Get actual action
            self.action_pub.publish(action) # Publish for q_action node to take action

            if self.previous_state is not None: # If there is a previous state compute reward and update the table
                reward = self.get_reward(self.current_state, action_idx) # get reward
                self.update_q_table(reward, self.current_state) # Update q_table

                bits = list(f"{self.current_state:04b}") # turn int into list of binary numbers
                if int(bits[3]) == 1 or not any(int(b) for b in bits[0:3]): # If in thermal state or obstacle state start training loop over with lower epsilon value
                    rospy.loginfo("Episode finished — resetting")
                    self.epsilon = max(self.min_epsilon, self.epsilon * self.epsilon_decay)
                    self.prev_state = None
                    self.prev_action = None
                    continue

            #=== Persist state and action for next loop
            self.prev_state = self.current_state
            self.prev_action = action_idx
            
            self.rate.sleep() # Sleep to run a 2Hz

if __name__ == "__main__": # Create node class instance and run till exception occurrs
    try:
        QLearningAgent()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
import random
import numpy as np
from std_msgs.msg import Int64, String

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
        self.num_states = 64 # 64 states with our input of 0000 -> 111111 binary numbers 0-63
        self.actions = ["forward", "left", "right", "backward", "stop"] # Possible actions in relations to inputs
        self.num_actions = len(self.actions) # Count of actions
        self.successful = False
        self.episode_reward = 0
        self.success_threshold = 100 # Success/endstate
        self.thermal_count = 0 # Track time in thermal zones
        self.thermal_threshold = 5 # Max time in thermal zone to restart
        self.non_thermal_count = 0 # Track time in safe zones
        #=== Q-Learning Vars ===
        self.q_table = np.zeros((self.num_states, self.num_actions)) # rewards for doing actions in certain states for now makes a 16x5 matrix with an action, reward, and input for each cell
        self.current_state = 0 # initialize starting state
        self.previous_state = None # What rover detected last step
        self.previous_action = None # What rover did last step

        #=== ROS Setup of sub and pub ===
        rospy.Subscriber("/rl_state", Int64, self.state_change_handler) # Get state values from rl_state
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
        """
        Determine rewards depending on rover states below:
        + 50 for stopping in optimal thermal zone
        + 20 for reaching open, safe zone
        + 10 for stopping in a non-thermal but sub optimal zone
        + 5 for good forward actions
        - 10 for collision(simulated)
        - 10 for stopping in thermal 
        - 5 for thermal hazards
        """
        bits = list(f"{state:06b}") # turn state int into binary list of numbers
        # Get obstacles from state dict
        front_obstacle = int(bits[0])
        left_obstacle = int(bits[1])
        right_obstacle = int(bits[2])
        rear_obstacle = int(bits[3])

        # Get thermal from state dict
        thermal = int(bits[4]) # Get thermal bit
        optimal_thermal = int(bits[5])
        reward = 0

        if optimal_thermal and self.actions[action_idx] == "stop":
            reward += 50
            rospy.loginfo("🚀 OPTIMAL ZONE — Mission Success! +50")

        elif not front_obstacle and not rear_obstacle and not left_obstacle and not right_obstacle and not thermal and self.actions[action_idx] == "stop":
            reward += 20
            rospy.loginfo("🟢 SAFE ZONE FOUND: 20")

        elif not thermal and self.non_thermal_count > 2 and self.actions[action_idx] == "stop":
            reward += 10
            rospy.loginfo("🟢 SEMI-SAFE ZONE FOUND: 10")
        
        elif self.actions[action_idx] == "forward" and not front_obstacle and not thermal:
            rospy.loginfo("🟢 ONWARD: 5")
            reward += 5

        if front_obstacle and self.actions[action_idx] == "forward":
            reward -= 10
            rospy.loginfo("💥 Front Collision -10")
        if rear_obstacle and self.actions[action_idx] == "backward":
            reward -= 10
            rospy.loginfo("💥 Rear Collision -10")
        if thermal:
            reward -= 5
            rospy.loginfo("🔥 Thermal Hazard -5")
        if thermal and self.actions[action_idx] == "stop":
            reward -= 10
            rospy.loginfo("🛑 Stopped in Thermal Zone -10")

        #Consistent obstacle penalties
        obstacle_penalties = 0
        if front_obstacle:
            obstacle_penalties -= 2
        if left_obstacle:
            obstacle_penalties -= 2
        if right_obstacle:
            obstacle_penalties -= 2
        if rear_obstacle:
            obstacle_penalties -= 2

        reward += obstacle_penalties # Add in penalties for obstacles

        rospy.loginfo(f"REWARD: {reward}")
        return reward # Reward for safe/good actions

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
        self.q_table[self.previous_state, self.previous_action] = updated_q # Update Q table state space

    def state_change_handler(self, msg):
        self.current_state = msg.data # Save current state sent over from rl_state
    
    def reset_episode(self):
        self.epsilon = max(self.min_epsilon, self.epsilon * self.epsilon_decay)
        self.previous_state = None
        self.previous_action = None
        self.episode_reward = 0
        self.thermal_count = 0

    def run(self):
        while not rospy.is_shutdown() and not self.successful:# While ros is running run Q-Learning node
            action_idx = self.choose_action(self.current_state) # Get action index
            action = self.actions[action_idx] # Get actual action
            self.action_pub.publish(action) # Publish for q_action node to take action

            if self.previous_state is not None: # If there is a previous state compute reward and update the table
                reward = self.get_reward(self.current_state, action_idx) # get reward
                self.update_q_table(reward, self.current_state) # Update q_table

                bits = list(f"{self.current_state:06b}") # turn int into list of binary numbers
                thermal = int(bits[4])
                optimal_thermal = int(bits[5])

                self.episode_reward += reward

                if thermal:
                    self.thermal_count += 1
                else:
                    self.thermal_count = 0

                # === Failure States ===
                if self.thermal_count > self.thermal_threshold: # If in thermal state or obstacle state start training loop over with lower epsilon value
                    rospy.loginfo(f"Episode max thermal hazard — resetting: {self.episode_reward}")
                    self.reset_episode()
                    continue
                if self.episode_reward < -20: # Reset if stuck in loop 
                    rospy.loginfo("🔁 Stuck in failure loop — resetting.")
                    self.reset_episode()
                    continue

                # === Check if Goal Achieved ===
                if self.episode_reward >= self.success_threshold and optimal_thermal == 1:
                    rospy.loginfo(f"✅ Goal achieved! Reward: {self.episode_reward}. Stopping rover.")
                    self.action_pub.publish("stop")
                    self.successful = True
                    break  # Optionally shutdown here

            #=== Persist state and action for next loop
            self.previous_state = self.current_state
            self.previous_action = action_idx

            self.rate.sleep() # Sleep to run a 2Hz

if __name__ == "__main__": # Create node class instance and run till exception occurrs
    try:
        QLearningAgent()
    except rospy.ROSInterruptException:
        pass
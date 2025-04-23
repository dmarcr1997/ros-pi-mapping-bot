#! /usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan

class RLStateNode():
    def __init__(self):
        #=== Node Initializer ===
        rospy.init_node("rl_state_node") # Create new rl_state node
        rospy.Subscriber("/scan", LaserScan, self.lidar_handler) # subscribe to LaserScan data from lidar
        rospy.Subscriber("/ir_sensor", Bool, self.ir_handler) # subscribe to ir_sensor data
        self.state_pub = rospy.Publisher("/rl_state", String, queue_size=10) # Create publisher for rl_state data for rviz/q-learning node

        # === Sensor state placeholders ===
        self.obstacle_front = 0
        self.obstacle_left = 0
        self.obstacle_right = 0
        self.thermal_hazard = 0

        self.rate = rospy.Rate(10)  # 10 Hz
        self.run()
    
    def lidar_handler(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:]     # 0 ± ~10°
        left_ranges = msg.ranges[80:100]                       # ~90°
        right_ranges = msg.ranges[260:280]                     # ~270°

        max_threshold = 0.3 # max value for obstacle detection
        min_threshold = 0.05 # max threshold for obstacle detection
        self.obstacle_front = int(any(r < max_threshold and r > min_threshold for r in front_ranges)) # Get front detection for valid readings between 0.05 and 0.3
        self.obstacle_left = int(any(r < max_threshold and r > min_threshold for r in left_ranges)) # Get left detection for valid readings between 0.05 and 0.3
        self.obstacle_right = int(any(r < max_threshold and r > min_threshold for r in right_ranges)) # Get right detection for valid readings between 0.05 and 0.3

    def ir_handler(self, msg):
        self.thermal_hazard = int(msg.data) # Save thermal_hazard as 0 : 1
    
    def run(self):
        while not rospy.is_shutdown():
            state_bits = [
                self.obstacle_front,
                self.obstacle_right,
                self.obstacle_left,
                self.thermal_hazard
            ] # create obstacle array to build binary state 

            # 0-15 value based on 4 bit binary state number
            state_id = int("".join(
                    map(str, state_bits) # Turn numbers into string
                ), # join string together
            2) # set as base 2 int 

            self.state_pub.publish(state_id) # Publish state info 
            rospy.loginfo(f"RL State: {state_bits} → {state_id}") # DEBUGGING LOGS

            self.rate.sleep() # keeps node at consistent 10 Hz run rate. 10 updates / second

if __name__ == "__main__":
    try:
        RLStateNode() # Create new RL State node
    except rospy.ROSInterruptException:
        pass # Fail silently on interrupt or node shutdown
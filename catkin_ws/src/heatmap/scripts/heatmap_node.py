#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
import numpy as np

class HeatMapNode:
    def __init__(self):

        #=== ROS Node and Pub/Sub Setup
        rospy.init_node("heatmap_node")

        # Subscribers
        rospy.Subscriber("/q_action", String, self.action_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/ir_sensor", Bool, self.ir_callback)

        # Publishers
        self.heatmap_pub = rospy.Publisher("/heatmap", OccupancyGrid, queue_size=10)

        # === Map Setup and flags ====
        self.map_size = 50  # 50x50 grid
        self.grid = np.zeros((self.map_size, self.map_size))  # 0 = unknown, higher = hazard
        self.pos = [self.map_size // 2, self.map_size // 2]  # Start Robot in middle of map
        self.heading = 0  # 0 = up, 90 = left, 180 = down, 270 = right

        # Sensor flags
        self.thermal_hazard = False
        self.front_obstacle = False

        self.rate = rospy.Rate(2)  # 2Hz node run rate
        rospy.loginfo("Heatmap Node Initialized...")
        self.run()

    def lidar_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:] # Get Front Ranges from lidar
        back_ranges = msg.ranges[170:190] # Get back Ranges from lidar
        max_threshold = 0.3 # Detection max distance
        min_threshold = 0.05 # Detection min distance
        self.front_obstacle = any(r < max_threshold and r > min_threshold for r in front_ranges) # Check for any front obstacles in scan
        self.back_obstacle = any(r < max_threshold and r > min_threshold for r in back_ranges) # Check for any back obstacles in scan
    def ir_callback(self, msg):
        self.thermal_hazard = msg.data # Save thermal data

    def action_callback(self, msg):
        action = msg.data.lower() # Get action from q_action

        if action == "forward":
            if self.heading == 0: # Global forward
                self.pos[1] += 1 # Move y pos +
            elif self.heading == 90: # Global left
                self.pos[0] -= 1 # Move x pos -
            elif self.heading == 180: # Global backward
                self.pos[1] -= 1 # Move y pos -
            elif self.heading == 270: # Global right
                self.pos[0] += 1 # Move x pos +
        elif action == "backward":
            if self.heading == 0: # Global forward
                self.pos[1] -= 1 # Move y pos -
            elif self.heading == 90: # Global left
                self.pos[0] += 1 # Move x pos +
            elif self.heading == 180: # Global backward
                self.pos[1] += 1 # Move y pos +
            elif self.heading == 270: # Global right
                self.pos[0] -= 1 # Move x pos -
        elif action == "left": # Handle left turn
            self.heading = (self.heading + 90) % 360 # psuedo update heading based on left spin
        elif action == "right": # Handle right turn
            self.heading = (self.heading - 90) % 360 # psuedo update heading based on right spin 

        # Clamp position to grid
        self.pos[0] = max(0, min(self.map_size - 1, self.pos[0])) # clamp x pos to 0 -> map_size
        self.pos[1] = max(0, min(self.map_size - 1, self.pos[1])) # clamp y pos to 0 -> map_size

    def update_grid(self):
        x, y = self.pos # Get current x,y position

        if self.thermal_hazard:
            self.grid[y, x] += 5  # Mark thermal hazard hotter heatmap value
        if self.front_obstacle:
            self.grid[y, x] += 3  # Mark front obstacle moderately hot heatmap value
        if self.back_obstacle:    # Mark back obstacle moderately hot heatmap value
            self.grid[y, x] += 3
        if not self.thermal_hazard and not self.front_obstacle:
            self.grid[y, x] -= 1  # Safe zone cool heatmap value

        # Clamp heat values
        self.grid[y, x] = max(0, min(10, self.grid[y, x]))

    def publish_grid(self):
        occ_grid = OccupancyGrid # Create occupancy grid for node message(rviz visualization)
        #=== Grid Header and Metadata ===
        occ_grid.header.stamp = rospy.Time.now() # time stamp now
        occ_grid.header.frame_id = "map" # Set map as frame for rviz
        occ_grid.info.resolution = 0.1 # 10cm resolution
        occ_grid.info.width = self.map_size
        occ_grid.info.height = self.map_size

        #=== Grid position/origin ===
        occ_grid.info.origin = Pose()
        occ_grid.info.origin.position = Point(-self.map_size * 0.05, -self.map_size * 0.05, 0) # Set origin at close to 0, 0, 0
        occ_grid.info.origin.orientation = Quaternion(0, 0, 0, 1)

        #=== Flatten grid to list ===
        flat_grid = (self.grid.flatten() * 10).clip(0, 100).astype(np.int8).tolist() # scale heatmap values between 0 and 100 and flatten to 1D list

        occ_grid.data = flat_grid
        self.heatmap_pub.publish(occ_grid) # PUblish data for rviz to consume

    def run(self):
        while not rospy.is_shutdown():
            self.update_grid()
            self.publish_grid()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        HeatMapNode()
    except rospy.ROSInterruptException:
        pass

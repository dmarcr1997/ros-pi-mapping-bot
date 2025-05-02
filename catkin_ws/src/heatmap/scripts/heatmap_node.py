#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import LaserScan
import numpy as np
from flask import Flask, jsonify
from threading import Thread

heatmap_node = None
@app.route('/heatmap')
def get_heatmap():
    if heatmap_node is not None:
        return jsonify({
            "grid": heatmap_node.grid.tolist()
        })
    else:
        return jsonify({"error": "heatmap_node is not running..."})

def start_flask():
    app.run(host="0.0.0.0", port=5000)

class HeatMapNode:
    def __init__(self):

        #=== ROS Node and Pub/Sub Setup
        rospy.init_node("heatmap_node")

        # Subscribers
        rospy.Subscriber("/q_action", String, self.action_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/thermal", Float32, self.handle_thermal_value)
        # Publishers
        self.heatmap_pub = rospy.Publisher("/heatmap", OccupancyGrid, queue_size=10)

        # === Map vars and flags ====
        self.map_size = 50  # 50x50 grid
        self.pos = [self.map_size // 2, self.map_size // 2]  # Start Robot in middle of map
        self.heading = 0  # 0 = up, 90 = left, 180 = down, 270 = right
        
        # === MAPS ===
        self.thermal_map = np.zeros((self.map_size, self.map_size)) # Track Thermal values
        self.obstacle_map = np.zeros((self.map_size, self.map_size)) # Track obstacles
        self.grid = np.zeros((self.map_size, self.map_size))  # 0 = unknown, higher = hazard

        # Sensor flags
        self.thermal_value = None
        self.front_obstacle = False

        self.rate = rospy.Rate(2)  # 2Hz node run rate
        rospy.loginfo("Heatmap Node Initialized...")

    def lidar_callback(self, msg):
        front_ranges = msg.ranges[0:10] + msg.ranges[-10:] # Get Front Ranges from lidar
        back_ranges = msg.ranges[170:190] # Get back Ranges from lidar
        max_threshold = 0.3 # Detection max distance
        min_threshold = 0.05 # Detection min distance
        self.front_obstacle = any(r < max_threshold and r > min_threshold for r in front_ranges) # Check for any front obstacles in scan
        self.back_obstacle = any(r < max_threshold and r > min_threshold for r in back_ranges) # Check for any back obstacles in scan
    
    def handle_thermal_value(self, msg):
        self.thermal_value = msg.data # Save thermal data

    def normalize_ir_thermal_values(self):
        if self.thermal_value is None:
            return 0.0
        return np.clip((self.thermal_value - 10000) / (36000 - 10000), 0.0, 1.0)

    def move_virtual_bot(self, direction):
        dx, dy = 0, 0
        if direction == "forward":
            if self.heading == 0: # Global forward
                dy += 1
            elif self.heading == 90: # Global left
                dx -= 1 # Move x pos -
            elif self.heading == 180: # Global backward
                dy -= 1 # Move y pos -
            elif self.heading == 270: # Global right
                dx += 1 # Move x pos +
        elif direction == "backward":
            if self.heading == 0: # Global forward
                dy -= 1 # Move y pos -
            elif self.heading == 90: # Global left
                dx += 1 # Move x pos +
            elif self.heading == 180: # Global backward
                dy += 1 # Move y pos +
            elif self.heading == 270: # Global right
                dx -= 1 # Move x pos -
        self.pos[0] = max(0, min(self.map_size - 1, self.pos[0] + dx))
        self.pos[1] = max(0, min(self.map_size - 1, self.pos[1] + dy))

    def action_callback(self, msg):
        action = msg.data.lower() # Get action from q_action
        if action in ["forward", "backward"]:
            self.move_virtual_bot(action)
        elif action == "left": # Handle left turn
            self.heading = (self.heading + 90) % 360 # psuedo update heading based on left spin
        elif action == "right": # Handle right turn
            self.heading = (self.heading - 90) % 360 # psuedo update heading based on right spin 

    def update_grid(self):
        x, y = self.pos # Get current x,y position. y = row, x = col

        # === Thermal Update ===
        thermal_normalized = self.normalize_ir_thermal_values()
        self.thermal_map[y, x] = thermal_normalized

        # === Obstacle Update ==
        if self.front_obstacle or self.back_obstacle:
            self.obstacle_map[y, x] = 1
        else:
            self.obstacle_map[y, x] = max(self.obstacle_map[y, x] - 0.1, 0)

        # === Combine Maps to grid ===
        self.grid = (
            self.thermal_map * 0.7 + self.obstacle_map * 0.3
        )
        self.grid = np.clip(self.grid, 0, 1.0) # Normalize all grid values

    def run(self):
        while not rospy.is_shutdown():
            self.update_grid()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        heatmap_node = HeatMapNode()
        flask_thread = Thread(target=start_flask)
        flask_thread.daemon = True
        flask_thread.start()
        heatmap_node.run()
    except rospy.ROSInterruptException:
        pass

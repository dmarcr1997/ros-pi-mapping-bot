#!/usr/bin/env python3
import rospy # Core ros packages
from sensor_msgs.msg import LaserScan # ROS message type for LiDAR data
from nav_msgs.msg import OccupancyGrid # 2D grid for SLAM
import numpy as np
import math

class LaserScanMapper:
    def __init__(self):
        rospy.init_node("laser_scan_mapper") # Initialize ROS node with name = "laser_scan_mapper"
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback) # subscribe to the /scan topic to get LiDAR laser scan data from yd x4-pro node
        self.map_pub = rospy.Publisher("/slam_map", OccupancyGrid, queue_size=1) # create a publisher for 2d occupancy grid
        self.grid_res = 0.05 # set grid cells to 5cmx5cm
        self.grid_size = 512 # grid dimensions 512 x 512
        self.origin = (self.grid_size // 2, self.grid_size // 2) # use center of grid as origin
        self.occ_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8) # set grid cells to 0
    
    def scan_callback(self, msg):
        self.occ_grid[:] = 0 # clear the grid(change in future resets everytime for now)
        angle = msg.angle_min # get first angle in scan
        for r in msg.ranges: # iterate through range measurements in the scan
            if msg.range_min < r < msg.range_max: # check for valid range readings
                x = r * math.cos(angle) # to polar x = r * cos(theta)
                y = r * math.sin(angle) # to polar y = r * sin(theta)
                mx = int(self.origin[0] + x / self.grid_res) # world x coordinate based on grid origin point for x
                my = int(self.origin[1] + y / self.grid_res) # world y coordinate based on grid origin point for y
                if 0 <= mx < self.grid_size and 0 <= my < self.grid_size: # if valid world coordinate add to occupancy grid
                    self.occ_grid[mx, my] = 100
            angle += msg.angle_increment # move to next angle
        self.publish_map(msg) # publish updated map

    def publish_map(self, scan_msg):
        grid = OccupancyGrid() # create new occupancy grid
        grid.header.stamp = rospy.Time.now() # Rviz timestamp for visualization
        grid.header.frame_id = "laser_frame" # rviz frame_id 

        # Map metadata
        grid.info.resolution = self.grid_res
        grid.info.width = self.grid_size
        grid.info.height = self.grid_size

        # set origin to center of map for rviz robot
        grid.info.origin.position.x = -self.grid_res * self.origin[0]
        grid.info.origin.position.y = -self.grid_res * self.origin[1]

        #flatten the 2d grid into a 1d list to send over ROS to rviz
        grid.data = self.occ_grid.flatten().tolist()

        #publish to map
        self.map_pub.publish(grid)

if __name__ == "__main__":
    LaserScanMapper() # Create class instance of mapper
    rospy.spin() # run node as long as process is running
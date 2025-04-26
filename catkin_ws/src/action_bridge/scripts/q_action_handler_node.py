#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class QActionHandler:
    def __init__(self):
        #=== ROS Node & Pub/Sub setup ===
        rospy.init_node("q_action_handler_node") # Initialize q_action handler ROS node
        rospy.Subscriber("/q_action", String, self.q_action_callback) # subscribe to q_action topic from q_learning node, send string to action_callback function
        self.motor_pub = rospy.Publisher("/motor_control", String, queue_size=10) # setup publisher to motor_control node to send commands to motor controller
        self.command_duration = rospy.get_param("~command_duration", 0.4) # Set motor command duration to 0.4 seconds

        #=== Param ===
        self.actions = ["forward", "left", "right", "backward", "stop"]
        #=== Start node ===
        rospy.loginfo("Q-Action Handler Started...")
        rospy.spin() # Keeps node running. Used in place of while loop waiting for shutdown. makes the node event driven so it only runs when it needs to
    
    def q_action_callback(self, msg):
        action = msg.data.lower()
        rospy.loginfo(f"Sending over Q-action:{action}")

        #=== send command to motor node
        if action in self.actions: #Check for valid action
            self.motor_pub.publish(action) # send action to motor_control node
        else:
            rospy.logward(f"Error: Invalid action:{action}")

if __init__ == "__main__": # File entry point
    try:
        QActionHandler() # Create and start new action handler node 
    except rospy.ROSInterruptException: # Stop on ROS interrupt or error
        pass
#! /usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

#=== GPIO PIN SETUP ===
IR_PIN = 5

GPIO.setmode(GPIO.BCM) # Pin reference set to GPIO # Instead of physical pin number
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set IR pin as input and enable internal pull-up resistor for signal stability

#=== ROS NODE INIT ====
rospy.init_node("ir_sensor_node") # Create new node ir_sensor_node
pub = rospy.Publisher("/ir_sensor", Bool, queue_size=10) # Create data publisher for IR_SENSOR node that returns a Bool depending on detection
rate = rospy.Rate(10)  # 10 Hz

rospy.loginfo("IR sensor node started...")

try:
    while not rospy.is_shutdown():
        ir_value = GPIO.input(IR_PIN)
        detected = ir_value == 0 # 0 or LOW Value means detection
        pub.publish(detected)
        rate.sleep()
except rospy.RosInterruptException:
    pass # Shutdown or issue with ros quitely shutdown
finally:
    GPIO.cleanup() # Clean up node after everything is finished




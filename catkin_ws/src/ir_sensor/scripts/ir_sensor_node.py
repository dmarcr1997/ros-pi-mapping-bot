#! /usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
import board
import busio

from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

#=== ROS NODE INIT ====
rospy.init_node("ir_sensor_node") # Create new node ir_sensor_node
pub = rospy.Publisher("/ir_sensor", Bool, queue_size=10) # Create data publisher for IR_SENSOR node that returns a Bool depending on detection
rate = rospy.Rate(10)  # 10 Hz
thermal_pub = rospy.Publisher("/thermal", Float32, queue_size=10) # Data publisher for ir values for thermal mapping
rospy.loginfo("IR sensor node started...")

#=== I2C + ADS1115 Setup ===
i2c = busio.I2C(board.SCL, board.SDA) # Setup I2C to use SCL + SDA
ads = ADS1115(i2c) # instantiate ADS1115 connected to i2c
chan = AnalogIn(ads, 0)

#=== Threshold =============
THRESHOLD_VALUE = 18000 # ADC IR Value for detection from IR SENSOR

try:
    while not rospy.is_shutdown():
        #voltage = chan.value # should be lower values for darker colors and > 1 for lighter colors
        rospy.loginfo(f"VALUE: {voltage}")
        detected = voltage > THRESHOLD_VALUE # 0 or LOW Value means detection
        pub.publish(detected) # Publish value to /ir_sensor topic
        thermal_pub.publish(voltage) # Publish thermal reading to thermal topic
        rate.sleep()

except rospy.RosInterruptException:
    pass # Shutdown or issue with ros quitely shutdown

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

IN1 = 11
IN2 = 12
IN3 = 13
IN4 = 15

def setup_gpio():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(IN1, GPIO.OUT)
	GPIO.setup(IN2, GPIO.OUT)
	GPIO.setup(IN3, GPIO.OUT)
	GPIO.setup(IN4, GPIO.OUT)
	stop()

def stop():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.LOW)

def forward():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)

def backward():
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)

def spin_left():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)

def spin_right():
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)

def callback(data):
	command = data.data.lower()
	if command == "forward":
		forward()
	elif command == "backward":
		backward()
	elif command == "left":
		spin_left()
	elif command == "right":
		spin_right()
	elif command == "stop":
		stop()

def motor_driver():
	rospy.init_node('motor_driver', anonymous=True)
	rospy.Subscriber("motor_commands", String, callback)
	rospy.spin()

if __name__ == '__main__':
	setup_gpio()
	try:
		motor_driver()
	except rospy.ROSInterruptException:
		pass
	finally:
		stop()
		GPIO.cleanup()

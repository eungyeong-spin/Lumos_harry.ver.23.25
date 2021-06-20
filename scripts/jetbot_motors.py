#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String, Int64
from lumos.msg import control
from geometry_msgs.msg import Twist



# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)


# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + 'LEFT:' + str(msg.left) + 'RIGHT:' + str(msg.right))
	set_speed(motor_left_ID,  msg.left)
	set_speed(motor_right_ID,  msg.right) 

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	elif msg.data.lower() == "slow":
		motor_left.setSpeed(65)
		motor_right.setSpeed(65)
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)

def on_cmd_vel(msg):
	ROBOT_WIDTH = 0.8
        left_value = 3*(msg.linear.x - msg.angular.z*ROBOT_WIDTH/2)
        right_value = 3*(msg.linear.x + msg.angular.z*ROBOT_WIDTH/2)
	rospy.loginfo(rospy.get_caller_id() + 'LEFT:' + str(left_value) + 'RIGHT:' + str(right_value))
	set_speed(motor_left_ID, left_value)
	set_speed(motor_right_ID, right_value)

def on_cx(msg):
	rospy.loginfo("cx = "+ str(msg.data))

	if msg.data >= 900:
		set_speed(motor_left_ID,  0)
		set_speed(motor_right_ID,  0.9) 
	elif msg.data < 900 and msg.data >= 750:
		set_speed(motor_left_ID,  0.6)
		set_speed(motor_right_ID,  0.9)
	elif msg.data < 750 and msg.data >= 550:
		set_speed(motor_left_ID,  0.9)
		set_speed(motor_right_ID, 0.9) 
	elif msg.data < 550 and msg.data >= 400:
		set_speed(motor_left_ID,   0.9)
		set_speed(motor_right_ID,  0.6)
	elif msg.data < 400:
		set_speed(motor_left_ID,  0.9)
		set_speed(motor_right_ID,  0) 

# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = Adafruit_MotorHAT(i2c_bus=1)

	motor_left_ID = 1
	motor_right_ID = 2

	motor_left = motor_driver.getMotor(motor_left_ID)
	motor_right = motor_driver.getMotor(motor_right_ID)

	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	rospy.Rate(10)
	
	rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
	rospy.Subscriber('~cmd_raw', control, on_cmd_raw)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)
	rospy.Subscriber('~cmd_vel', Twist, on_cmd_vel)
	rospy.Subscriber('cx', Int64, on_cx)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()


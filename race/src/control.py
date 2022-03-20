#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
err_sum = 0

 
# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward. 
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 0.0	#TODO

# Publisher for moving the car. 
# TODO: Use the coorect topic /car_x/offboard/command.
command_pub = rospy.Publisher('/car_3/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle
	global scalar
	global err_sum
	angle = 0.0

	print("PID Control Node is Listening to error")
	
	## Your PID code goes here
	#: Use kp, ki & kd to implement a PID controller
	
	# 1. Scale the error
	# err_scaled = data.pid_error*scalar
	err_sum += data.pid_error

	# 2. Apply the PID equation on error to compute steering
	# thetad = kp*data.pid_error + kd*(prev_error - err_scaled)
	# for derivative of error, need (current error - prev_error)/time between messages
	angle = kp*data.pid_error + kd*(prev_error - data.pid_error) + ki*err_sum
	

	
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# Make sure the steering value is within bounds [-100,100]
	angle = angle if angle < 100 else 100
	angle = angle if angle > -100 else -100
	command.steering_angle = angle

	# Make sure the velocity is within bounds [0,100]
	

	# TODO: implement dynamic velocity control (scale velocity down as error goes up, up as error goes down)
	vel_input = vel_input/abs(data.pid_error) if abs(data.pid_error) > 0.5 else vel_input
	vel_input = vel_input if vel_input < 100 else 100
	vel_input = vel_input if vel_input >= 0 else 0
	command.speed = vel_input

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':
	# global kp
	# global kd
	# global ki
	# global vel_input
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	scalar = input("Input error scalar")
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()

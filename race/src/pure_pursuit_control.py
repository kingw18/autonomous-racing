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
err_queue = [] 
err_queue_size = 10
min_speed = 30
max_speed = 50
max_speed_curve = 40
err_thresh = 1.21
err_exp = 0.5
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
	global err_queue
	global err_queue_size
	# print("PID Control Node is Listening to error")
	
	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	
	# 1. Scale the error
	# err_scaled = data.pid_error*scalar
	
	if len(err_queue) >= err_queue_size:
		err_sum -= err_queue.pop()
	
	err_sum += data.pid_error
	err_queue.insert(0, data.pid_error)
	# err_sum += data.pid_error
	error = data.pid_error
	front_dist = data.pid_vel
	# print("Error:", error)
	# 2. Apply the PID equation on error to compute steering
	# thetad = kp*data.pid_error + kd*(prev_error - err_scaled)
	# for derivative of error, need (current error - prev_error)/time between messages
	error_diff = prev_error - error
	angle = kp*data.pid_error + kd*(error_diff) + ki*err_sum
	
	# print("Error difference:", error_diff)
	# print("Angle:", angle)
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	angle = angle if angle < 100 else 100
	angle = angle if angle > -100 else -100
	command.steering_angle = angle
	
	'''	
	if front_dist < 1:
		set_speed = 15
		# print("Scaled speed to:", set_speed)
	elif front_dist < 4:
		set_speed = vel_input/((5-front_dist)**.5)*(8+front_dist)/12
	else:
		set_speed = vel_input
	if set_speed < 15:
		set_speed = 15
	if set_speed > 60:
		set_speed = 60
	'''
	
	# max_speed = 80
	if abs(error) > err_thresh:
		set_speed = vel_input*((err_thresh / abs(error)) ** err_exp)
	else:
		set_speed = vel_input
	if set_speed < min_speed:
		set_speed = min_speed
	if set_speed > max_speed:
		set_speed = max_speed
	command.speed = set_speed
	
	if data.pid_vel and set_speed > max_speed_curve:
		command.speed = max_speed_curve	
		
	prev_error=error
	# print("Error:", data.pid_error)
	# print("Angle:", angle)
	# print("Speed:", set_speed)
	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':
	# global kp
	# global kd
	# global ki
	# global vel_input
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# ki = input("Enter Ki Value: ")
	# vel_input = input("Enter desired velocity: ")
	kp = 10
	kd = .1
	ki = .001
	vel_input = max_speed
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()

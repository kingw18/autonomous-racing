#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall)
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
car_width = 0.35
prev_distance = 0
look_ahead = 4
# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data, angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
	global prev_distance
	'''
	# Convert angle to LIDAR's coordinates
	angle += 30
	angle = math.radians(angle)

	# Get index
	angle_ind = int(round(angle / data.angle_increment))

	distance = data.ranges[angle_ind]
	if data.range_min <= distance <= data.range_max:
		return distance
	return 100.0	# TODO: Better implementation of NaNs?
	'''
	distance = data.ranges[int((angle+30)*len(data.ranges)/240)]
	if math.isnan(distance) or not data.range_min <= distance <= data.range_max:
		return data.range_max
	return distance

def callback(data):
	global forward_projection

	theta = 60 # you need to try different values for theta
	side_theta = 0
	forward_theta = side_theta+theta
	if checkFront(data):
		error = 0
	else:
		error = follow_wall(data, side_theta, forward_theta)
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error	
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)

def follow_wall(data, side_theta, forward_theta):
	'''
	RIGHT SIDE
	'''
	a = getRange(data, forward_theta)
	b = getRange(data, side_theta)
	if a > 4 or b > 4:
		right_error = 0
	else:
		theta = math.radians(forward_theta-side_theta)
		alpha = math.atan((a * math.cos(theta) - b)/(a * math.sin(theta)))
		AB = b * math.cos(alpha)
		AC = vel
		CD = AB + AC * math.sin(alpha)
		right_error = desired_distance - CD
	'''
	LEFT SIDE
	'''
	forward_theta = 180 - forward_theta
	side_theta = 180 - side_theta
	a = getRange(data, forward_theta)
	b = getRange(data, side_theta)
	if a > 4 or b > 4:
		left_error = 0
	else:
		theta = math.radians(side_theta-forward_theta)
		alpha = math.atan((a * math.cos(theta) - b)/(a * math.sin(theta)))
		AB = b * math.cos(alpha)
		AC = vel
		CD = AB + AC * math.sin(alpha)
		left_error = desired_distance - CD
	return right_error - left_error

def checkFront(data):
	global car_width
	global look_ahead
	fov_start_angle = math.atan(look_ahead/(car_width*.8))*180/math.pi
	fov_end_angle = 180 - fov_start_angle
	angle_increment = len(data.ranges)/angle_range
	correction = (fov_start_angle * angle_increment, fov_end_angle * angle_increment)
	close_seq = 0
	for i in range(correction[0], correction[1]):
		dist = data.ranges[i]
		if math.isnan(dist) or not data.range_min + 0.0001 <= dist <= data.range_max:
			dist = data.range_max
		if dist < look_ahead:
			close_seq += 1
		else:
			close_seq = 0
		if close_seq > 5:
			return False
	return True

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("/car_3/scan", LaserScan, callback)
	rospy.spin()

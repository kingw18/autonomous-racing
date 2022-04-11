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
car_width = 0.50  # TODO: How wide is the car??
prev_distance = 0
forward_angle = 90	# Angle representing the direction the car is currently facing

# FTG Variables
gap_threshold = 1.0	# Threshold distance for defining gap
bubble_rad = 10	# The radius (in angle increments) of the safety bubble (naive implementation)

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
	if data.range_min <= distance <= data.range_max:
		prev_distance=distance	
		return distance
	return prev_distance


def dist_between_measurements(dist1, dist2, angle):
	"""
	Computes the distance between two measurement points given their respective distances from the car
	and the angle between them
	"""
	dist_squared = (dist1 ** 2) + (dist2 ** 2) - (2 * dist1 * dist2 * math.cos(math.radians(angle)))	# Law of Cosines
	return dist_squared ** 0.5


def ftg_target_angle(data):
	"""
	Returns the target goal determine by Follow-The-Gap in degrees
	"""
	angle_increment = len(data.ranges) / angle_range

	# Copy the distance into a list that can be manipulated in order to preserve the original data
	ranges = []
	closest_point = data.range_max + 1
	closest_point_ind = -1
	for i in range(len(data.ranges)):
		dist = data.ranges[i]
		if not data.range_min <= dist <= data.range_max:
			ranges.append(data.range_max + 1) # Use as a substitute for infinity, actual value shouldn't matter as long as its greater than max
		else:
			if dist < closest_point:
				closest_point = dist
				closest_point_ind = i
			ranges.append(dist)

	# Create Safety Bubble - Naive Implementation
	"""
	This just creates the bubble around a certain number of measurements on either side of the closest point.
	I remember Prof. Behl saying in lecture that a more proper implementation would actually compute distances
	to make the radius of the bubble equal to the width of the car. I tried to implement that below
	"""
	for i in range(max(0, closest_point_ind - bubble_rad), min(len(data.ranges), closest_point_ind + bubble_rad)):
		ranges[i] = 0

	# # Create Safety Bubble - Full Implementation
	# """
	# I think this is the proper implementation but it seems computationally very expensive
	# """
	# ind = closest_point_ind + 1
	# while ind < len(ranges) and dist_between_measurements(closest_point, ranges[ind], angle_increment * (ind - closest_point_ind)) <= car_width:
	# 	ranges[ind] = 0
	# 	ind += 1
	# ind = closest_point_ind - 1
	# while ind >= 0 and dist_between_measurements(closest_point, ranges[ind], angle_increment * (closest_point_ind - ind)) <= car_width:
	# 	ranges[ind] = 0
	# 	ind -= 1

	# TODO: Disparity Extender

	# Find Max Gap
	max_gap = (0, 0)
	gap_start = 0
	in_gap = False
	for i in range(len(ranges)):
		if in_gap and ranges[i] == 0:	# Should we compare directly to 0 or within a certain threshold?
			if i - gap_start > max_gap[1] - max_gap[0]:
				max_gap = (gap_start, i)
			in_gap = False
		elif not in_gap and ranges[i] > 0:
			gap_start = i
			in_gap = True

	# Find target angle - Naive Implementation (Find deepest point)
	max_ind = -1
	deepest_dist = 0
	for i in range(max_gap[0], max_gap[1]):
		if ranges[i] > deepest_dist:
			deepest_dist = ranges[i]
			max_ind = i

	return -30 + (max_ind * angle_increment)


def callback(data):
	global forward_projection

	"""
	# Legacy Code
	theta = 60 # you need to try different values for theta
	a = getRange(data, theta) # obtain the ray distance for theta
	b = getRange(data, 0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	print(a, b)
	theta = math.radians(theta)	# Convert to radians to make angle computations work

	# Compute Alpha, AB, and CD.. and finally the error.

	alpha = math.atan((a * math.cos(theta) - b)/(a * math.sin(theta)))
	"""
	alpha = ftg_target_angle(data) - forward_angle
	alpha = math.radians(alpha)
	# AB = b * math.cos(alpha)
	# AC = vel
	# CD = AB + AC * math.sin(alpha)
	error = vel * math.sin(vel * math.sin(alpha))
	

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error	
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("/car_3/scan", LaserScan, callback)
	rospy.spin()

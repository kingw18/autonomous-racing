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
car_width = 0.3	# Car is about 30cm wide
prev_distance = 0
forward_angle = 90	# Angle representing the direction the car is currently facing
min_threshold = 0.05
# FTG Variables
gap_threshold = 0.1	# Threshold distance for defining gap
bubble_rad = 100	# The radius (in angle increments) of the safety bubble (naive implementation)
depth_threshold=2
disparity_threshold=0.5	# The difference required to create a disparity

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

# Lidar's min range is .02 and max range is 6
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
	global gap_threshold
	angle_increment = len(data.ranges) / angle_range

	# Copy the distance into a list that can be manipulated in order to preserve the original data
	ranges = []
	closest_point = data.range_max + 1
	closest_point_ind = -1
	for i in range(len(data.ranges)):
		dist = data.ranges[i]
    
		if dist < closest_point and dist >= min_threshold:
			closest_point = dist
			closest_point_ind = i
			ranges.append(dist)
		
		elif dist > data.range_max or dist <= min_threshold:
			if i != 0 and i != len(data.ranges) - 1:
				ranges.append((data.ranges[i-1] + data.ranges[i+1])/2)
			else:
				if i == 0:
					ranges.append(data.ranges[i+1])
				else:
					ranges.append(data.ranges[i-1])
					
		else:
			ranges.append(dist)

	for i in range(len(ranges)):
		if ranges[i] < min_threshold or ranges[i] > data.range_max:
			ranges[i] = data.range_max + 2
	'''
	ranges = []
	closest_point = data.range_max + 1
	closest_point_ind = -1
	for i in range(len(data.ranges)):
		dist = data.ranges[i]
		
		
		if dist < closest_point and dist >= min_threshold:
			closest_point = dist
			closest_point_ind = i
			ranges.append(dist)
		
		elif dist > data.range_max:
			ranges.append(data.range_max + 1)
		
		elif dist <= min_threshold:
			# global prev_distance
			# data.ranges[i] = prev_distance # this is sus so we might need to replace this
			ranges.append(data.range_max + 1)
		else:
			ranges.append(dist)
	'''
	
	# Create Safety Bubble - Naive Implementation
	"""
	This just creates the bubble around a certain number of measurements on either side of the closest point.
	I remember Prof. Behl saying in lecture that a more proper implementation would actually compute distances
	to make the radius of the bubble equal to the width of the car. I tried to implement that below
	"""
	'''
	# print('Closest point dist: ' + str(closest_point) + ' at ' + str(closest_point_ind/angle_increment - 30) + ' degrees.')
	for i in range(max(0, closest_point_ind - bubble_rad), min(len(data.ranges), closest_point_ind + bubble_rad)):
		ranges[i] = 0
	'''
	
	last_measurement = ranges[0]
	disparity_ind = 0
	in_disparity = False
	for i in range(1, len(ranges)):

		if in_disparity:
			if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (i-disparity_ind)/angle_increment) <= car_width:
				if ranges[i] > ranges[disparity_ind]:
					ranges[i] = ranges[disparity_ind]			
				else:
					in_disparity = False

			elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:
				in_disparity = True
				disparity_ind = i-1
				ranges[i] = ranges[i - 1]
			last_measurement = ranges[i]

	# Create Safety Bubble - Full Implementation
	"""
	I think this is the proper implementation but it seems computationally very expensive
	"""
	ind = closest_point_ind + 1
	while ind < len(ranges) and dist_between_measurements(closest_point, ranges[ind], angle_increment * (ind - closest_point_ind)) <= car_width:
		ranges[ind] = 0
		ind += 1
	ind = closest_point_ind - 1
	while ind >= 0 and dist_between_measurements(closest_point, ranges[ind], angle_increment * (closest_point_ind - ind)) <= car_width:
		ranges[ind] = 0
		ind -= 1
		
	print('Closest point:' + str(closest_point) + ' at ' + str(closest_point_ind/angle_increment - 30) + ' degrees')
	'''		
	# Disparity Extender
	last_measurement = ranges[0]
	disparity_ind = 0
	in_disparity = False
	for i in range(1, len(ranges)):
		if in_disparity:
			if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (i - disparity_ind)/angle_increment) <= car_width:
				if ranges[i] > ranges[disparity_ind]:
					ranges[i] = ranges[disparity_ind]
			else:
				in_disparity = False
		elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:
			in_disparity = True
			disparity_ind = i - 1
			ranges[i] = ranges[i - 1]
		last_measurement = ranges[i]
	'''
	'''
	# Find Max Gap
	max_gap = (0, 0)
	gap_start = 0
	current_deepest_in_gap = 0
	in_gap = False
	for i in range(len(ranges)):
		if in_gap and ranges[i] <= gap_threshold:	# Should we compare directly to 0 or within a certain threshold?
			if i - gap_start > max_gap[1] - max_gap[0]:
				max_gap = (gap_start, i)
				# if current_deepest_in_gap > depth_threshold  and dist_between_measurements(ranges[max_gap[0]], ranges[max_gap[1]], (max_gap[1]-max_gap[0])/angle_increment) > 2 * car_width:
					# print( ((max_gap[0] + max_gap[1])/2)/angle_increment - 30)
					# print(max_gap[0], max_gap[1])
	#				return ((max_gap[0] + max_gap[1])/2)/angle_increment - 30
			in_gap = False
		
	#		if current_deepest_in_gap > depth_threshold:
	#			max_gap = (gap_start, i)
	#			break
			current_deepest_in_gap = 0
		elif not in_gap and ranges[i] > 0:
			gap_start = i
			in_gap = True
			
	#	else:
	#		current_deepest_in_gap = max(current_deepest_in_gap, ranges[i])
			
	#	if in_gap:
	#		current_deepest_in_gap = max(current_deepest_in_gap, ranges[i])
			

	if in_gap and len(ranges) - gap_start > max_gap[1] - max_gap[0]:
		max_gap = (gap_start, len(ranges))
		
	# print(max_gap[0], max_gap[1])
	# max_gap[1]-=1
	# Find target angle - Naive Implementation (Find deepest point)
	max_ind = -1
	deepest_dist = 0
	for i in range(max_gap[0], max_gap[1]):
		if ranges[i] > deepest_dist:
			deepest_dist = ranges[i]
			max_ind = i
	
	
	print('Min: ' + str(max_gap[0]/angle_increment - 30) + ' Max: ' + str(max_gap[1]/angle_increment - 30))
	print(str(-30 + max_ind/angle_increment) + ' degrees')
	return -30 + (max_ind/angle_increment)


	# return ((max_gap[0] + max_gap[1])/2)/angle_increment - 30
	# print(data.angle_increment)
	# print(-30 + ((max_gap[0] + max_gap[1])/2)*angle_increment)	
	# return -30 + ((max_gap[0] + max_gap[1])/2)*data.angle_increment
	'''
	max_gap = (0, 0)
	gap_start = 0
	current_deepest_in_gap = 0
	current_deepest_in_gap_ind = 0
	in_gap = False
	for i in range(len(ranges)):
		if in_gap and ranges[i] <= data.range_min:	# Use range min # Should we compare directly to 0 or within a certain threshold?
			if i - gap_start > max_gap[1] - max_gap[0]:
				max_gap = (gap_start, i)

			in_gap = False
			if current_deepest_in_gap > depth_threshold and dist_between_measurements(ranges[max_gap[0]], ranges[max_gap[1]], (max_gap[1]-max_gap[0])/angle_increment) > car_width:
				print('[ ' + str(max_gap[0]/angle_increment - 30) + ', ' + str(max_gap[1]/angle_increment - 30) + ']')
				print('Going ' + str(current_deepest_in_gap_ind/angle_increment - 30) + ' degrees')
				# print(ranges[max_gap[0]: max_gap[1]])
				return -30 + (current_deepest_in_gap_ind/angle_increment)

			current_deepest_in_gap = 0
			current_deepest_in_gap_ind = i
		elif not in_gap and ranges[i] > data.range_min:
			gap_start = i
			in_gap = True
			
		if in_gap:
			if ranges[i] > current_deepest_in_gap and ranges[i] <= data.range_max:
				current_deepest_in_gap = max(current_deepest_in_gap, ranges[i])
				current_deepest_in_gap_ind = i


	#if in_gap and len(ranges) - gap_start > max_gap[1] - max_gap[0]:
	max_gap = (gap_start, len(ranges))
	print('[' + str(max_gap[0]/angle_increment - 30) + ', ' + str(max_gap[1]/angle_increment - 30) + ']')
	print('Going ' + str(current_deepest_in_gap_ind/angle_increment - 30) + ' degrees')
	return -30 + (current_deepest_in_gap_ind/angle_increment)


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
	# print(alpha+forward_angle)
	alpha = math.radians(alpha)
	
	# AB = b * math.cos(alpha)
	# AC = vel
	# CD = AB + AC * math.sin(alpha)
	# error = vel * math.sin(vel * math.sin(alpha))  # ADD THIS BACK LATER PROBABLY
	error = vel * alpha
	# print(error)	
	print('Error:', error)
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

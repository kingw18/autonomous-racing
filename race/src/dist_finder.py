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
car_width = 0.4	# Car is about 30cm wide
prev_distance = 0
forward_angle = 90	# Angle representing the direction the car is currently facing
min_threshold = 0.05
# FTG Variables
gap_threshold = 0.1	# Threshold distance for defining gap
bubble_rad = 100	# The radius (in angle increments) of the safety bubble (naive implementation)
depth_threshold=2
disparity_threshold=0.4	# The difference required to create a disparity
angle_increment = 0
fov = 150

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

# Lidar's min range is .02 and max range is 6
"""
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
"""


def dist_between_measurements(dist1, dist2, angle):
	"""
	Computes the distance between two measurement points given their respective distances from the car
	and the angle between them
	"""
	dist_squared = (dist1 ** 2) + (dist2 ** 2) - (2 * dist1 * dist2 * math.cos(math.radians(angle)))	# Law of Cosines
	return dist_squared ** 0.5

# Get the angle between 2 indices in ranges
def angle_between_indices(lower, higher):
	global angle_increment
	return abs(higher - lower)/angle_increment

def ftg_target_angle(data):
	
	"""
	Returns the target goal determine by Follow-The-Gap in degrees
	"""
	global gap_threshold
	global angle_increment
	angle_increment = len(data.ranges) / angle_range
	# Copy the distance into a list that can be manipulated in order to preserve the original data
	# ranges = data.ranges.copy()
	print('len:', len(data.ranges))
	ranges = []
	closest_point = data.range_max + 1
	closest_point_ind = -1
	correction = (240 - fov)*angle_increment/2
	for i in range(correction, len(data.ranges) - correction):
		dist = data.ranges[i]
		'''
		if dist < closest_point:
			closest_point = dist
			closest_point_ind = i
		'''
		if math.isnan(dist):
			# print('nan found')
			dist = 6
			ranges.append(dist)
		elif dist < data.range_min + .0001:
			dist = 6
			ranges.append(dist)	
		else:
			# print(dist)
			# print(math.isnan(dist))
			ranges.append(dist)
		
		if dist < closest_point:
			closest_point = dist
			closest_point_ind = i - correction

	# print(ranges)
	# print(ranges[closest_point_ind])
	# print('closest point at', ind_to_deg(closest_point_ind) , 'at distance', closest_point)
		
	def ind_to_deg(index):
		return 15 + (index/angle_increment)

	print('closest point at', ind_to_deg(closest_point_ind), 'at distance', closest_point)
	# print("Min Degrees:", ind_to_deg(0), "Max Degrees", ind_to_deg(len(ranges) - 1), 'Correction:', correction)
	def set_safety_bubble(index):
		# print(index, len(ranges))
		# print(ranges, index)
		lo = index - 1
		hi = index + 1
		# print(index, ranges[index])
		while lo >= 0 and index-lo < 300 and dist_between_measurements(ranges[index], ranges[index], angle_between_indices(lo, index)) <= .5 * car_width:  #and ranges[lo] >= ranges[index] :
			# print(ranges[lo] >= ranges[index])
			# print(dist_between_measurements(ranges[index], ranges[index], angle_between_indices(lo, index)))
			# ranges[lo] = 0
			lo -= 1

		while hi < len(ranges) and hi-index < 300 and dist_between_measurements(ranges[index], ranges[index], angle_between_indices(index, hi)) <= .5 * car_width: # and ranges[hi] >= ranges[index] :
			# print(index, hi, angle_increment)
			# dist_between_measurements(ranges[index], ranges[index], angle_between_indices(index, hi))
			# print('hi', hi)
			# ranges[hi] = 0
			hi += 1

		# ranges[index] = 0
		# Return the range to set to 0s (I can explain why we don't directly set them to 0)
		# print('Lo, hi', lo, hi)
		print('Safety bubble set at:', ind_to_deg(index), 'to', ind_to_deg(lo), ind_to_deg(hi))
		return (lo, hi)

	# Holds all ranges that I should set to zero (safety bubbles)
	set_to_zero = []
	set_to_zero.append(set_safety_bubble(closest_point_ind))
	
	# Set a safety bubble around all disparities
	# DISCLAIMER: This isn't the most efficient but it should be fast enough and is more readable
	# Worst case: everything will be set to 0 at most twice so this isn't an O(n^2) algorithm I think
	for i in range(0, len(ranges) - 1):
		if ranges[i+1] - ranges[i] > disparity_threshold and ranges[i] <3:
			# There is a disparity at index i
			# print(i)
			set_to_zero.append(set_safety_bubble(i))
    
	for i in reversed(range(1, len(ranges))):
		if ranges[i -1] - ranges[i] > disparity_threshold and ranges[i] < 3:
			# There is a disparity at index i
			# print(i)
			set_to_zero.append(set_safety_bubble(i))

	for r in set_to_zero:
		for i in range(r[0], r[1]):
			ranges[i] = 0

	# print(ranges)
	# List of all gaps
	# Each gap is a tuple: (start index, end index)
	gaps = []
	# print(set_to_zero)
	#print(ranges)
	# We want to find every gap 
	# ^ occurs when a nonzero value is next to a 0
	gap_start = 0
	for i in range(1, len(ranges) - 1):
		
		if ranges[i - 1] == 0 and ranges[i] != 0:
			# If ranges[i-1] is 0 and ranges[i] isn't, we must be at the start of a gap
			gap_start = i
		if ranges[i + 1] == 0 and ranges[i] != 0:
			# If ranges[i+1] is 0 and ranges[i] isn't, we must be at the end of a gap
			# Append the gap to the gaps list
			# print('Gap from', gap_start, i)
			gaps.append((gap_start, i))
			gap_start = i+1
		
	# In case a gap extends past the fov
	gaps.append((gap_start, len(ranges) - 1))
	gap_metrics = []
	gap_metrics_uncombined = []
	
	# This is inefficient but I'm doing this for readability
	# Finds the gap metric of all gaps
	print('Gaps:',)
	for gap in gaps:
		print('(', ind_to_deg(gap[0]), ind_to_deg(gap[1]),')',)
		sliced = ranges[gap[0]:gap[1] + 1]
		min_depth = min(sliced)
		max_depth = max(sliced)
		gap_width = dist_between_measurements(ranges[gap[0]], ranges[gap[1]], angle_between_indices(gap[0], gap[1]))
		avg_dist = sum(sliced)/(gap[1] - gap[0] + 1)
		gap_metrics.append(avg_dist*(max_depth**.5)*gap_width)
		gap_metrics_uncombined.append((min_depth, max_depth, gap_width, avg_dist))

	print()
	# Finds the best gap according to the gap metric
	max_metric = 0
	best_gap = 0
	for i in range(len(gap_metrics)):
		if gap_metrics[i] > max_metric:
			max_metric = gap_metrics[i]
			best_gap = i

	
	print('Gap metrics', gap_metrics)
	# print('Gap metrics uncombined', gap_metrics_uncombined)
	far_ind = 0
	far_dist = 0
	for i in range(gaps[best_gap][0], gaps[best_gap][1]):
		if ranges[i] > far_dist:
			far_dist = ranges[i]
			far_ind = i
	'''
	if gap_metrics_uncombined[best_gap][0] > 2:
		print("Best gap is gap #", best_gap)
		print('Best gap:', gaps[best_gap])
		print('Target angle:', ind_to_deg(far_ind))
		print('Target index:', far_ind)	
		return ind_to_deg((gaps[best_gap][0]+gaps[best_gap][1])/2)
	'''
	print("Best gap is gap #", best_gap)
	print("Best gap from:", ind_to_deg(gaps[best_gap][0]), 'to', ind_to_deg(gaps[best_gap][1]))
	print("Target angle:", ind_to_deg(far_ind))
	print("Target index:", far_ind)
	
	middle_of_gap = (gaps[best_gap][0] + gaps[best_gap][1])/2
	print("Middle of gap:", ind_to_deg(middle_of_gap))
	return ind_to_deg(.45 *far_ind + .55 * middle_of_gap)
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
	'''
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
	
	last_measurement = ranges[0]
	disparity_ind = 0
	in_disparity = False
	'''

	# Create Safety Bubble - Full Implementation
	"""
	I think this is the proper implementation but it seems computationally very expensive
	"""
	'''
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
	'''
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
	# Disparity Extender

	
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
	'''

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

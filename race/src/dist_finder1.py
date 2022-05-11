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
min_threshold = 0.03
# FTG Variables
gap_threshold = 0.1	# Threshold distance for defining gap
bubble_rad = 100	# The radius (in angle increments) of the safety bubble (naive implementation)
depth_threshold=2.5
disparity_threshold=0.5	# The difference required to create a disparity
fov_width = 150 # input("Enter FOV in degrees: ")
obst_theta = 75 # input("Enter obstacle finding angle: ")
# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

# Lidar's min range is .02 and max range is 6
def getRange(data, angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
	global prev_distance
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

'''
def ftg_target_angle(data):
	
	"""
	Returns the target goal determine by Follow-The-Gap in degrees
	"""
	global gap_threshold
	ranges = []
	closest_point = data.range_max + 1
	closest_point_ind = -1
	angle_increment = len(data.ranges) / angle_range
	fov_angle_index = int(angle_increment*(240-fov_width)/2)
	
	"""
	STEP 1 & 2
	Step 1: Preprocess the laser scans and set the FOV
	Step 2: Find the closest point
	"""
	for i in range(fov_angle_index, len(data.ranges) - fov_angle_index):
		dist = data.ranges[i] 
		angle_from_fwd = abs(len(data.ranges)/2 - i)/angle_increment
	#	if angle_from_fwd <= obst_theta:
	#		dist = dist - .45
		if dist < closest_point and dist >= min_threshold and angle_from_fwd <= obst_theta:
			closest_point = dist #AAAAAAAAAAAAAAAAAAAAAAAA
			closest_point_ind = i - fov_angle_index
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
		if ranges[i] < min_threshold: 
			ranges[i] = data.range_min
		elif ranges[i] > data.range_max:
			ranges[i] = data.range_max + 2
	
	"""
	STEP 3
	Disparity extender and safety bubble
	"""
	# CCW
	last_measurement = ranges[0]
	disparity_ind = 0
	in_disparity = False
	for i in range(1, len(ranges)):
		if in_disparity:
			if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (i-disparity_ind)/angle_increment) <=1.3 *car_width:
				if ranges[i] > ranges[disparity_ind]:
					ranges[i] = ranges[disparity_ind]			
				else:
					in_disparity = False

		elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:
			disparity_ind = i-1
			in_disparity = True
			print("Found disparity at", -30 + 120-fov_width/2 + i/angle_increment)
			ranges[i] = ranges[i - 1]
		last_measurement = ranges[i]
	# CLOCKWISE
	last_measurement = ranges[len(ranges)-1]
	disparity_ind = len(ranges) - 1
	in_disparity = False
	for i in reversed(range(0, len(ranges)-1)):
		if in_disparity:
			if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (disparity_ind-i)/angle_increment) <= 1.3 * car_width:
				if ranges[i] > ranges[disparity_ind]:
					ranges[i] = ranges[disparity_ind]			
				else:
					in_disparity = False

		elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:	
			in_disparity = True
			print("Found disparity at", -30 + 120-fov_width/2 + i/angle_increment)
			disparity_ind = i + 1
			ranges[i] = ranges[i + 1]
		last_measurement = ranges[i]
	
	left_zero_ind = len(ranges)
	right_zero_ind = 0
	ind = closest_point_ind + 1
	while ind < len(ranges) and dist_between_measurements(closest_point, ranges[ind], angle_increment * (ind - closest_point_ind)) <= 1.3*car_width:
		ranges[ind] = 0
		left_zero_ind = ind
		ind += 1
	ind = closest_point_ind - 1
	# print('First ind going right', ind)
	while ind >= 1 and dist_between_measurements(closest_point, ranges[ind], angle_increment * (closest_point_ind - ind)) <=  1.3*car_width:
		ranges[ind] = 0
		right_zero_ind = ind
		ind -= 1
	#	print("ind in right while:", ind)
	ranges[closest_point_ind] = 0
	print('Safety Bubble Range:', right_zero_ind/angle_increment - 30 + (240-fov_width)/2, left_zero_ind/angle_increment - 30 + (240-fov_width)/2)
	"""
	STEP 4
	Find the widest gap
	"""
	
	if right_zero_ind <= 1:
		max_gap = (left_zero_ind, len(ranges) - 1)
		print('1')
	elif left_zero_ind > len(ranges)-3:
		max_gap = (0, right_zero_ind)
		print('2')
	elif max(ranges[:right_zero_ind]) > depth_threshold and max(ranges[left_zero_ind:]) < depth_threshold:
		max_gap = (0, right_zero_ind)
		print('3')
	elif max(ranges[:right_zero_ind]) < depth_threshold and max(ranges[left_zero_ind:]) > depth_threshold:
		max_gap = (left_zero_ind, len(ranges) - 1)
		print('4')
	elif len(ranges) - 1 - left_zero_ind > right_zero_ind:
		max_gap = (left_zero_ind, len(ranges) - 1)
		print('5')
	else:
		max_gap = (0, right_zero_ind)
		print('6')
	
	deepest_dist = ranges[max_gap[0]]
	deepest_dist_ind = max_gap[0]
	
	closest_in_front = ranges[int(len(ranges)/2)]
	for i in range(len(ranges)):
		angle_from_fwd = abs(len(ranges)/2 - i)/angle_increment
		if angle_from_fwd < 20 and ranges[i] < closest_in_front:
			closest_in_front = ranges[i]
	for i in range(max_gap[0], max_gap[1]):
		if ranges[i] > deepest_dist:
			deepest_dist_ind = i
			deepest_dist = ranges[i]
	print('Closest point:' + str(closest_point) + ' at ' + str((closest_point_ind+fov_angle_index)/angle_increment - 30) + ' degrees')
	print('Min index:' ,max_gap[0],'Max index:', max_gap[1])
	print('Min: ' + str((max_gap[0]+fov_angle_index)/angle_increment - 30) + ' Max: ' + str((max_gap[1]+fov_angle_index)/angle_increment - 30))
	# target = -30 + (240-fov_width)/2 + (max_gap[0] + max_gap[1])/(2*angle_increment)
	target = (-30 + (240-fov_width)/2 + (deepest_dist_ind/angle_increment))
	print('Target:', target)
	print(ranges[target-20:target+20])
	# print('Range:', len(ranges))
	print('Left index:',left_zero_ind, 'Right index:', right_zero_ind)
	return target, closest_in_front
	'''	
	
	
	'''	
	gap_max_depths = []
	gap_max_inds = []
	gap_min_depths = []
	edges = []
	gaps = []
	# Find edges
	print("len of ranges:", len(ranges))
	edges.append(0)
	for i in range(len(ranges)-1):
    		if abs(ranges[i]- ranges[i+1]) > 0.25: # if there is an edge here
        		edges.append(i)
			print("Found an edge :)")
		
	edges.append(len(ranges)-1)
	# Define gaps
	for i in range(1, len(edges)):
		gaps.append((edges[i-1]+1, edges[i]))
	# Find how deep the gaps are
	for i in range(len(gaps)):
    		max_in_gap = 0
    		min_in_gap = data.range_max
		max_index = 0
    		for j in range(gaps[i][0], gaps[i][1]):
        		if ranges[j] > max_in_gap:
            			max_in_gap = ranges[j]
            			max_index = j
        		if ranges[j] < min_in_gap:
            			min_in_gap = ranges[j]

    		gap_max_depths[i] = max_in_gap
    		gap_min_depths[i] = min_in_gap
    		gap_max_inds[i] = max_index

	gap_metric = []
	for i in range(len(gaps)):
    		gap_dists = ranges[gaps[i][0]:gaps[i][1]]
    		gap_width_incs = gaps[i][1]- gaps[i][0]
    		gap_width = dist_between_measurements(gap_dists[0], gap_dists[-1], gap_width_incs/angle_increment)
    		gap_avg_dist = sum(gap_dists)/(gap_width_incs)
    		gap_metric[i] = gap_avg_dist*(max(gap_dists)**0.5)*gap_width
    		if gap_width < car_width:
        		gap_metric[i] = 0

	max_metric = 0
	best_gap = 0
	for i in range(len(gap_metric)):
    		if gap_metric[i] > max_metric:
        		max_metric = gap_metric[i]
        		best_gap = i
	print("Aiming for gap #", best_gap)
	print("Out of", len(gap_max_inds), "gaps")
	target_ind = gap_max_inds[best_gap]
	
	return -30 + (240-fov_width)/2 + target_ind/angle_increment
	'''
# mid_index = int((gaps[best_gap][1] - gaps[best_gap][0])/2)
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
	alpha, front_dist = ftg_target_angle(data)
	alpha -= forward_angle
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
	msg.pid_vel = front_dist		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("/car_3/scan", LaserScan, callback)
	rospy.spin()

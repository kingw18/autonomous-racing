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
depth_threshold=2.5
disparity_threshold=0.5	# The difference required to create a disparity
fov_width = input("Enter FOV in degrees: ")
obst_theta = input("Enter obstacle finding angle: ")
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
	last_measurement = ranges[0]
	disparity_ind = 0
	in_disparity = False
	for i in range(1, len(ranges)):
	    if in_disparity:
		if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (i-disparity_ind)/angle_increment) <= 0.5 * car_width:
		    print("Dist between the measurements:", disparity_ind, "and", i, ":", dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (i-disparity_ind)/angle_increment))
		    print("Angle:", (i-disparity_ind)/angle_increment)
		    if ranges[i] > ranges[disparity_ind]:
			ranges[i] = ranges[disparity_ind]			
		    else:
			in_disparity = False
			print("leaving disparity")

	    elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:
		in_disparity = True
		print("Found disparity at", i)
		disparity_ind = i-1
		ranges[i] = ranges[i - 1]
	    last_measurement = ranges[i]
	# CLOCKWISE
	last_measurement = ranges[len(ranges)-1]
	disparity_ind = len(ranges) - 1
	in_disparity = False
	for i in reversed(range(0, len(ranges)-1)):
	    if in_disparity:
		if dist_between_measurements(ranges[disparity_ind], ranges[disparity_ind], (disparity_ind-i)/angle_increment) <= 0.5 * car_width:
		    if ranges[i] > ranges[disparity_ind]:
			ranges[i] = ranges[disparity_ind]			
		    else:
			in_disparity = False

	    elif ranges[i] > last_measurement + disparity_threshold and not in_disparity:
		in_disparity = True
		disparity_ind = i + 1
		ranges[i] = ranges[i + 1]
	    last_measurement = ranges[i]
	gap_max_depths = []
	gap_min_depths = []
	edges = []
	gaps = []
	# Find edges
	# print(ranges)
	edges.append(0)
	for i in range(len(ranges)-2):
	    if abs(ranges[i] - ranges[i+1]) > 0.25:
		edges.append(i+1)
		print("found an edge at", i)
	edges.append(len(ranges)-1)
	# Define gaps
	for i in range(len(edges) - 1):
	    gaps.append((edges[i], edges[i+1]-1))

	# Find how deep the gaps are
	for i in range(len(gaps)):
	    max_in_gap = 0
	    min_in_gap = 8
	    for j in range(gaps[i][0], gaps[i][1]):
		if ranges[j] > max_in_gap:
		    max_in_gap = ranges[j]
		    max_index = j
		if ranges[j] < min_in_gap:
		    min_in_gap = ranges[j]

	    gap_max_depths.append(max_in_gap)
	    gap_min_depths.append(min_in_gap)

	gap_metric = []
	print("gaps before metric", gaps)
	for i in range(len(gaps)):
	    gap_dists = ranges[gaps[i][0]:gaps[i][1]+1]
	    gap_width_incs = gaps[i][1]- gaps[i][0]
	    # print("Gap dists:", gap_dists, "range:", gaps[i][0], "to", gaps[i][1]+1)

	    first = gap_dists[0]
	    second = gap_dists[-1]
	    gap_width = dist_between_measurements(gap_dists[0], gap_dists[-1], gap_width_incs/(len(ranges)/150))
	    if gap_width_incs:
		gap_avg_dist = sum(gap_dists)/(gap_width_incs)
	    else: 
		gap_avg_dist = sum(gap_dists)
	    gap_metric.append(gap_avg_dist*(max(gap_dists)**0.5)*gap_width)
	    if gap_width < car_width:
		gap_metric[i] = 0

	max_metric = 0
	best_gap = 0
	for i in range(len(gap_metric)):
	    if gap_metric[i] > max_metric:
		max_metric = gap_metric[i]
		best_gap = i

	far_ind = 0
	far_dist = 0
	for i in range(gaps[best_gap][0],gaps[best_gap][1]):
	    if ranges[i] > far_dist:
		far_dist = ranges[i]
		far_ind = i
	print(gaps)
	print("Best gap is gap #", best_gap)
	print("Best gap:", gaps[best_gap])
	print("Target angle:", 15 + far_ind/(len(ranges)/150))
	print("Target index:", far_ind)
	
	target = 15 + far_ind/(len(ranges)/150)
	return target
		

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

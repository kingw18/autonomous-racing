#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
from geometry_msgs.msg import PoseStamped
from dist_finder import ftg_target_angle, forward_angle
from dist_find_pure_pursuit import infer_pos, find_steering_angle

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall)
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
car_width = 0.4	# Car is about 30cm wide
prev_distance = 0
min_threshold = 0.05
# FTG Variables
gap_threshold = 0.1	# Threshold distance for defining gap
bubble_rad = 100	# The radius (in angle increments) of the safety bubble (naive implementation)
depth_threshold=2
disparity_threshold=0.4	# The difference required to create a disparity
angle_increment = 0
fov = 150

# Hybrid Variables
thresh_dist = 0.4

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def callback(data):
	global forward_projection

	alpha_ftg = ftg_target_angle(data) - forward_angle
	alpha_ftg = math.radians(alpha_ftg)
	alpha_pp = find_steering_angle()

	ranges = []
	for i in range(len(data.ranges)):
		if data.ranges[i] < data.range_min + .0001:
			ranges.append(6)
		else:
			ranges.append(data.ranges[i])

	if min(ranges) < thresh_dist:
		alpha = alpha_ftg
	else:
		alpha = alpha_pp

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
	rospy.Subscriber("/car_3/particle_filter/viz/inferred_pose", PoseStamped, infer_pos)
	rospy.spin()

#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
from geometry_msgs.msg import PoseStamped

# Some useful variable declarations.
angle_range = 240  # Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5  # distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9  # distance from the wall (in m). (defaults to right wall)
vel = 15  # this vel variable is not really used here.
error = 0.0  # initialize the error
car_length = 0.50  # Traxxas Rally is 20 inches or 0.5 meters
car_width = 0.4  # Car is about 30cm wide
prev_distance = 0
forward_angle = 90  # Angle representing the direction the car is currently facing
min_threshold = 0.05
steering_angle_constant = .7
# ure Pursuit Variables
# waypoints = [(0, 0), (80, 7), (87, -2), (10, -46), (102, -65), (90, -95), (0, -123), (-34, -43)]
'''
waypoints = [(0, 0), (.3, -.08), (.63, -.25), (1, -.4), (1.5, -.63), (1.9, -.86), (2.4, -1), 
	(2.8, -1.1), (3.17, -1.16), (3.5, -.94), (3.71, -.64), (3.8, -.01), (3.61, .22),
	(3.19, .62), (2.31, 1.1), (1.82, 1.67), (1.99, 2.01), (2.61, 2.3), (3.14, 2.43),
	(3.65, 2.55), (4.07, 2.86), (4.29, 3.31), (4.23, 3.86), (3.92, 4.32), (3.61, 4.68),
	(3.25, 5.06), (2.53, 5.4), (1.9, 5.51), (1.25, 5.5), (.47, 5.22), (-.25, 4.9),
	(-.78, 4.32), (-1.07, 3.9), (-1.48, 2.72), (-1.35, 2.03), (-1.15, 1.35), (-1.03, .91),(-.78, .46), (-.39, .08)]
'''
waypoints = [
	(-.09, -.01), (.3, -.14), (.61, -.32), (1.04, -.49), (1.45, -.61), (2.07, -.8), (2.9, -.87), (3.49, -.7),
	(3.78, -.45), (3.84, .16), (3.28, .57), (2.56, .86), (2.04, 1.04), (1.5, 1.5), (1.32, 1.6), (1.32, 2.09),
	(1.79, 2.61), (2.69, 2.61), (3.39, 2.53), (4.15, 2.4), (4.76, 2.74), (5.1, 3.41), (4.59, 3.91),
	(3.92, 4.24), (3.1, 4.76), (2.22, 5), (1.27, 5.32), (.32, 5.4), (-.78, 4.56), (-1.45, 3.67), (-1.74, 2.68),
	(-1.63, 1.87), (-1.19, 1.15), (-.7, .63)]
lookahead_dist = 1.5

# Global Variables
car_x = 0
car_y = 0
car_theta = 0

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def quad_formula(a, b, c):
	"""
	Helper function to compute the quadratic formula
	"""
	print('a, b, c', a, b, c)
	return (-b + (b ** 2 - 4 * a * c) ** 0.5) / (2 * a)


def infer_pos(data):
	# Conversion from quaternions from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	global car_x, car_y, car_theta
	car_x = data.pose.position.x
	car_y = data.pose.position.y
	siny_cosp = 2 * (data.pose.orientation.w * data.pose.orientation.z)
	cosy_cosp = 1 - 2 * (data.pose.orientation.z * data.pose.orientation.z)
	car_theta = math.atan2(siny_cosp, cosy_cosp)


def steering_angle(y, L):
	"""
	Computes the steering angle gamma based on the formula from slide 34 of pure pursuit
	@param y: The distance from the car to the goal that is perpendicular to the car's current direction
	@param L: The lookahead distance
	@return: the steering angle gamma
	"""
	# return steering_angle_constant * 2 * abs(y) / (L ** 2)
	return steering_angle_constant * 2 * y / (L ** 2)

def dist(p1, p2):
	"""
	Returns the distance between two points
	"""
	return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5


def goal_point():
	"""
	Returns the current goal point based on the pure pursuit slides
	"""
	closest_i = 0
	dist_to_waypoint = dist(waypoints[0], (car_x, car_y))
	for i in range(1, len(waypoints)):
		if dist(waypoints[i], (car_x, car_y)) < dist_to_waypoint:
			dist_to_waypoint = dist(waypoints[i], (car_x, car_y))
			closest_i = i
	
	small_i	= closest_i	# The index of the last waypoint before the lookahead distance
	large_i = closest_i	# The index of the first waypoint after the lookahead distance
	short_dist = dist_to_waypoint	# The distance of the last waypoint before the lookahead distance
	long_dist = dist_to_waypoint	# The distance of the first waypoint after the lookahead distance
	while dist(waypoints[large_i], (car_x, car_y)) < lookahead_dist:
		large_i += 1
		large_i %= len(waypoints)
		short_dist = long_dist
		long_dist = dist(waypoints[large_i], (car_x, car_y))
	
	if short_dist > lookahead_dist:
		return waypoints[small_i]
	
	print('Small i waypoint:', small_i)
	print('Large i waypoint:', large_i)
	print('Waypoint small i:', waypoints[small_i])
	# Lots of math to interpolate between the two waypoints-- not sure if this can be simplified?
	a = short_dist
	c = dist(waypoints[large_i], waypoints[small_i])
	phi = math.atan2(car_y - waypoints[small_i][1], car_x - waypoints[small_i][0]) \
			- math.atan2(waypoints[large_i][1] - waypoints[small_i][1], waypoints[large_i][0] - waypoints[small_i][0])
	k = quad_formula(1, -2 * a * math.cos(phi), a**2 - lookahead_dist**2)
	q = k / c
	new_x = waypoints[small_i][0] + (waypoints[large_i][0] - waypoints[small_i][0]) * q
	new_y = waypoints[small_i][1] + (waypoints[large_i][1] - waypoints[small_i][1]) * q
	return new_x, new_y


def dist_between_measurements(dist1, dist2, angle):
	"""
	Computes the distance between two measurement points given their respective distances from the car
	and the angle between them
	"""
	dist_squared = (dist1 ** 2) + (dist2 ** 2) - (2 * dist1 * dist2 * math.cos(math.radians(angle)))  # Law of Cosines
	return dist_squared ** 0.5


def find_steering_angle():
	"""
	Determines the steering angle based on the current inferred position
	"""
	goal = goal_point()
	print('Before Translationg Goal:', goal, 'Car pos:', car_x, car_y)
	goal = (goal[0] - car_x, goal[1] - car_y)	# Translate origin to car position
	print('After translation Goal:', goal)
	# Roatate so that x axis is along the car
	goal = (goal[0] * math.cos(-1 * car_theta) - goal[1] * math.sin(-1 *car_theta),  (goal[0] * math.sin(-1 * car_theta) + goal[1] * math.cos(-1 * car_theta)))
	print('After rotation goal:', goal)
	print('Car theta:', car_theta)
	angle = steering_angle(goal[1], lookahead_dist)
	print('Angle:', angle)
	return angle


def callback(data):
	global forward_projection
	alpha = find_steering_angle()

	# error = vel * math.sin(vel * math.sin(alpha))  # ADD THIS BACK LATER PROBABLY
	error = vel * alpha
	# print('Error:', error)
	msg = pid_input()  # An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel  # velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder', anonymous=True)
	rospy.Subscriber("/car_3/scan", LaserScan, callback)
	rospy.Subscriber("/car_3/particle_filter/viz/inferred_pose", PoseStamped, infer_pos)
	rospy.spin()

# Kalman Buterbaugh (kb2cj)
def dist(p1, p2):
	"""
	Returns the distance between two points
	"""
	return ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5


def quad_formula(a, b, c):
	"""
	Helper function to compute the quadratic formula
	"""
	return (-b + (b ** 2 - 4 * a * c) ** 0.5) / (2 * a)


def interpolate(p1, p2, p3, L):
	a = dist(p2, p1)
	c = dist(p3, p2)
	theta = math.atan2(p2[1] - p3[1], p2[0] - p3[0]) \
			- math.atan2(car_y - p3[1], car_x - p3[0])
	phi = math.atan2(car_y - p2[1], car_x - p2[0]) \
			- math.atan2(p3[1] - p2[1], p3[0] - p2[0])
	k = quad_formula(1, -2 * a * math.cos(phi), a**2 - L**2)
	return (c - k) * math.sin(theta), (c - k) * math.cos(theta)


print(interpolate((0, 0), (0, 1), (2, 2), 3))


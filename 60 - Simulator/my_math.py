import math
import random

# math tool
def constraint(variable, min, max):
    if variable>max:
        return max
    elif variable<min:
        return min
    else:
        return variable

def gaussian_noise(mu, sig):
    return random.gauss(mu, sig)

def euclidean_distance(a, b):
    return math.hypot(b[0]-a[0], b[1]-a[1])

def angle_between_vectors(a, b):
    """Calculate the angle of the vector a to b"""
    return math.atan2(b[1]-a[1], b[0]-a[0])

# return True if the distance bewtween two points is greater than "distance"
def compare_two_points_distance(x,y,x2,y2,distance):
	return ( (x-x2)*(x-x2) + (y-y2)*(y-y2) ) < (distance*distance)

import my_odometry
import my_occupancy_map


import numpy as np
import math
import random

from my_math import *

class robot_odometry:

	def __init__(self, origin_x, origin_y, origin_h):

		self.ignore_counter = 160 #ignore first frames

		# odometry
		self.odom = my_odometry.odometry(origin_x, origin_y, origin_h)

		# map
		self.occupacy_map = my_occupancy_map.occupancy_map(2000,250,20.0)
		self.loop_counter = 0

	def process(
		self,
		dt,
		actual_speed_ms,
		actual_rotation_speed_dps,
		real_position_x,
		real_position_y,
		real_position_h,
		lidar_distance_left,
		lidar_distance_right

		 ):

		# wait for 3D/Phys stabilize
		if self.ignore_counter > 0:
			self.ignore_counter -= 1
			return

		#debug inputs
		#print("actual_speed_ms:" + str(round(actual_speed_ms,3))+ "ms    actual_rotation_speed_dps:" + str(round(actual_rotation_speed_dps,2)) + "dps")

		# stage #1 : odometry
		# update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		self.odom.update(actual_speed_ms,actual_rotation_speed_dps,dt)
		self.odom.print()

		# stage #2 : occupancy from real position
		x1 = real_position_x + lidar_distance_left * math.cos(math.radians(real_position_h+60))
		y1 = real_position_y + lidar_distance_left * math.sin(math.radians(real_position_h+60))
		x2 = real_position_x + lidar_distance_right * math.cos(math.radians(real_position_h-60))
		y2 = real_position_y + lidar_distance_right * math.sin(math.radians(real_position_h-60))

        # update map
		origin = np.array([real_position_x,real_position_y])
		end1 = np.array([x1,y1])
		end2 = np.array([x2,y2])

		self.occupacy_map.ray(origin,end1,lidar_distance_left<1.9)
		self.occupacy_map.ray(origin,end2,lidar_distance_right<1.9)
		if self.loop_counter % (60*10) == 0:
				self.occupacy_map.save()

		# real position
		print("real     x:" + str(round(real_position_x,2)) + "m   y:" + str(round(real_position_y,2)) + "m   h:" + str(round(real_position_h,2)) +"deg" )
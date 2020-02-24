import my_odometry


import numpy as np
import math
import random

from my_math import *

class robot_odometry:

	def __init__(self):

		self.ignore_counter = 120 #ignore first frames

		# odometry
		self.odom = my_odometry.odometry( 0.0, 0.0, 0.0)

	def process(
		self,
		dt,
		actual_speed_ms,
		actual_rotation_speed_dps
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

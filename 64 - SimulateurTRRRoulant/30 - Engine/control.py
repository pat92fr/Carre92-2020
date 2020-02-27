import my_controller
from my_math import *
import numpy as np
class robot_controller:

	def __init__(self):

		# speed controller settings
		self.min_speed_ms = 2.0 # 1.0 m/s
		self.cornering_speed = 3.0
		self.max_speed_ms = 10.0 # 10 m/s

		self.acceleration = 0.1 # m/s per 1/60eme
		self.deceleration = 0.5 # m/s per 1/60eme

		self.pid_speed = my_controller.pid(kp=0.3, ki=0.0, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.5) 
		self.pid_speed_kff = 0.0 # feed forward apart from speed PID
		self.steering_k_speed = 0.0

		# speed controller state
		self.target_speed_ms_ref = 0.0 # m/s (square)
		self.target_speed_ms = 0.0 # m/s (square)
		self.current_speed_ms = 0.0 # m/s (trapeze)
		self.actual_speed_ms = 0.0 # m/s from encoder (real)
		self.actual_speed_error_ms = 0.0 # m/s

		# lidar steering controller settings
		self.pid_wall_following = my_controller.pid(kp=0.6, ki=0.0, kd=8.0, integral_max=1000, output_max=1.0, alpha=0.2) 
		self.lidar_direction_k_speed = 0.2
		self.lidar_maximum_distance = 2.0

		# lidar steering controller state
		self.actual_lidar_direction_error = 0.0
		self.pid_wall = 0.0

		# steering controller state
		self.ratio_ai = 1.0

		# steering settings
		self.ration_ai_x1 = 0.2 #
		self.ration_ai_x2 = 0.4 #

		print("Done.")

	# speed strategy
	def max_speed_from_distance(self, distance):
		distance = distance % 110.0
		if distance > 0.0 and distance < 4.0:
			return self.max_speed_ms
		elif distance > 15.0 and distance < 25.0:
			return self.max_speed_ms
		elif distance > 45.0 and distance < 55.0:
			return self.max_speed_ms
		elif distance > 65.0 and distance < 120.0:
			return self.max_speed_ms
		else:
			return self.cornering_speed

	def process(
		self,				
		dt,
		actual_speed_ms,
		total_distance,
		lidar_distance_droit,
		lidar_distance_gauche,
		lidar_distance_haut
		 ):

		# copy 
		self.actual_speed_ms = actual_speed_ms

		# controlled state
		steering = 0.0
		throttle = 0.0

		# speed controller (stage 1)
		self.target_speed_ms_ref = self.max_speed_from_distance(total_distance)

		# wall following PID controller
		lidar_distance_droit = constraint(lidar_distance_droit,0, self.lidar_maximum_distance)
		lidar_distance_gauche = constraint(lidar_distance_gauche,0, self.lidar_maximum_distance)
		self.actual_lidar_direction_error = -constraint(lidar_distance_droit - lidar_distance_gauche, -self.lidar_maximum_distance, self.lidar_maximum_distance)/self.lidar_maximum_distance
		self.pid_wall = self.pid_wall_following.compute(self.actual_lidar_direction_error)

		# blending PID
		self.ratio_ai = 0.0
		if abs(self.actual_lidar_direction_error) < self.ration_ai_x1:
			self.ratio_ai = 0.0
		elif abs(self.actual_lidar_direction_error) > self.ration_ai_x2:
			self.ratio_ai = 1.0
		else:
			self.ratio_ai = ( abs(self.actual_lidar_direction_error) - self.ration_ai_x1 ) / (self.ration_ai_x2-self.ration_ai_x1)
		steering = self.ratio_ai * self.pid_wall + (1.0-self.ratio_ai) * 0.0
		print('+'  * int(self.ratio_ai*10.0))
		steering = constraint(steering, -1.0, 1.0)

		# reduce current speed according lidar positional error
		self.target_speed_ms = self.target_speed_ms_ref - ( self.ratio_ai * self.lidar_direction_k_speed * abs(self.actual_lidar_direction_error) + (1.0 - self.ratio_ai) * 0.0 )*self.target_speed_ms_ref
		self.target_speed_ms -= self.steering_k_speed*abs(steering)*self.target_speed_ms_ref
		self.target_speed_ms = constraint(self.target_speed_ms, self.min_speed_ms,self.target_speed_ms_ref)

		# compute current speed from target and time passing (trapeze)
		if self.current_speed_ms < self.target_speed_ms:
			self.current_speed_ms += self.acceleration
			self.current_speed_ms = min(self.current_speed_ms, self.target_speed_ms)
		if self.current_speed_ms > self.target_speed_ms:
			self.current_speed_ms -= self.deceleration
			self.current_speed_ms = max(self.current_speed_ms, self.target_speed_ms)
		self.current_speed_ms = constraint(self.current_speed_ms, self.min_speed_ms, self.max_speed_ms)
		###print(str(round(self.target_speed_ms,1)) + " m/s  " + str(round(self.current_speed_ms,1)) + " m/s  ")

		# compute throttle according actual_speed
		self.actual_speed_error_ms = self.current_speed_ms-self.actual_speed_ms
		throttle = self.pid_speed.compute(self.actual_speed_error_ms) + self.pid_speed_kff *self.current_speed_ms
		throttle = constraint(throttle, -1.0, 1.0)

		print("target_speed_ms:" + str(self.target_speed_ms) + "m/s     "
			+ "actual_speed_ms:" + str(self.actual_speed_ms) + "m/s     "
			+ "throttle:" + str(throttle) )

		# controlled state
		return steering, throttle

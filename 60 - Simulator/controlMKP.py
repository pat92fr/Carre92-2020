import my_controller
from my_math import *
from trackPR import *

import numpy as np
import math

# helper : return True if the waypoint is near the current position
def is_near_waypoint(x,y,waypoint_x,waypoint_y,distance):
	return math.dist( (x,y), (waypoint_x,waypoint_y) ) < distance


class robot_controller:

	def __init__(self):

		# speed controller settings
		self.min_speed_ms = 0.1 # 0.5 m/s
		self.cornering_speed = 0.2
		self.max_speed_ms = 0.2 # 3.5 m/s

		self.acceleration = 0.05 # m/s per 1/60eme
		self.deceleration = 0.2 # m/s per 1/60eme

		self.pid_speed = my_controller.pid(kp=0.3, ki=0.001, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.1) 
		self.pid_speed_kff = 0.05 # feed forward apart from speed PID
		self.steering_k_speed = 0.2

		# speed controller state
		self.target_speed_ms = 0.0 # m/s (square)
		self.current_speed_ms = 0.0 # m/s (trapeze)
		self.actual_speed_ms = 0.0 # m/s from encoder (real)
		self.actual_speed_kmh = 0.0 # km.h from encoder
		self.actual_speed_error_ms = 0.0 # m/s

		# lidar steering controller settings
		self.pid_wall_following = my_controller.pid(kp=1.0, ki=0.0, kd=10.0, integral_max=1000, output_max=1.0, alpha=0.2) 
		self.lidar_direction_k_speed = 0.0
		self.lidar_maximum_distance = 2.0

		# lidar steering controller state
		self.actual_lidar_direction_error = 0.0
		self.pid_wall = 0.0

		# AI steering controller settings
		self.pid_line_following = my_controller.pid(kp=1.0, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.2) 
		self.ai_direction_k_speed = 0.0
		self.ai_direction_alpha = 0.3

		# AI steering controller state
		self.line_pos_unfiltered = 0.0
		self.line_pos = 0.0
		self.pid_line = 0.0
		self.ratio_ai = 1.0

		# steering settings
		self.ration_ai_x1 = 0.0 #
		self.ration_ai_x2 = 0.1 #

		# AI input
		self.frame = np.zeros((1,90,160,1))

	# speed strategy
	def max_speed_from_distance(self, distance):
		# if distance > 0.0 and distance < 4.0:
		# 	return self.max_speed_ms
		# elif distance > 15.0 and distance < 25.0:
		# 	return self.max_speed_ms
		# elif distance > 45.0 and distance < 55.0:
		# 	return self.max_speed_ms
		# elif distance > 65.0 and distance < 120.0:
		# 	return self.max_speed_ms
		# else:
		# 	return self.cornering_speed
		return self.max_speed_ms

	def frame_update(self,frame):
		self.frame = frame

	def process(
		self,				
		dt,
		actual_speed_ms,
		total_distance,
		lidar_distance
		 ):

		# controlled state
		steering = 0.0
		throttle = 0.0

		# speed controller (stage 1)
		self.target_speed_ms = self.max_speed_from_distance(total_distance)
		self.actual_speed_ms = 0.8*self.actual_speed_ms + 0.24*actual_speed_ms

		# wall following PID controller
		self.actual_lidar_direction_error = 0
		# sum = 0.0
		# weighted = 0.0
		# for angle in range(-135,+135,5):
		# 	key ='LidarCN'+str(angle)
		# 	weighted += angle*lidar_distance[key]
		# 	sum += lidar_distance[key]
		# weighted_average = weighted / sum
		# #####print(weighted_average)

		# # find free sectors
		# threshold = 1.0 #1m
		# sectors = {}
		# for angle in range(-90,+90,5):
		# 	key ='LidarCN'+str(angle)
		# 	if lidar_distance[key] > threshold:
		# 		sectors[key] = 1 # free sector
		# 	else:
		# 		sectors[key] = 0
		# print(sectors)

		# #find group of free sectors (begin,end) 
		# sectors_begin_end = []
		# begin = -1
		# end = -1
		# in_free_sector = False
		# for angle in range(-90,+90,5):
		# 	key ='LidarCN'+str(angle)
		# 	if sectors[key] == 1: # if free sector
		# 		if not in_free_sector:
		# 			begin = angle
		# 			in_free_sector = True
		# 	else:
		# 		if in_free_sector:
		# 			end = angle-5
		# 			in_free_sector = False
		# 			sectors_begin_end.append( (begin,end))
		# if in_free_sector:
		# 	end = 90
		# 	in_free_sector = False
		# 	sectors_begin_end.append( (begin,end))			
		# print(sectors_begin_end)

		# # find largest group of free sectors
		# size = 0
		# begin = 0
		# end = 0
		# for s in sectors_begin_end:
		# 	current_size = s[1]-s[0]
		# 	if current_size>size:
		# 		size=current_size
		# 		begin = s[0]
		# 		end = s[1]
		# print(begin)
		# print(end)

		# sum = 0.0
		# weighted = 0.0
		# for angle in range(begin,end,5):
		# 	key ='LidarCN'+str(angle)
		# 	weighted += angle*lidar_distance[key]
		# 	sum += lidar_distance[key]
		# weighted_average = weighted / sum
		# #####print(weighted_average)

		# self.actual_lidar_direction_error = weighted_average / 45.0

		# for each measure, compute force (x,y)
		F = 2.0
		fx = 0.0
		fy = 0.0
		for angle in range(-135,+135,5):
			key ='LidarCN'+str(angle)
			fx += F / (lidar_distance[key]*lidar_distance[key]) * math.cos(math.radians(angle + 180.0))
			fy += F / (lidar_distance[key]*lidar_distance[key]) * math.sin(math.radians(angle + 180.0))
		fx /= 270.0/5.0
		fy /= 270.0/5.0
		angle_av = math.atan(fy/fx)
		print(str(fy) + ' ' + str(fx))

		self.actual_lidar_direction_error = fy
		self.pid_wall = self.pid_wall_following.compute(self.actual_lidar_direction_error)

		# use CNN
		self.line_pos_unfiltered = 0.0

		# line following PID controller
		self.line_pos = self.line_pos * (1.0-self.ai_direction_alpha) + self.ai_direction_alpha * self.line_pos_unfiltered
		self.pid_line = self.pid_line_following.compute(self.line_pos)

		# blending PID
		self.ratio_ai = 0.0
		if abs(self.actual_lidar_direction_error) < self.ration_ai_x1:
			self.ratio_ai = 0.0
		elif abs(self.actual_lidar_direction_error) > self.ration_ai_x2:
			self.ratio_ai = 1.0
		else:
			self.ratio_ai = ( abs(self.actual_lidar_direction_error) - self.ration_ai_x1 ) / (self.ration_ai_x2-self.ration_ai_x1)
		steering = self.ratio_ai * self.pid_wall + (1.0-self.ratio_ai) * self.pid_line
		print('+'  * int(self.ratio_ai*10.0))
		steering = constraint(steering, -1.0, 1.0)

		# reduce current speed according lidar positional error
		self.target_speed_ms -= ( self.ratio_ai * self.lidar_direction_k_speed * abs(self.actual_lidar_direction_error) + (1.0 - self.ratio_ai) *self.ai_direction_k_speed*abs(self.line_pos_unfiltered) )*self.max_speed_ms 
		self.target_speed_ms -= self.steering_k_speed*abs(steering)*self.max_speed_ms
		self.target_speed_ms = constraint(self.target_speed_ms, self.min_speed_ms, self.max_speed_ms)

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


		# controlled state
		return steering, throttle

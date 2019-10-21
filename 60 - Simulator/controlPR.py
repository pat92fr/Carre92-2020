import my_controller


from my_math import *
from trackPR import *

import numpy as np
import math
import random

# helper : return True if the waypoint (wx,wy) is near the current position (x,y)
def point_to_point_distance(x,y,waypoint_x,waypoint_y,distance):
	return ( (x-waypoint_x)*(x-waypoint_x) + (y-waypoint_y)*(y-waypoint_y) ) < (distance*distance)

def CatmullRomSpline(p0, p1, p2, p3, nPoints=100):
    """ Compute trajectories"""
    P0, P1, P2, P3 = map(np.array, [p0, p1, p2, p3])
    # Calculate t0 to t4
    alpha = 0.7 #0.7
    beta = alpha
    def tj(ti, Pi, Pj):
        xi, yi = Pi
        xj, yj = Pj
        return (((xj- xi)**2 + (yj-yi)**2)**beta)**alpha + ti

    t0 = 0
    t1 = tj(t0, P0, P1)
    t2 = tj(t1, P1, P2)
    t3 = tj(t2, P2, P3)

    # Only calculate points between P1 and P2
    t = np.linspace(t1, t2, nPoints)
    # Reshape so that we can multiply by the points P0 to P3
    # and get a point for each value of t
    t = t .reshape(len(t), 1)

    A1 = (t1 - t) / (t1 - t0) * P0 + (t - t0) / (t1 - t0) * P1
    A2 = (t2 - t) / (t2 - t1) * P1 + (t - t1) / (t2 - t1) * P2
    A3 = (t3 - t) / (t3 - t2) * P2 + (t - t2) / (t3 - t2) * P3
#    print("t:", t)
#    print("A1:", A1)
#    print("A2:", A2)
#    print("A3:", A3)

    B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
    B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
    C  = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2

    #print(">>>>>>>>>>>>>>>>>>>>>>>>>", C)
    return C



class robot_controller:

	def __init__(self):

		# speed controller settings
		self.min_speed_ms = 0.1 # 0.5 m/s
		self.cornering_speed = 0.2
		self.max_speed_ms = 2.0 # 3.5 m/s

		self.acceleration = 0.05 # m/s per 1/60eme
		self.deceleration = 0.2 # m/s per 1/60eme

		self.pid_speed = my_controller.pid(kp=0.3, ki=0.001, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.1) 
		self.pid_speed_kff = 0.05 # feed forward apart from speed PID
		self.steering_k_speed = 0.05

		# speed controller state
		self.target_speed_ms = 0.0 # m/s (square)
		self.current_speed_ms = 0.0 # m/s (trapeze)
		self.actual_speed_ms = 0.0 # m/s from encoder (real)
		self.actual_speed_kmh = 0.0 # km.h from encoder
		self.actual_speed_error_ms = 0.0 # m/s
		self.filtered_actual_speed_ms = 0.0
		self.actual_rotation_speed_dps = 0.0
		self.last_actual_speed_ms = 0.0

		# lidar steering controller settings
		self.pid_wall_following = my_controller.pid(kp=0.05, ki=0.0, kd=0.05, integral_max=1000, output_max=1.0, alpha=0.2) 
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

		# PR
		self.current_waypoint_index = 0


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
		actual_rotation_speed_dps,
		total_distance,
		lidar_distance,
		position_x,
		position_y,
		heading
		 ):

		#print("----")

		# reset controller state
		steering = 0.0
		throttle = 0.0

		# update controller state
		self.last_actual_speed_ms = self.actual_speed_ms
		self.actual_speed_ms = actual_speed_ms
		self.actual_rotation_speed_dps = actual_rotation_speed_dps
		#print(self.actual_speed_ms)
		#print(self.actual_rotation_speed_dps)
		
		# speed controller (stage 1)
		self.target_speed_ms = self.max_speed_from_distance(total_distance)
		self.filtered_actual_speed_ms = 0.8*self.filtered_actual_speed_ms + 0.2*actual_speed_ms

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
		# F = 2.0
		# fx = 0.0
		# fy = 0.0
		# for angle in range(-135,+135,5):
		# 	key ='LidarCN'+str(angle)
		# 	fx += F / (lidar_distance[key]*lidar_distance[key]) * math.cos(math.radians(angle + 180.0))
		# 	fy += F / (lidar_distance[key]*lidar_distance[key]) * math.sin(math.radians(angle + 180.0))
		# fx /= 270.0/5.0
		# fy /= 270.0/5.0
		# angle_av = math.atan(fy/fx)
		# print(str(fy) + ' ' + str(fx))

		# self.actual_lidar_direction_error = fy
		# self.pid_wall = self.pid_wall_following.compute(self.actual_lidar_direction_error)

		# use CNN
		# self.line_pos_unfiltered = 0.0

		# line following PID controller
		# self.line_pos = self.line_pos * (1.0-self.ai_direction_alpha) + self.ai_direction_alpha * self.line_pos_unfiltered
		# self.pid_line = self.pid_line_following.compute(self.line_pos)

		# # blending PID
		# self.ratio_ai = 0.0
		# if abs(self.actual_lidar_direction_error) < self.ration_ai_x1:
		# 	self.ratio_ai = 0.0
		# elif abs(self.actual_lidar_direction_error) > self.ration_ai_x2:
		# 	self.ratio_ai = 1.0
		# else:
		# 	self.ratio_ai = ( abs(self.actual_lidar_direction_error) - self.ration_ai_x1 ) / (self.ration_ai_x2-self.ration_ai_x1)
		# steering = self.ratio_ai * self.pid_wall + (1.0-self.ratio_ai) * self.pid_line
		# print('+'  * int(self.ratio_ai*10.0))
		# steering = constraint(steering, -1.0, 1.0)

		# angle from -pi to -pi
		if heading>180:
			heading -=360 
		if heading<-180:
			heading +=360 
		#print(heading)

		# next waypoint
		target_waypoint_pose = wp_position[self.current_waypoint_index]
		waypoint_x = target_waypoint_pose[0]
		waypoint_y = target_waypoint_pose[1]
		waypoint_heading = target_waypoint_pose[2]

		# interpolation from current position to waypoint
		P1 = [position_x-1.0*math.cos(math.radians(heading)),position_y-1.0*math.sin(math.radians(heading))]
		P2 = [position_x,position_y]
		P3 = [waypoint_x,waypoint_y]
		P4 = [ wp_position[(self.current_waypoint_index+1)%len(wp_position)][0], wp_position[(self.current_waypoint_index+1)%len(wp_position)][1] ]
		#P4 = [waypoint_x+2.0*math.cos(math.radians(waypoint_heading)),waypoint_y+2.0*math.sin(math.radians(waypoint_heading))]
		C = CatmullRomSpline(P1, P2, P3, P4, 10)

		# next interpolated position
		target_x = C[1][0]
		target_y = C[1][1]
		target_heading = math.degrees(math.atan2( target_y-position_y, target_x-position_x ))
		
		# angle error from -pi to -pi
		delta_angle = target_heading-heading
		if delta_angle > 180:
			delta_angle -=360
		if delta_angle < -180:
			delta_angle +=360

		# PID heading
		self.pid_wall = self.pid_wall_following.compute(delta_angle)
		steering = constraint(self.pid_wall, -1.0, 1.0)
		#print("target_heading:" + str(round(target_heading,1)) + "  current_heading:" + str(round(heading,1)) + "  steering:" + str(round((target_heading-heading),1)) )


		# waypoint detection and sequencing
		distance = 1.0
		if point_to_point_distance(position_x,position_y,waypoint_x,waypoint_y,distance):
			self.current_waypoint_index+=1
		if self.current_waypoint_index == len(wp_position):
			self.current_waypoint_index=0
		










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
		self.actual_speed_error_ms = self.current_speed_ms-self.filtered_actual_speed_ms
		throttle = self.pid_speed.compute(self.actual_speed_error_ms) + self.pid_speed_kff *self.current_speed_ms
		throttle = constraint(throttle, -1.0, 1.0)


		# controlled state
		return steering, throttle

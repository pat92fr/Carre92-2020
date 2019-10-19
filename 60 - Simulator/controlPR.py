import my_controller
from my_math import *
from trackPR import *

import numpy as np
import math

# helper : return True if the waypoint (wx,wy) is near the current position (x,y)
def point_to_point_distance(x,y,waypoint_x,waypoint_y,distance):
	return ( (x-waypoint_x)*(x-waypoint_x) + (y-waypoint_y)*(y-waypoint_y) ) < (distance*distance)

def CatmullRomSpline(p0, p1, p2, p3, nPoints=100):
    """ Compute trajectories"""
    P0, P1, P2, P3 = map(np.array, [p0, p1, p2, p3])
    # Calculate t0 to t4
    alpha = 0.9
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


# select anchors : return a list of anchor positions [ (x1,y1), (x2,y2), ..] from global anchor positions, 
# each anchor of the list is in 'distance' range of current position (x,y)
def select_anchors(x,y,distance):
	list = []
	for p in anchor_position:
		if point_to_point_distance(x,y,p[0],p[1],distance):
			list.append( (p[0],p[1]) )
	return list


# primitive for cost function : y=f(x)
# x=0 ==> y=1
# x=a ==> y=0
# x>a ==> y=0
def cost_function_primitive(x,a=0.5):
	return max(0.0,1.0-x/a)
# this primitive is used in order to update the waight of particles.
# the range around an anchor is 50cm



# compute one anchor weight
# from the estimated anchor position, and the list of anchors in range,
# compute the weight using the cost function primitive
def compute_one_plot_weight(x,y,anchors_xy_in_range):
	weight = 0.0
	for p in anchors_xy_in_range:
		weight += cost_function_primitive(math.sqrt( (x-p[0])*(x-p[0]) + (y-p[1])*(y-p[1]) ))
	return weight


# compute one particle weight, based on the list of estimated anchor position and the list of anchors in range
# compute the average weight using the cost function primitive
def compute_one_particles_weight(anchors_xy,anchors_xy_in_range):
		average_weight = 0.0
		if anchors_xy:	
			for p in anchors_xy:
				average_weight += compute_one_plot_weight(p[0],p[1],anchors_xy_in_range)
			average_weight /= len(anchors_xy)		
		return average_weight





class robot_controller:

	def __init__(self):

		# speed controller settings
		self.min_speed_ms = 0.1 # 0.5 m/s
		self.cornering_speed = 0.2
		self.max_speed_ms = 6.0 # 3.5 m/s

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
		self.pid_wall_following = my_controller.pid(kp=0.05, ki=0.0, kd=0.1, integral_max=1000, output_max=1.0, alpha=0.2) 
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
		total_distance,
		lidar_distance,
		position_x,
		position_y,
		heading
		 ):

		# controlled state
		steering = 0.0
		throttle = 0.0

		# speed controller (stage 1)
		self.target_speed_ms = self.max_speed_from_distance(total_distance)
		self.actual_speed_ms = 0.8*self.actual_speed_ms + 0.24*actual_speed_ms


		# process LIDAR point clound to find anchors
		anchors = []
		anchor_radius = 0.1 #m
		threshold_max_distance = 9.99 #m
		# find cluster
		cluster_radius = 0.2 #m
		current_cluster_min_distance = 0.0 #m
		current_cluster_begin_angle = 0.0 # deg
		current_cluster_end_angle = 0.0 # deg
		current_cluster_median_angle = 0.0 # deg
		in_cluster = False
		for angle in range(-135,+135,1):
			key ='LidarCN'+str(angle)
			distance = lidar_distance[key]

			if not in_cluster and (distance < threshold_max_distance): # new cluster detection
				# start new cluster
				in_cluster = True
				current_cluster_min_distance = distance
				current_cluster_begin_angle = angle
				current_cluster_end_angle = angle

			elif in_cluster and (distance >= threshold_max_distance): # end of current cluster
				# regsiter current cluster
				current_cluster_median_angle = (current_cluster_end_angle+current_cluster_begin_angle)/2.0
				anchors.append( (current_cluster_median_angle, current_cluster_min_distance+anchor_radius ))
				in_cluster = False

			elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)<=cluster_radius) : # inside current cluster, new distance
				current_cluster_min_distance = min(distance,current_cluster_min_distance)
				current_cluster_end_angle = angle

			elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)>cluster_radius) : # end of current cluster and new cluster detection
				# regsiter current cluster
				current_cluster_median_angle = (current_cluster_end_angle+current_cluster_begin_angle)/2.0
				anchors.append( (current_cluster_median_angle, current_cluster_min_distance+anchor_radius ))
				in_cluster = False
				# start new cluster
				in_cluster = True
				current_cluster_min_distance = distance
				current_cluster_begin_angle = angle
				current_cluster_end_angle = angle

			elif not in_cluster and (distance >= threshold_max_distance):
				in_cluster = False #nop

		# anchors contain (angle,distance) for each visible anchors (in range 10m)
		#print(anchors)

		# for each particles
		# for each particles
		# for each particles
		particles = []
		particles.append( (position_x,position_y,heading) ) # P0
		particles.append( (position_x+0.1*math.cos(math.radians(heading)),position_y+0.1*math.sin(math.radians(heading)),heading+1.0 ) ) # P1
		particles.append( (position_x+0.2*math.cos(math.radians(heading+90)),position_y+0.2*math.sin(math.radians(heading+90)),heading+2.0 ) ) # P2
		particles.append( (position_x+0.3*math.cos(math.radians(heading+180)),position_y+0.3*math.sin(math.radians(heading+180)),heading-3.0 ) ) # P3
		particles.append( (position_x+0.4*math.cos(math.radians(heading+270)),position_y+0.4*math.sin(math.radians(heading+270)),heading-5.0 ) ) # P4

		# compute the weight of particles and normalize
		weights = []
		weight_sum = 0.0
		for pa in particles:
			# for each particle (x,y,heading), compute the anchor positions (px,py) from observation (distance,heading) and particule curent position (x,y)
			anchors_xy = []
			for a in anchors:
				px = pa[0] + a[1]*math.cos(math.radians(a[0]+pa[2]))
				py = pa[1] + a[1]*math.sin(math.radians(a[0]+pa[2]))
				anchors_xy.append( (px,py) )
			#print(anchors_xy)

			# list the plot plots in range in order to compare estimated plot (x,y) with ground truth plot position (x,y)
			anchors_xy_in_range = select_anchors(pa[0],pa[1],10.0+1.0)
			#print(plots_xy_in_range)

			weight = compute_one_particles_weight(anchors_xy,anchors_xy_in_range)
			weight_sum += weight
			weights.append(weight)

		if weight_sum>0.0:
			weights[:] = [w / weight_sum for w in weights]

		print(weights)







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

		# next waypoint
		target_waypoint_pose = wp_position[self.current_waypoint_index]
		waypoint_x = target_waypoint_pose[0]
		waypoint_y = target_waypoint_pose[1]
		waypoint_heading = target_waypoint_pose[2]

		# interpolation from current position to waypoint
		P1 = [position_x-3.0*math.cos(math.radians(heading)),position_y-3.0*math.sin(math.radians(heading))]
		P2 = [position_x,position_y]
		P3 = [waypoint_x,waypoint_y]
		P4 = [ wp_position[(self.current_waypoint_index+1)%len(wp_position)][0], wp_position[(self.current_waypoint_index+1)%len(wp_position)][1] ]
		#P4 = [waypoint_x+2.0*math.cos(math.radians(waypoint_heading)),waypoint_y+2.0*math.sin(math.radians(waypoint_heading))]
		C = CatmullRomSpline(P1, P2, P3, P4, 20)

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
		self.actual_speed_error_ms = self.current_speed_ms-self.actual_speed_ms
		throttle = self.pid_speed.compute(self.actual_speed_error_ms) + self.pid_speed_kff *self.current_speed_ms
		throttle = constraint(throttle, -1.0, 1.0)


		# controlled state
		return steering, throttle

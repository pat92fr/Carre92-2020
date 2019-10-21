import my_odometry

import numpy as np
import math
import random

from my_math import *
from trackPR import *

# helper : return True if the waypoint (wx,wy) is near the current position (x,y)
def point_to_point_distance(x,y,waypoint_x,waypoint_y,distance):
	return ( (x-waypoint_x)*(x-waypoint_x) + (y-waypoint_y)*(y-waypoint_y) ) < (distance*distance)

# helper : select anchors : return a list of anchor positions [ (x1,y1), (x2,y2), ..] from global anchor positions, 
# each anchor of the list is in 'distance' range of current position (x,y)
def select_anchors(x,y,distance):
	list = []
	for p in anchor_position:
		if point_to_point_distance(x,y,p[0],p[1],distance):
			list.append( (p[0],p[1]) )
	return list

# algorithm : from LIDAR point cloud, return a list of polar coord (angle,distance) for every anchors in range
# hyp : -135,+135, step 1°
# tune anchor radius
# tune max distance
def localize_anchors(lidar_distance):
	# process LIDAR point clound to find anchors
	anchors = []
	anchor_radius = 0.2 #m
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
	return anchors




# helper : primitive for cost function : y=f(x)
# x=0 ==> y=1
# x=a ==> y=0
# x>a ==> y=0
def cost_function_primitive(x,a=0.2): # good result with 0.1, not too bad with 0.2
	return max(0.0,1.0-x/a)
# this primitive is used in order to update the waight of particles.
# the range around an anchor is 50cm

# helper : compute one anchor weight
# from the estimated anchor position, and the list of anchors in range,
# compute the weight using the cost function primitive
def compute_one_plot_weight(x,y,anchors_xy_in_range):
	weight = 0.0
	for p in anchors_xy_in_range:
		weight += cost_function_primitive(math.sqrt( (x-p[0])*(x-p[0]) + (y-p[1])*(y-p[1]) ))
	return weight


# helper : compute one particle weight, based on the list of estimated anchor position and the list of anchors in range
# compute the average weight using the cost function primitive
def compute_one_particles_weight(anchors_xy,anchors_xy_in_range):
		average_weight = 0.0
		if anchors_xy:	
			for p in anchors_xy:
				average_weight += compute_one_plot_weight(p[0],p[1],anchors_xy_in_range)
			average_weight /= len(anchors_xy)		
		return average_weight





class robot_odometry:

	def __init__(self):

		# state
		self.last_actual_speed_ms = 0.0
		self.actual_speed_ms = 0.0

		# anchors (detected)
		self.anchors = []
		self.anchors_xy = []

		# Particles filter : creation of particles
		# from global known start position (approx x,y,heading), create a set of initial particles around
		self.particles_count = 100
		self.particles = []
		self.weights = []
		# # initial pose error
		# self.xy_error = 0.0 #m
		# self.heading_error = 0.0 # deg
		# # create particles
		# for index in range(self.particles_count):
		# 	distance = random.uniform(0.0,self.xy_error)
		# 	heading = start_position[2] + random.uniform(-self.heading_error,self.heading_error)
		# 	angle = random.uniform(0.0,360.0)
		# 	self.particles.append( 
		# 		(

		# 			start_position[0]+distance*math.cos(math.radians(angle)),
		# 			start_position[1]+distance*math.sin(math.radians(angle)),
		# 			heading 
		# 		)
		# 	)
		# 	self.weights.append(0.0) #reset particle weight
		# #print(self.particles)

		# odometry
		self.odom = my_odometry.odometry( start_position[0], start_position[1], start_position[2])

		# log
		self.data_logger = open("log.txt","w")

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

		# update controller state
		self.last_actual_speed_ms = self.actual_speed_ms
		self.actual_speed_ms = actual_speed_ms
		self.actual_rotation_speed_dps = actual_rotation_speed_dps
		#print(self.actual_speed_ms)
		#print(self.actual_rotation_speed_dps)

		# process LIDAR point clound to find anchors
		self.anchors = localize_anchors(lidar_distance)
		# anchors contain (angle,distance) for each anchors in range
		#print(self.anchors)
		#print(len(self.anchors))

		# odometry update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		self.odom.update(self.actual_speed_ms,self.actual_rotation_speed_dps,dt)
		print("ground_truth is x:" + str(round(position_x,2)) + "m   y:" + str(round(position_y,2)) + "m   h:" + str(round(heading,2)) +"deg" )
		self.odom.print()

		# move particles according speeds (v,w)
		# delta_h = self.actual_rotation_speed_dps*dt
		# delta_xy = (self.last_actual_speed_ms+self.actual_speed_ms)*dt/2.0
		# particles = []
		# for pa in self.particles:
		# 	# little error
		# 	xy_error = random.uniform(-3.0*dt,3.0*dt) # 0.1m/s error +/-
		# 	xy_heading_error = random.uniform(-3.0*dt,3.0*dt) # 0.1dps error +/-
		# 	heading_error = random.uniform(-0.01*dt,0.01*dt) # 0.1dps error +/-
		# 	# compute next position
		# 	pa_x = pa[0] + (delta_xy+xy_error)*math.cos(math.radians(pa[2] + delta_h/2.0 + xy_heading_error))
		# 	pa_y = pa[1] + (delta_xy+xy_error)*math.sin(math.radians(pa[2] + delta_h/2.0 + xy_heading_error))
		# 	pa_heading = pa[2] + delta_h + heading_error
		# 	particles.append( (pa_x,pa_y,pa_heading) )
		# self.particles = particles
		# #print(self.particles)

		# spread particles around the pose given by the last odometry update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		# arrange particles in a grid
		# first save cos and sin of heading
		hcos = math.cos(math.radians(self.odom.h))
		hsin = math.sin(math.radians(self.odom.h))
		# fix relative position of paricles
		relative_particle_position = [
			(  0.0,   0.0),
			(  1.0,   0.0), # front
			( -1.0,   0.0), # back
			(  0.0,   1.0), # left
			(  0.0,  -1.0), # right
			(  1.0,   1.0), 
			( -1.0,   1.0), 
			(  1.0,  -1.0), 
			(  1.0,  -1.0), 

			(  2.0,   0.0), # front
			( -2.0,   0.0), # back
			(  0.0,   2.0), # left
			(  0.0,  -2.0), # right
			(  2.0,   2.0), 
			( -2.0,   2.0), 
			(  2.0,  -2.0), 
			(  2.0,  -2.0), 

			(  3.0,   0.0), # front
			( -3.0,   0.0), # back
			(  0.0,   3.0), # left
			(  0.0,  -3.0), # right
			(  3.0,   3.0), 
			( -3.0,   3.0), 
			(  3.0,  -3.0), 
			(  3.0,  -3.0), 								
		]
		scale_particle_position = 0.15 #m
		# compute (x,y) of each particle
		self.particles.clear()
		for rpp in relative_particle_position:
			pax = self.odom.x + scale_particle_position*( hcos*rpp[0] - hsin*rpp[1] )
			pay = self.odom.y + scale_particle_position*( hsin*rpp[0] + hcos*rpp[1] ) 
			pah = self.odom.h
			self.particles.append( (pax,pay,pah) )
		#print(self.particles)

		# compute the weight of particles and normalize

		# TODO : prendre en compte le nombre de plots observés car il ne faut pas diviser par le nombre de plots total sous peine de pénaliser certaines particules.
		# TODO à degub finement !

		weights = []
		weight_sum = 0.0
		for pa in self.particles:
			# for each particle (x,y,heading), compute the anchor positions (px,py) from observation (distance,heading) and particule curent position (x,y)
			self.anchors_xy.clear()
			for a in self.anchors:
				px = pa[0] + a[1]*math.cos(math.radians(a[0]+pa[2]))
				py = pa[1] + a[1]*math.sin(math.radians(a[0]+pa[2]))
				self.anchors_xy.append( (px,py) )
			#print(self.anchors_xy)

			# list the plot plots in range in order to compare estimated plot (x,y) with ground truth plot position (x,y)
			anchors_xy_in_range = select_anchors(pa[0],pa[1],10.0+1.0)
			#print(plots_xy_in_range)

			weight = compute_one_particles_weight(self.anchors_xy,anchors_xy_in_range)
			weight_sum += weight
			weights.append(weight)
		if weight_sum>0.0:
			weights[:] = [w / weight_sum for w in weights]
		self.weights = weights
		print(self.weights)


		# # resample partciles
		# if weight_sum>0.0:
		# 	new_particle_list_index = np.random.choice(
		# 		len(self.particles), 
		# 		self.particles_count, 
		# 		p=self.weights)
		# 	#print("resampling:" + str(new_particle_list_index) )
		# 	# then copy heavy particles
		# 	resampling_particles = []
		# 	for i in new_particle_list_index:
		# 		resampling_particles.append( self.particles[i] )
		# 	self.particles = resampling_particles
		# 	#print(self.particles)

		# centroid
		centroid_x = 0.0
		centroid_y = 0.0
		centroid_h = 0.0
		for pa,w in zip(self.particles,self.weights):
			centroid_x += pa[0]*w
			centroid_y += pa[1]*w
			centroid_h += pa[2]*w
		#centroid_x /= len(self.particles)
		#centroid_y /= len(self.particles)
		#centroid_h /= len(self.particles)
		# compare with ground truth
		print("centroid x:" + str(round(centroid_x,2)) + "  y:" + str(round(centroid_y,2)) + "  h:" + str(round(centroid_h,2)) )

		alpha = 0.1
		beta = 1.0-alpha
		if weight_sum>0.0:
			self.odom.x = self.odom.x*beta + centroid_x*alpha
			self.odom.y = self.odom.y*beta + centroid_y*alpha


		self.data_logger.write(

				str(round(position_x,2)) + ";" +
				str(round(position_y,2)) + ";" +
				str(round(heading,2)) + ";" +

				str(round(self.odom.x,2)) + ";" +
				str(round(self.odom.y,2)) + ";" +
				str(round(self.odom.h,2)) + ";" +


				str(round(centroid_x,2)) + ";" +
				str(round(centroid_y,2)) + ";" +
				str(round(centroid_h,2)) + "\n"
			)

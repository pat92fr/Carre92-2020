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
def select_anchors(x,y,distance,anchor_map):
	list = []
	for p in anchor_map:
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
#TODO vectorise




class robot_odometry:

	def __init__(self):

		# state
		self.last_actual_speed_ms = 0.0
		self.actual_speed_ms = 0.0

		# anchors (detected)
		self.anchors = []
		self.anchors_xy = []

		# Particles filter : creation of particles
		self.particles_count = 100
		self.particles = []
		self.weights = []
		# fix relative position of paricles around (x,y,h) given by odometry
		self.relative_particle_position = []
		for xi in range(-9,+9,1):
			for yi in range(-9,+9,1):
				self.relative_particle_position.append( (xi,yi) ) 
		# self.relative_particle_position.append( (0.0,0.0) )
		# for angle in range(0,360,20):
		# 	for distance in range(1,10,1):
		# 		self.relative_particle_position.append( (0.1*float(distance)*math.cos(math.radians(angle)),0.1*float(distance)*math.sin(math.radians(angle)) ) ) 


		self.scale_particle_position = 0.15 #m
		self.centroid_x = 0.0
		self.centroid_y = 0.0
		self.centroid_h = 0.0

		# odometry
		self.odom = my_odometry.odometry( start_position[0], start_position[1], start_position[2])

		# map (x,y,w) of discovered anchors 
		# (x,y,) for position
		# (,w) for the number of time the anchors as beed merged (quality)
		self.map = []

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
		#print("ground_truth is x:" + str(round(position_x,2)) + "m   y:" + str(round(position_y,2)) + "m   h:" + str(round(heading,2)) +"deg" )
		#self.odom.print()


		# MAP STAGE #####

		# from current position, compute position of anchors : (x,y,h)position X (angle,distance)anchor ==> (x,y)anchor
		self.anchors_xy.clear()
		for a in self.anchors:
			#ax = position_x + a[1]*math.cos(math.radians(a[0]+heading)) # I'm using ground truth position for testing purpose, use odometry at the end
			#ay = position_y + a[1]*math.sin(math.radians(a[0]+heading)) # I'm using ground truth position for testing purpose, use odometry at the end
			ax = self.odom.x + a[1]*math.cos(math.radians(a[0]+self.odom.h)) # I'm using ground truth position for testing purpose, use odometry at the end
			ay = self.odom.y + a[1]*math.sin(math.radians(a[0]+self.odom.h)) # I'm using ground truth position for testing purpose, use odometry at the end
			self.anchors_xy.append( (ax,ay) )

		# merge with map
		new_map = []
		for a in self.anchors_xy:
			ax, ay = a # take each visible anchor, one by one
			merged = False
			for m in self.map:
				mx, my, mw = m # take on anchor from map
				# merge if possible
				if point_to_point_distance(mx,my,ax,ay,0.5): # tune max distance between anchor, tune number of iteration until locking anchor					
					# merge, tune rate
					if mw < 100:
						mx = mx*0.9 + ax*0.1
						my = my*0.9 + ay*0.1
					else:
						mx = mx*0.99 + ax*0.01
						my = my*0.99 + ay*0.01
					mw += 1
					new_map.append( (mx,my,mw) )
					merged = True
			if not merged:
				new_map.append( (ax,ay,0) )
		# then add existing and untouched anchors from previous map
		for m in self.map:
			mx, my, mw = m 
			exist_in_new_map = False
			for n in new_map:
				nx, ny, nw = n # take on anchor from new map	
				if point_to_point_distance(mx,my,nx,ny,0.5):
					exist_in_new_map = True
			if not exist_in_new_map:
				new_map.append( m )
		self.map = new_map
		#print(self.map)
		#print(len(self.map))





		# POSITION TRACKING #####

		# spread particles around the pose given by the last odometry update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		# arrange particles in a grid
		# first save cos and sin of heading
		hcos = math.cos(math.radians(self.odom.h))
		hsin = math.sin(math.radians(self.odom.h))
		# compute (x,y) of each particle
		self.particles.clear()
		for rpp in self.relative_particle_position:
			pax = self.odom.x + self.scale_particle_position*( hcos*rpp[0] - hsin*rpp[1] )
			pay = self.odom.y + self.scale_particle_position*( hsin*rpp[0] + hcos*rpp[1] ) 
			pah = self.odom.h
			self.particles.append( (pax,pay,pah) )
		#print(self.particles)

		# compute the weight of particles and normalize

		# TODO : prendre en compte le nombre de plots observés car il ne faut pas diviser par le nombre de plots total sous peine de pénaliser certaines particules.
		# TODO à degub finement !

		# TODO module le champ de particule 'nombre' et 'écartement' en fonction du poids précédent. Réduire le nombre de particules et serrer sur le centroide/odom lorsque l'estimation est bonne.
		#tODO : voire faire deux groupes de particules (un au niveau de ODOM) et un autre centré sur le dernier centroide pour tracker les dévidations

		# list the plot plots in range in order to compare estimated plot (x,y) with ground truth plot position (x,y)
		anchors_xy_in_range = select_anchors(self.odom.x,self.odom.y,10.0+2.0,self.map)
		#print(plots_xy_in_range)
		
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


			weight = compute_one_particles_weight(self.anchors_xy,anchors_xy_in_range)
			weight_sum += weight
			weights.append(weight)
		if weight_sum>0.0:
			weights[:] = [w / weight_sum for w in weights]
		self.weights = weights
		#print(self.weights)

		# centroid
		if weight_sum>0.0:
			self.centroid_x = 0.0
			self.centroid_y = 0.0
			self.centroid_h = 0.0
			for pa,w in zip(self.particles,self.weights):
				self.centroid_x += pa[0]*w
				self.centroid_y += pa[1]*w
				self.centroid_h += pa[2]*w
			#centroid_x /= len(self.particles)
			#centroid_y /= len(self.particles)
			#centroid_h /= len(self.particles)
			# compare with ground truth
			#print("centroid x:" + str(round(self.centroid_x,2)) + "  y:" + str(round(self.centroid_y,2)) + "  h:" + str(round(self.centroid_h,2)) )

			# update odometr according centroid using filter 
			#alpha = 0.2
			#beta = 1.0-alpha
			#self.odom.x = self.odom.x*beta + self.centroid_x*alpha
			#self.odom.y = self.odom.y*beta + self.centroid_y*alpha
			self.odom.x = self.centroid_x
			self.odom.y = self.centroid_y
		# else:
		# 	self.centroid_x = self.odom.x
		# 	self.centroid_y = self.odom.y
		# 	self.centroid_h = self.odom.h

		self.data_logger.write(

				str(round(position_x,2)) + ";" +
				str(round(position_y,2)) + ";" +
				str(round(heading,2)) + ";" +

				str(round(self.odom.x,2)) + ";" +
				str(round(self.odom.y,2)) + ";" +
				str(round(self.odom.h,2)) + ";" +


				str(round(self.centroid_x,2)) + ";" +
				str(round(self.centroid_y,2)) + ";" +
				str(round(self.centroid_h,2)) + "\n"
			)
		self.data_logger.flush()


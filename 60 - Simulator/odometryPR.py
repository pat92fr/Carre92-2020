import my_odometry

import numpy as np
import math
import random

from my_math import *
from trackPR import *

# helper : return True if the waypoint (wx,wy) is near the current position (x,y)
def point_to_point_distance(x,y,waypoint_x,waypoint_y,distance):
	return ( (x-waypoint_x)*(x-waypoint_x) + (y-waypoint_y)*(y-waypoint_y) ) < (distance*distance)

# helper : select landmarks : return a list of anchor positions [ (x1,y1), (x2,y2), ..] from global anchor positions,
# each anchor of the list is in 'distance' range of current position (x,y)
def select_landmarks(x,y,distance,anchor_map):
	list = []
	for p in anchor_map:
		if point_to_point_distance(x,y,p[0],p[1],distance):
			list.append( (p[0],p[1]) )
	return list

# algorithm : from LIDAR point cloud, return a list of polar coord (angle,distance) for every landmarks in range
# hyp : -135,+135, step 1°
# tune anchor radius
# tune max distance
def localize_landmarks(lidar_distance):
	# process LIDAR point clound to find landmarks
	landmarks = []
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
			landmarks.append( (current_cluster_median_angle, current_cluster_min_distance+anchor_radius ))
			in_cluster = False

		elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)<=cluster_radius) : # inside current cluster, new distance
			current_cluster_min_distance = min(distance,current_cluster_min_distance)
			current_cluster_end_angle = angle

		elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)>cluster_radius) : # end of current cluster and new cluster detection
			# regsiter current cluster
			current_cluster_median_angle = (current_cluster_end_angle+current_cluster_begin_angle)/2.0
			landmarks.append( (current_cluster_median_angle, current_cluster_min_distance+anchor_radius ))
			in_cluster = False
			# start new cluster
			in_cluster = True
			current_cluster_min_distance = distance
			current_cluster_begin_angle = angle
			current_cluster_end_angle = angle

		elif not in_cluster and (distance >= threshold_max_distance):
			in_cluster = False #nop
	return landmarks




# helper : primitive for cost function : y=f(x)
# x=0 ==> y=1
# x=a ==> y=0
# x>a ==> y=0
def cost_function_primitive(x,a=0.2): # good result with 0.1, not too bad with 0.2
	return max(0.0,1.0-x/a)
# this primitive is used in order to update the waight of particles.
# the range around an anchor is 50cm

# helper : compute one anchor weight
# from the estimated anchor position, and the list of landmarks in range,
# compute the weight using the cost function primitive
def compute_one_plot_weight(x,y,landmarks_xy_in_range):
	weight = 0.0
	for p in landmarks_xy_in_range:
		weight += cost_function_primitive(math.sqrt( (x-p[0])*(x-p[0]) + (y-p[1])*(y-p[1]) ))
	return weight

# helper : compute one particle weight, based on the list of estimated anchor position and the list of landmarks in range
# compute the average weight using the cost function primitive
def compute_one_particles_weight(landmarks_xy,landmarks_xy_in_range):
		weight = 0.0
		if landmarks_xy:
			for p in landmarks_xy:
				weight += compute_one_plot_weight(p[0],p[1],landmarks_xy_in_range)
			weight /= len(landmarks_xy)
		return weight



class robot_odometry:

	def __init__(self):

		# odometry
		self.odom = my_odometry.odometry( start_position[0], start_position[1], start_position[2])
		self.odom_with_slam = my_odometry.odometry( start_position[0], start_position[1], start_position[2])

		# landmarks (detected)
		self.landmarks = []

		# Particles filter : creation of particles
		self.particles_count = 100
		self.particles = []
		self.weights = []
		# fix relative position of paricles around (x,y,h) given by odometry
		self.relative_particle_position = []
		self.xy_scale_particle_position = 0.15 #m
		self.h_scale_particle_position = 0.1 #m
		for xi in range(-8,+8,1):
			for yi in range(-8,+8,1):
				for hi in range(-1,+1,1):
					self.relative_particle_position.append( (
							xi*self.xy_scale_particle_position,
							yi*self.xy_scale_particle_position,
							hi*self.h_scale_particle_position) 
					)

		# map (x,y,w) of discovered landmarks
		# (x,y,) for position
		# (,w) for the number of time the landmarks as beed merged (quality)
		self.map = []


	def process(
		self,
		dt,
		actual_speed_ms,
		actual_rotation_speed_dps,
		lidar_distance
		 ):

		# stage #1 : odometry
		# update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		self.odom.update(actual_speed_ms,actual_rotation_speed_dps,dt)
		self.odom_with_slam.update(actual_speed_ms,actual_rotation_speed_dps,dt)
		#print("ground_truth is x:" + str(round(position_x,2)) + "m   y:" + str(round(position_y,2)) + "m   h:" + str(round(heading,2)) +"deg" )
		#self.odom.print()
		#self.odom_with_slam.print()


		# stage #2 : landmark extraction (processing LIDAR point clound)
		self.landmarks = localize_landmarks(lidar_distance)
		# landmarks contain (angle,distance) for each landmarks in range
		#print(self.landmarks)
		#print(len(self.landmarks))


		# stage 3 : spread particles around the current position given by the last odometry update (v,w)t X (x,y,h)t-1 => (x,y,h)t
		# first save cos and sin of heading
		hcos = math.cos(math.radians(self.odom_with_slam.h))
		hsin = math.sin(math.radians(self.odom_with_slam.h))
		# compute (x,y) of each particle
		self.particles.clear()
		for rpp in self.relative_particle_position:
			pax = self.odom_with_slam.x + ( hcos*rpp[0] - hsin*rpp[1] )
			pay = self.odom_with_slam.y + ( hsin*rpp[0] + hcos*rpp[1] )
			pah = self.odom_with_slam.h + rpp[2]
			self.particles.append( (pax,pay,pah) )
		#print(self.particles)


		# stage #4 & #5 : compute the weight of particles and normalize
		# list the plot plots in range in order to compare estimated plot (x,y) with ground truth plot position (x,y)
		landmarks_xy_in_range = select_landmarks(self.odom_with_slam.x,self.odom_with_slam.y,10.0+2.0,self.map)
		#print(plots_xy_in_range)
		weights = []
		weight_sum = 0.0
		for pa in self.particles:
			# for each particle (x,y,h), compute the landmark positions (px,py) from observation (angle,distance) and particule current position (x,y,h)
			landmarks_xy = []
			for obs in self.landmarks:
				px = pa[0] + obs[1]*math.cos(math.radians(obs[0]+pa[2]))
				py = pa[1] + obs[1]*math.sin(math.radians(obs[0]+pa[2]))
				landmarks_xy.append( (px,py) )
			#print(self.landmarks_xy)
			weight = compute_one_particles_weight(landmarks_xy,landmarks_xy_in_range)
			weight_sum += weight
			weights.append(weight)
		if weight_sum>0.0:
			weights[:] = [w / weight_sum for w in weights]
		self.weights = weights
		#print(self.weights)
		#print(weight_sum)

		# TODO module le champ de particule 'nombre' et 'écartement' en fonction du poids précédent. Réduire le nombre de particules et serrer sur le centroide/odom lorsque l'estimation est bonne.

		# stage #6 : centroid and odometry correction
		if weight_sum>0.0:
			centroid_x = 0.0
			centroid_y = 0.0
			centroid_h = 0.0
			for pa,w in zip(self.particles,self.weights):
				centroid_x += pa[0]*w
				centroid_y += pa[1]*w
				centroid_h += pa[2]*w
			# compare with ground truth
			#print("centroid x:" + str(round(self.centroid_x,2)) + "  y:" + str(round(self.centroid_y,2)) + "  h:" + str(round(self.centroid_h,2)) )
			# update odometr according centroid using filter
			self.odom_with_slam.x = centroid_x
			self.odom_with_slam.y = centroid_y
			self.odom_with_slam.h = centroid_h			


		# stage #7 : update map
		# from current position, compute position of landmarks : (x,y,h)position X (angle,distance)anchor ==> (x,y)anchor
		landmarks_xy = []
		for obs in self.landmarks:
			#ax = position_x + a[1]*math.cos(math.radians(a[0]+heading)) # I'm using ground truth position for testing purpose, use odometry at the end
			#ay = position_y + a[1]*math.sin(math.radians(a[0]+heading)) # I'm using ground truth position for testing purpose, use odometry at the end
			ax = self.odom_with_slam.x + obs[1]*math.cos(math.radians(obs[0]+self.odom_with_slam.h)) # I'm using ground truth position for testing purpose, use odometry at the end
			ay = self.odom_with_slam.y + obs[1]*math.sin(math.radians(obs[0]+self.odom_with_slam.h)) # I'm using ground truth position for testing purpose, use odometry at the end
			landmarks_xy.append( (ax,ay) )
		# merge with map
		new_map = []
		for a in landmarks_xy:
			ax, ay = a # take each visible anchor, one by one
			merged = False
			for m in self.map:
				mx, my, mw = m # take on anchor from map
				# merge if possible
				if point_to_point_distance(mx,my,ax,ay,1.2): # tune max distance between anchor, tune number of iteration until locking anchor
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
		# then add existing and untouched landmarks from previous map
		for m in self.map:
			mx, my, mw = m
			exist_in_new_map = False
			for n in new_map:
				nx, ny, nw = n # take on anchor from new map
				if point_to_point_distance(mx,my,nx,ny,1.2):
					exist_in_new_map = True
			if not exist_in_new_map:
				new_map.append( m )
		self.map = new_map
		print(self.map)
		print(len(self.map))


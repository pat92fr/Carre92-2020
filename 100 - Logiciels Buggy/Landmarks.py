import math

# algorithm : from LIDAR point cloud, return a list of polar coord (angle,distance) for every landmarks in range
# hyp : -135,+135, step 1°
# tune anchor radius
# tune max distance
def localize_landmarks(lidar_distance):
	# process LIDAR point clound to find landmarks
	landmarks = []
	anchor_radius = 0.05 #m
	threshold_max_distance = 9.5 #m
	# find cluster
	cluster_radius = 0.4 #m (at least 1m between landmarks)
	current_cluster_min_distance = 0.0 #m
	current_cluster_min_angle = 0.0 # deg
	current_cluster_begin_angle = 0.0 # deg
	current_cluster_end_angle = 0.0 # deg
	# TODO : use begin and end angles to check if this cluster is a landmark (radius and distance do an angle)
	in_cluster = False
	for i in range(lidar_distance.shape[0]):
		angle = math.degrees(lidar_distance[i,0])
		distance = lidar_distance[i,1]/1000.0

		if not in_cluster and (distance < threshold_max_distance): # new cluster detection
			# start new cluster
			in_cluster = True
			current_cluster_min_distance = distance
			current_cluster_min_angle = angle
			current_cluster_begin_angle = angle
			current_cluster_end_angle = angle

		elif in_cluster and (distance >= threshold_max_distance): # end of current cluster
			# regsiter current cluster
			if( current_cluster_min_distance*math.sin(math.radians(current_cluster_end_angle-current_cluster_begin_angle)) <= 2.0*anchor_radius):
				landmarks.append( (current_cluster_min_angle, current_cluster_min_distance+anchor_radius ))
			in_cluster = False

		elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)<=cluster_radius) : # inside current cluster, new distance
			if distance < current_cluster_min_distance:
				current_cluster_min_distance = distance
				current_cluster_min_angle = angle
			elif distance == current_cluster_min_distance:
				current_cluster_min_angle = ( angle + current_cluster_min_angle ) / 2.0
			current_cluster_end_angle = angle

		elif in_cluster and (distance < threshold_max_distance) and (abs(distance-current_cluster_min_distance)>cluster_radius) : # end of current cluster and new cluster detection
			# regsiter current cluster
			if( current_cluster_min_distance*math.sin(math.radians(current_cluster_end_angle-current_cluster_begin_angle)) <= 2.0*anchor_radius):
				landmarks.append( (current_cluster_min_angle, current_cluster_min_distance+anchor_radius ))
			in_cluster = False
			# start new cluster
			in_cluster = True
			current_cluster_min_distance = distance
			current_cluster_min_angle = angle
			current_cluster_begin_angle = angle
			current_cluster_end_angle = angle

		elif not in_cluster and (distance >= threshold_max_distance):
			in_cluster = False #nop
	return landmarks


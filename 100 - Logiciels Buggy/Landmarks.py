import math

# algorithm : from LIDAR point cloud, return a list of polar coord (angle,distance) for every landmarks in range
# hyp : -135,+135, step 1°
# tune anchor radius
# tune max distance


'''
def is_clusterized(a1,  d1,  a2,  d2):
    cluster_distance_criteria = 150
    # if a measure is missing, cluster is ended
    angle_criteria = 0.5
    
    if abs(a1 - a2) > angle_criteria:
        return False
    
    if abs(d1 - )

    return abs(dista)
'''

# return a list of clusters
def localize_landmarks(lidar_distance):
    #initiate variables
    last_distance = 0
    distance = 10000
    cluster_size = 0
    cluster_start_angle = 0
    landmarks = []
    cluster = []
    #minimum size of cluster
    min_cluster_size = 10
    max_cluster_size = 40
    #distance between two points for clustering
    clustering_distance = 10
    
       
        
    for i in range(lidar_distance.shape[0]):
        angle = math.degrees(lidar_distance[i,0])
        last_distance = distance
        distance = lidar_distance[i,1]
        
        #is a cluster detected ? 
        if abs(last_distance - distance) < clustering_distance:
            if cluster_size > 0:
                    #cluster is continued
                    cluster_size = cluster_size + 1
                                
            else:
                #start cluster
                cluster_size = 2
                cluster_start_angle = angle
        
        else:            
            if cluster_size > 0:
                #end of cluster
                # is it big enough ? 
                if cluster_size > min_cluster_size and cluster_size < max_cluster_size:
                    # add cluster to landmark list
                    #landmarks.append( ((angle + cluster_start_angle)/2, distance/1000))
                    
                    landmarks.append(cluster)
                cluster_size = 0
                
    return landmarks
        

def localize_landmarks_old(lidar_distance):
	# process LIDAR point clound to find landmarks
	landmarks = []
	anchor_radius = 0.05 #m
	threshold_max_distance = 9.5 #m
	# find cluster
	cluster_radius = 0.5 #m (at least 1m between landmarks)
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
			if( current_cluster_min_distance*math.sin(math.radians(current_cluster_end_angle-current_cluster_begin_angle)) <= 10.0*anchor_radius):
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
			if( current_cluster_min_distance*math.sin(math.radians(current_cluster_end_angle-current_cluster_begin_angle)) <= 10.0*anchor_radius):
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


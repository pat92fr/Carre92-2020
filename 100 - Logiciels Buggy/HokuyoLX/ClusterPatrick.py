
import matplotlib.pyplot as plt
import numpy as np
import math
from sklearn.cluster import DBSCAN

# ax in radians
# dx in meters
# max distance in meters
# return True if the distance between two point in polar coord is lesser than max_distance
def is_near( a1, d1, a2, d2, max_distance ):
	da = a1  - a2
	d3 = d1 * math.sin(da)
	d4 = abs( d1 * math.cos(da) - d2 )
	return (d3*d3+d4*d4) < (max_distance*max_distance)

# return a label (0..n cluster) for each point, or -1
def clustering(raw_data,max_distance,min_points):
	# count elements
	m = raw_data.shape[1]
	# create label array
	labels = np.zeros(m)
	# reset cluster class id
	k = 0
	# process each couple of points
	start_of_cluster_index = 0
	near_test = False
	for index in range(m-1):
		# this point and the next one are in the same cluster ?
		near_test = is_near(
			raw_data[0,index],
			raw_data[1,index],
			raw_data[0,index+1],
			raw_data[1,index+1],
			max_distance )
		if not near_test:
		# no, then
			# close current cluster
			nb_points = index - start_of_cluster_index + 1
			# too small to be a cluster, is parasite
			if nb_points < min_points:
				labels[start_of_cluster_index:index+1] = -1		
			# nice cluster is labeled
			else:
				labels[start_of_cluster_index:index+1] = k
				k += 1 # next cluster class id
			# next cluster start index
			start_of_cluster_index = index+1 
	# process last point
	if not near_test:
		labels[-1] = -1
	else:
		nb_points = (m-1) - start_of_cluster_index + 1
		if nb_points < min_points:
			labels[start_of_cluster_index:-1] = -1		
		else:
			labels[start_of_cluster_index:-1] = k		
	# finish
	return labels

# process each cluster of point cloud, and return the polar coord and the cluster id of landmarks
def find_landmarks(raw_data,labels):
	landmarks = []
	lm_labels = []
	#print(set(labels))
	for k in set(labels):
		if k != -1: #do not process noise
			print(k)
			# find points of the current cluster k
			mask = (labels == k)
			cluster_data = raw_data[:,mask]
			print(cluster_data)
			# compute cone angle
			cone_angle = cluster_data[0,-1]-cluster_data[0,0]
			print(math.degrees(cone_angle))
			# compute nearest point from cluster
			min_index = np.argmin(cluster_data, axis=1)
			print(min_index)
			nearest_point = cluster_data[:, min_index[1] ]
			print(nearest_point)
			# compute width
			width = nearest_point[1] * math.sin(cone_angle)
			
			# accept cluster width < 200 mm and not clipped by dead angle lidar
			if width<200.0 and abs(nearest_point[0])<math.radians(130):
				landmarks.append( (nearest_point[0],nearest_point[1]+width/2.0) )
				print(width)
				lm_labels.append(int(k))


			# suppress landmarks behind nearer landmark by sectors (+10°,-10°) from begin and end angle of nearest cluster
			# debug du premier et dernier cluster
			# check cluster counting point on left and right sides

	return np.array(landmarks),lm_labels


def run():

	# load cloud.txt 
	print("Loading file...")
	file = open("cloud6.txt", "r")
	content = file.read()
	file.close()
	print("Done.")

	# parse file 
	print("Parsing file...")
	lines = content.splitlines()
	m = len(lines)
	print(" m=" + str(m) + " data")
	print("Done.")

	plt.ion()
	ax = plt.subplot(111, projection='polar')
	plot_raw_data = ax.plot([], [], 'ro')[0]
	plot_cluster = []
	for i in range(20):
		plot_cluster.append( ax.plot([], [], '.')[0] )
	colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, 20)]		
	plot_landmarks = ax.plot([], [], 'k*')[0]
	text = plt.text(0, 1, '', transform=ax.transAxes)
	ax.set_rmax(5000)
	ax.grid(True)
	plt.show()
	for l in lines:
		# decode raw data
		fields = l.split(';')
		timestamp = int(fields[0])
		data_size = (len(fields)-1)//2
		###print(data_size)
		raw_data = np.zeros((2,data_size))
		for i in range(data_size):
			raw_data[0,i] = float(fields[i*2+1])
			raw_data[1,i] = float(fields[i*2+2])
		plot_raw_data.set_data(raw_data)
		# processing raw data
		cluster_labels = clustering(raw_data,200.0,5)
		landmarks, lm_labels = find_landmarks(raw_data,cluster_labels)
		# display
		for k in range(20):
			if k == 19:
				mask = (cluster_labels == -1)
				empty = [ [], [] ]
				plot_cluster[k].set_data(empty)
			else:
				if k in lm_labels:
					mask = (cluster_labels == k)
					plot_cluster[k].set_data(raw_data[:,mask])
				else:
					empty = [ [], [] ]
					plot_cluster[k].set_data(empty)
		if len(landmarks)>0:
			plot_landmarks.set_data(landmarks.T)
		text.set_text('t: %d' % timestamp)
		# update plot
		plt.draw()
		plt.pause(0.100)

if __name__ == '__main__':
    run()


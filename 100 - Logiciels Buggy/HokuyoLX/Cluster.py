
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
	return raw_data, labels

	# # from polar to xy
	# xy_data = np.zeros(raw_data.shape)
	# for i in range(raw_data.shape[1]):
	# 	xy_data[0,i] = raw_data[1,i] * math.cos(raw_data[0,i])
	# 	xy_data[1,i] = raw_data[1,i] * math.sin(raw_data[0,i])

	# # DBSCAN
	# clustering = DBSCAN(eps=100, min_samples=2).fit(xy_data.T)
	# labels = clustering.labels_
	# ###print(labels)

	# # Number of clusters in labels, ignoring noise if present.
	# n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
	# n_noise_ = list(labels).count(-1)
	# print("clusters:"+str(n_clusters_) + "    noises:" + str(n_noise_))

	# #from xy to polar
	# polar_data = np.zeros(xy_data.shape)
	# for i in range(xy_data.shape[1]):
	# 	polar_data[0,i] = math.atan2(xy_data[1,i],xy_data[0,i])
	# 	polar_data[1,i] = math.sqrt(xy_data[0,i]*xy_data[0,i] + xy_data[1,i]*xy_data[1,i])
	# return polar_data, labels

def run():

	# load cloud.txt 
	print("Loading file...")
	file = open("cloud1.txt", "r")
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
	plot_raw_data = ax.plot([], [], 'o')[0]
	plot_cluster = []
	for i in range(20):
		plot_cluster.append( ax.plot([], [], '.')[0] )
	colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, 20)]		
	text = plt.text(0, 1, '', transform=ax.transAxes)
	ax.set_rmax(2000)
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
		###plot_raw_data.set_data(raw_data)
		# processing raw data
		cluster_data, cluster_labels = clustering(raw_data,50.0,5)
		for k in range(20):
			if k == 19:
				mask = (cluster_labels == -1)
				###plot_cluster[k].set_data(cluster_data[:,mask])
			else:
				mask = (cluster_labels == k)
				plot_cluster[k].set_data(cluster_data[:,mask])
			
			

		#plot_cluster.set_data(cluster_data)
		text.set_text('t: %d' % timestamp)
		# update plot
		plt.draw()
		plt.pause(0.010)

if __name__ == '__main__':
    run()


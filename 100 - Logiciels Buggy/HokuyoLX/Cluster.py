
import matplotlib.pyplot as plt
import numpy as np
import math
from sklearn.cluster import DBSCAN

def clustering(raw_data):
	# from polar to xy
	xy_data = np.zeros(raw_data.shape)
	for i in range(raw_data.shape[1]):
		xy_data[0,i] = raw_data[1,i] * math.cos(raw_data[0,i])
		xy_data[1,i] = raw_data[1,i] * math.sin(raw_data[0,i])

	# DBSCAN
	clustering = DBSCAN(eps=100, min_samples=2).fit(xy_data.T)
	labels = clustering.labels_
	###print(labels)

	# Number of clusters in labels, ignoring noise if present.
	n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
	n_noise_ = list(labels).count(-1)
	print("clusters:"+str(n_clusters_) + "    noises:" + str(n_noise_))

	#from xy to polar
	polar_data = np.zeros(xy_data.shape)
	for i in range(xy_data.shape[1]):
		polar_data[0,i] = math.atan2(xy_data[1,i],xy_data[0,i])
		polar_data[1,i] = math.sqrt(xy_data[0,i]*xy_data[0,i] + xy_data[1,i]*xy_data[1,i])
	return polar_data, labels

def run():

	# load cloud.txt 
	print("Loading file...")
	file = open("cloud.txt", "r")
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
	plot_raw_data = ax.plot([], [], '.')[0]
	plot_cluster = []
	for i in range(20):
		plot_cluster.append( ax.plot([], [], '.')[0] )
	colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, 20)]		
	text = plt.text(0, 1, '', transform=ax.transAxes)
	ax.set_rmax(10000)
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
		cluster_data, cluster_labels = clustering(raw_data)
		for k in range(20):
			if k == 19:
				mask = (cluster_labels == -1)
				plot_cluster[k].set_data(cluster_data[:,mask])
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


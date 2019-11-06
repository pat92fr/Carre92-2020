
import matplotlib.pyplot as plt
import numpy as np

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
	plot = ax.plot([], [], '.')[0]
	text = plt.text(0, 1, '', transform=ax.transAxes)
	ax.set_rmax(2000)
	ax.grid(True)
	plt.show()
	for l in lines:
		fields = l.split(';')
		timestamp = int(fields[0])
		data_size = (len(fields)-1)//2
		print(data_size)
		data = np.zeros((2,data_size))
		for i in range(data_size):
			data[0,i] = float(fields[i*2+1])
			data[1,i] = float(fields[i*2+2])
		plot.set_data(data)
		text.set_text('t: %d' % timestamp)
		plt.draw()
		plt.pause(0.001)




if __name__ == '__main__':
    run()


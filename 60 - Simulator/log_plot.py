import matplotlib.pyplot as plt
import numpy as np

# load log.txt 
print("Loading log file...")
log_file = open("log.txt", "r")
content = log_file.read()
log_file.close()
print("Done.")


# parse label.txt 
print("Parsing log file...")
lines = content.splitlines()
m = len(lines)
print(" m=" + str(m) + " data")
print("Done.")

x = np.zeros(m)
y = np.zeros(m)
h = np.zeros(m)
ox = np.zeros(m)
oy = np.zeros(m)
oh = np.zeros(m)
px = np.zeros(m)
py = np.zeros(m)
ph = np.zeros(m)
counter = 0
for l in lines:
	fields = l.split(';')
	x[counter] = fields[0]
	y[counter] = fields[1]
	h[counter] = fields[2]

	ox[counter] = fields[3]
	oy[counter] = fields[4]
	oh[counter] = fields[5]

	px[counter] = fields[6]
	py[counter] = fields[7]
	ph[counter] = fields[8]

	counter += 1

p1=plt.plot(x,y,'g:')
p2=plt.plot(ox,oy,'b-')
p3=plt.plot(px,py,'r-')
plt.legend(["Truth", "Odometry","MonteCarlo"] )
plt.title("Path")  
plt.xlabel('x')
plt.ylabel('y')
plt.show()
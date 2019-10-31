import matplotlib.pyplot as plt
import numpy as np
import re


def CatmullRomSpline(P, alpha = 0.7, nPoints=100):
    """ Compute trajectories"""
    # Calculate t0 to t4
    beta = alpha
    def tj(ti, Pi, Pj):
        xi = Pi[0]
        yi = Pi[1]
        xj = Pj[0]
        yj = Pj[1]
        return (((xj- xi)**2 + (yj-yi)**2)**beta)**alpha + ti

    t0 = 0
    t1 = tj(t0, P[0], P[1])
    t2 = tj(t1, P[1], P[2])
    t3 = tj(t2, P[2], P[3])

    # Only calculate points between P1 and P2
    t = np.linspace(t1, t2, nPoints)
    # Reshape so that we can multiply by the points P0 to P3
    # and get a point for each value of t
    t = t .reshape(len(t), 1)

    A1 = (t1 - t) / (t1 - t0) * P[0] + (t - t0) / (t1 - t0) * P[1]
    A2 = (t2 - t) / (t2 - t1) * P[1] + (t - t1) / (t2 - t1) * P[2]
    A3 = (t3 - t) / (t3 - t2) * P[2] + (t - t2) / (t3 - t2) * P[3]
    B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2
    B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
    C  = (t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2
    return C


# load wp.txt 
print("Loading waypoint file...")
wp_file = open("wp.txt", "r")
content = wp_file.read()
wp_file.close()
print("Done.")

# parse wp.txt 
print("Parsing waypoint file...")
lines = content.splitlines()
m = len(lines)
print(" lines=" + str(m))
print("Done.")

# convert text to matrix
# m lines (x,y,v,alpha)
wp = np.zeros( (0,4) )
for l in lines:
	fields = re.split(' +',l)
	if fields[0]=='WP':
		wpi = np.array( [float(fields[1]),float(fields[2]),float(fields[3]),float(fields[4])] )
		wp = np.vstack( (wp,wpi) )
m = wp.shape[0]
print( str(m) + " waypoints")
#print(wp)

# interpolate
iwp = np.zeros( (0,2) )
for index in range(m):
		P = np.array( 
			[ 
				wp[(index-1)%m,0:2],
				wp[(index+0)%m,0:2],
				wp[(index+1)%m,0:2],
				wp[(index+2)%m,0:2] 
			]
		)
		C = CatmullRomSpline(P, alpha = wp[index,3], nPoints = 100)
		iwp =np.vstack( (iwp,C) )
im = iwp.shape[1]
print( str(im) + " interpolated waypoints")
print(iwp)

im = plt.imread("track.png")
implot = plt.imshow(im, extent=[-20, +20, -20, +20])
p1=plt.plot(wp[:,0],wp[:,1],'ko')
p2=plt.plot(iwp[:,0],iwp[:,1],'b:')
plt.legend(["WP", "Interpolated"] )
plt.title("Path")  
plt.xlabel('x')
plt.ylabel('y')
plt.show()


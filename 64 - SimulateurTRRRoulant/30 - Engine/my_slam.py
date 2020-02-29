import math
import numpy as np
import my_occupancy_map

# pose is (x,y,h) (m,m,degres)
# slam computes next pose from initial/current pose and speeds/dtime/observations
class slam:

	# save initial pose
	def __init__(self,x,y,h,map):
		self.x = x # pose
		self.y = y
		self.h = h
		self.occupacy_map = map

		# compute next pose from speed/dtime (m/s,dps,s)
	def update(
			self,
			speed_ms,
			speed_dps,
			dt,
			lidar_distance_left,
			lidar_distance_right
		):

		# stage 1: last position -> estimate new position without observation (odometry algorihm)


		# compute delta distance & delta heading
		delta_h = speed_dps*dt
		delta_xy = speed_ms*dt	
		# add delta to current position
		self.x += delta_xy*math.cos(math.radians(self.h + delta_h/2.0))
		self.y += delta_xy*math.sin(math.radians(self.h + delta_h/2.0))
		self.h += delta_h

		# stage 2: observe from estimated position and compare to observation from map
		x = self.x
		y = self.y
		h = self.h
		x1 = x + lidar_distance_left * math.cos(math.radians(h+60))
		y1 = y + lidar_distance_left * math.sin(math.radians(h+60))
		x2 = x + lidar_distance_right * math.cos(math.radians(h-60))
		y2 = y + lidar_distance_right * math.sin(math.radians(h-60))
		origin = np.array([x,y])
		end1 = np.array([x1,y1])
		end2 = np.array([x2,y2])
		p1 = self.occupacy_map.test_ray(origin,end1)
		p2 = self.occupacy_map.test_ray(origin,end2)
		print("slam p1:" + str(round(p1,2)) + "    p2:" + str(round(p2,2)) )

		#debug by tracing position and observation on a picture and save it regularly TODO
		#debug by tracing position and observation on a picture and save it regularly TODO
		#debug by tracing position and observation on a picture and save it regularly TODO
		#debug by tracing position and observation on a picture and save it regularly TODO

	def print(self):
		print("slam x:" + str(round(self.x,2)) + "m   y:" + str(round(self.y,2)) + "m   h:" + str(round(self.h,2)) +"deg" )



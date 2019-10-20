import math

# pose is (x,y,h)
# odometry computes next pose from initial/current pose and speeds/dtime
class odometry:

	# save initial pose
	def __init__(self,x,y,h):
		self.x = x # pose
		self.y = y
		self.h = h
		self.last_speed_ms = 0.0 # linear speed

		# compute next pose from speed/dtime
	def update(self,speed_ms,speed_dps,dt):
		# compute delta heading
		delta_h = speed_dps*dt
		# compute delta distance using average speed
		delta_xy = (self.last_speed_ms+speed_ms)*dt/2.0	
		# add delta
		self.x += delta_xy*math.cos(math.radians(self.h + delta_h/2.0))
		self.y += delta_xy*math.sin(math.radians(self.h + delta_h/2.0))
		self.h += delta_h
		# save speed to do averaging
		self.last_speed_ms = speed_ms

	def print(self):
		print("odometry x:" + str(round(self.x,2)) + "m   y:" + str(round(self.y,2)) + "m   h:" + str(round(self.h,2)) +"deg" )


# tested
# heading very accurate
# position (x,y) has good accuracy (about 0.1m for 100m)
# but when robot slips, error increases dramatically, cant handle that.
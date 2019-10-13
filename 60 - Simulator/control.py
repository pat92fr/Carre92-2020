import my_controller


from keras import models
from keras.models import load_model

class robot_controller:

	def __init__(self):

		# speed controller settings
		self.min_speed = 2.5 # 0.5 m/s
		self.cornering_speed = 3.0
		self.max_speed = 5.0 # 3.5 m/s

		self.acceleration = 0.05 # m/s per 1/60eme
		self.deceleration = 0.2 # m/s per 1/60eme

		self.pid_speed = my_controller.pid(kp=1.0, ki=0.0, kd=0.0, integral_max=1000, output_max=128.0, alpha=0.5) 
		self.pid_speed_kff = 0.0 # feed forward apart from speed PID
		self.steering_k_speed = 0.0

		# speed controller state
		self.target_speed_ms = 0.0 # m/s (square)
		self.current_speed_ms = 0.0 # m/s (trapeze)
		self.actual_speed_ms = 0.0 # m/s from encoder (real)
		self.actual_speed_kmh = 0.0 # km.h from encoder
		self.actual_speed_error_ms = 0.0 # m/s

		# lidar steering controller settings
		self.pid_wall_following = my_controller.pid(kp=1.0, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.2) 
		self.lidar_direction_k_speed = 0.0
		self.lidar_maximum_distance = 2.0

		# lidar steering controller state
		self.lidar_distance_droit = self.lidar_maximum_distance
		self.lidar_distance_gauche = self.lidar_maximum_distance
		self.lidar_distance_haut = self.lidar_maximum_distance
		self.actual_lidar_direction_error = 0.0
		self.pid_wall = 0.0

		# AI steering controller settings
		self.pid_line_following = my_controller.pid(kp=1.0, ki=0.0, kd=0.0, integral_max=1000, output_max=1.0, alpha=0.2) 
		self.ai_direction_k_speed = 0.0
		self.ai_direction_alpha = 0.3

		# AI steering controller state
		self.line_pos_unfiltered = 0.0
		self.line_pos = 0.0
		self.pid_line = 0.0
		self.ratio_ai = 1.0

		# steering settings
		self.steering_trim = 0
		self.dual_rate = 0.5
		self.ration_ai_x1 = 0.3 #
		self.ration_ai_x2 = 0.5 #

		# open model
		# print("Load model from disk ...")
		# model = load_model("model/model.h5")
		# model.summary()
		# print("Done.")

	# speed strategy
	def max_speed_from_distance(self, distance):
		if distance > 0.0 and distance < 4.0:
			return max_speed
		elif distance > 15.0 and distance < 25.0:
			return max_speed
		elif distance > 45.0 and distance < 55.0:
			return max_speed
		elif distance > 65.0 and distance < 120.0:
			return max_speed
		else:
			return cornering_speed

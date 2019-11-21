import time
import serial

class stm32_proxy:

	def __init__(self,device="/dev/ttyUSB0"):
		self.port = serial.Serial(device,baudrate=115200,timeout=1.0)
		self.counter = 0
		self.manual_steering = 0
		self.manual_throttle = 0
		self.auto_steering = 0
		self.auto_throttle = 0
		self.main_state = 0
		self.ai_mode = 0
		self.start_countdown = 0
		self.tachymeter_pulse_count = 0
		self.gyro_heading = 0
		self.tachymeter_pulse_period_5us = 0
		self.gyro_dps = 0
		self.gyro_bias = 0
		self.last_time_receive = 0
		# metric values
		self.total_distance_m = 0.0
		self.actual_speed_mps = 0.0


	def receive(self):
		message_in = self.port.readline()
		fields = message_in.decode('ascii').split(';')
		###print(fields)
		if len(fields)==13:
			self.counter = int(fields[0])
			self.manual_steering = int(fields[1])
			self.manual_throttle = int(fields[2])
			self.auto_steering = int(fields[3])
			self.auto_throttle = int(fields[4])
			self.main_state = int(fields[5])
			self.ai_mode = int(fields[6])
			self.start_countdown = int(fields[7])
			self.tachymeter_pulse_count = int(fields[8])
			self.gyro_heading = float(fields[9])/1000.0
			self.tachymeter_pulse_period_5us = int(fields[10])
			self.gyro_dps = float(fields[11])/1000.0
			self.gyro_bias = float(fields[12])/1000.0		
			self.last_time_receive = time.time_ns()//1000000
			###print(self.last_time_receive)	
			# metric values
			magnet_count = 4.0
			gear_ratio = 2.64
			wheel_perimeter = 0.204
			self.total_distance_m = round( ( float(self.tachymeter_pulse_count) / magnet_count ) / gear_ratio * wheel_perimeter, 2)
			self.actual_speed_mps = round( 200000.0/(magnet_count*float(self.tachymeter_pulse_period_5us+1)) / gear_ratio * wheel_perimeter, 2)
			###print("distance:" + str(round(self.total_distance_m,1)) + "m" )
			###print("speed:" + str(round(self.actual_speed_mps,1)) + "mps" )

              
	def send(self,steering,throttle,mode):
		#message_out =  "{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii')
		self.port.write("{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii'))



			# // STM32 replies to AI by a frame that contains informations :
			# //  - all PWM values [0..255] for DIR/THR from RC and from AI
			# //  - modes AUTO/MANUAL from RC and from AI
			# //  - start countdown
			# //  - actual absolute/total distance in pulses (1/4 main gear rotation)
			# //  - actual heading in 0.001 degrees
			# //  - actual instant x speed in 5us period of last pulses (1/4 main gear rotation)
			# //  - actual instant w speed in 0.001 dps
			# //  - actual gyro bias in 0.001 dps
			# static uint32_t rep_counter = 0;
			# HAL_Serial_Print(&ai_com, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
			# 	rep_counter++,
			# 	pwm_to_int(pwm_manual_dir),
			# 	pwm_to_int(pwm_manual_thr),
			# 	pwm_to_int(pwm_auto_dir),
			# 	pwm_to_int(pwm_auto_thr),
			# 	main_state,
			# 	ai_mode,
			# 	start_countdown,
			# 	(uint32_t)tachymeter_pulse_count,
			# 	(int32_t)(gyro_get_heading()*1000.0),
			# 	tachymeter_pulse_period_5us,
			# 	(int32_t)(gyro_get_dps()*1000.0),
			# 	(int32_t)(gyro_get_bias()*1000.0)
			# );
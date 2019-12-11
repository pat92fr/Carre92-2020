from STM32_proxy import *
from my_odometry import *
# #port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)
# port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=1.0)
from hokuyolx import HokuyoLX
import math

file = open("log_serial.txt","w")

steering = 128.0
throttle = 128.0
mode = 0

# lidar
DMAX = 10000
laser = HokuyoLX()

proxy =  stm32_proxy("/dev/ttyUSB0")
odom = odometry(0.0,0.0,0.0)

# STM32 proxy RX
proxy.receive()


# init x and w speed computation
last_total_distance_m = proxy.total_distance_m
last_heading_deg = proxy.gyro_heading

# loop
while True:

	timestamp, scan = laser.get_filtered_dist(dmax=DMAX)
	file.write(
		"L;" )
	for s in scan:
	        file.write(str(round(s[0], 4))+";"+str(round(s[1], 2))+";" )
    file.write("\n")
    file.flush()	

	# STM32 proxy RX
	proxy.receive()

	# compute x and w speed and update odometry
	dt = 0.01 #100Hz STM32 precision clock
	speed_ms = ( proxy.total_distance_m - last_total_distance_m )  / dt
	speed_dps = ( proxy.gyro_heading - last_heading_deg ) / dt
	last_total_distance_m = proxy.total_distance_m
	last_heading_deg = proxy.gyro_heading
	odom.update(speed_ms,speed_dps,dt)
	odom.print()
	file.write(
		"O;" +
		str(round(dt,3)) +";"+		
		str(round(speed_ms,3)) +";"+
		str(round(speed_dps,3)) +";"+
		str(round(odom.x,2)) +";"+
		str(round(odom.y,2)) +";"+
		str(round(odom.h,2)) +";" + "\n"
	)


	# STM32 proxy TX
	proxy.send(steering,throttle,mode)


	
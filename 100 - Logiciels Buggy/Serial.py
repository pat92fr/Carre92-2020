from STM32_proxy import *
from my_odometry import *
# #port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)
# port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=1.0)

steering = 128.0
throttle = 128.0
mode = 0

proxy =  stm32_proxy("/dev/ttyUSB0")
odom = odometry(0.0,0.0,0.0)

# STM32 proxy RX
proxy.receive()

# init x and w speed computation
last_total_distance_m = proxy.total_distance_m
last_heading_deg = proxy.gyro_heading

# loop
while True:
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


	# STM32 proxy TX
	proxy.send(steering,throttle,mode)


	
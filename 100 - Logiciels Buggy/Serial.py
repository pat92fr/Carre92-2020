from STM32_proxy import *

# #port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)
# port = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout=1.0)

# steering = 128.0
# throttle = 128.0
# mode = 0

# while True:
# 	message_in = port.readline()
# 	print(message_in)
# 	#message_out =  "{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii')
# 	#port.write("{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii'))



proxy =  stm32_proxy("/dev/ttyUSB0")


while True:
	proxy.receive()
	
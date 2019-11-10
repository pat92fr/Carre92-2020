import time
import serial


port = serial.Serial("/dev/ttyS0",baudrate=115200,timeout=3.0)

steering = 128.0
throttle = 128.0
mode = 0

while True:
	time.sleep(1)
	message_out =  "{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii')
	#print(">>>" + str(message_out) )
	port.write("{:d};".format(int(steering)).encode('ascii') + "{:d};".format(int(throttle)).encode('ascii') + "{:d}\r\n".format(mode).encode('ascii'))
	message_in = port.readline()
	print(message_in)


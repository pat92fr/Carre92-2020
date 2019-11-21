##http://ozzmaker.com/using-python-with-a-gps-receiver-on-a-raspberry-pi/

import serial
import math

#port = "/dev/serial0"
port = "/dev/ttyS0"
 
def parseGPS(data):
    #print(data)
    longitude = 0.0
    latitude = 0.0
    altitude = 0.0
    #if data[0:6] == "$GPRMC":
    if data[0:6] == "$GNRMC":
        sdata = data.split(",")
        print(sdata)
        if sdata[2] == 'V':
            print("no satellite data available")
            return longitude,latitude,altitude
        #print("---Parsing GPRMC---")
        time = sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
        lat = decode(sdata[3]) #latitude
        dirLat = sdata[4]      #latitude direction N/S
        lon = decode(sdata[5]) #longitute
        dirLon = sdata[6]      #longitude direction E/W
        speed = sdata[7]       #Speed in knots
        trCourse = sdata[8]    #True course
        date = sdata[9][0:2] + "/" + sdata[9][2:4] + "/" + sdata[9][4:6]#date
        
        print("time : %s, latitude : %s(%s), longitude : %s(%s), speed : %s, True Course : %s, Date : %s" %  (time,lat,dirLat,lon,dirLon,speed,trCourse,date) )

        longitude_fields = lon.split(' ')
        latitude_fields = lat.split(' ')
        longitude = math.radians( float(longitude_fields[0]) + float(longitude_fields[2])/60.0 )
        latitude = math.radians( float(latitude_fields[0]) + float(latitude_fields[2])/60.0 )
    return longitude,latitude,altitude
 
def decode(coord):
    #Converts DDDMM.MMMMM > DD deg MM.MMMMM min
    x = coord.split(".")
    head = x[0]
    tail = x[1]
    deg = head[0:-2]
    min = head[-2:]
    return deg + " deg " + min + "." + tail + " min"
 
 
print("Receiving GPS data")
ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
counter = 0
while True:
	data = ser.readline()
	pointLon, pointLat, pointAlt = parseGPS(data.decode('ascii'))

	if not (pointLon==0.0 and pointLat==0.0):
		#print(pointLon)
		#print(pointLat)
		#print(pointAlt)

		# now XY from my home
		centerLat = math.radians(48.944665)
		centerLon = math.radians(2.326023)
		centerAlt = 0.0
	   
		r = centerAlt + 6378137
		xCenter = r * math.cos(centerLat) * math.cos(centerLon)
		yCenter = r * math.cos(centerLat) * math.sin(centerLon)
		zCenter = r * math.sin(centerLat) 


		r = pointAlt + 6378137
		xPoint = r * math.cos(pointLat) * math.cos(pointLon)
		yPoint = r * math.cos(pointLat) * math.sin(pointLon)
		zPoint = r * math.sin(pointLat) 

		if counter == 0:
			x = (xPoint - xCenter)
			y = (yPoint - yCenter)
			z = (zPoint - zCenter)
		else:
			alpha = 0.1
			beta = 1.0 - alpha
			x = x*beta + alpha*(xPoint - xCenter)
			y = y*beta + alpha*(yPoint - yCenter)
			z = z*beta + alpha*(zPoint - zCenter)

		print(round(x,2))
		print(round(y,2))
		#print(round(x,1))
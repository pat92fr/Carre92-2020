'''Animates distances using single measurment mode'''
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt

import numpy as np
from Landmarks import *
from Telemetry import *

DMAX = 2000 #10000

last_timestamp = 0

def update(laser, plot, plot2, text):
    global last_timestamp
    
    timestamp, scan = laser.get_filtered_dist(dmax=DMAX, grouping=0)
    # plot.set_data(*scan.T)
    # text.set_text('t: %d' % timestamp)
    # plt.draw()
    # plt.pause(0.001)

    ##print(scan.shape)
    ##print(math.degrees(scan[1,0]))

    #text.set_text('t: %d' % timestamp)
    
    if last_timestamp != 0:
        print(timestamp - last_timestamp)
    
    last_timestamp = timestamp
        
    landmarks = localize_landmarks(scan)
    print(len(landmarks))
    print(landmarks)
    
    if landmarks:
        data = np.copy(landmarks)
        for i in range(data.shape[0]):
            data[i,0] = math.radians( data[i,0] )
            data[i,1] *= 1000.0
        plot.set_data(data.T)
        plot2.set_data(scan.T)
        plt.draw()
        plt.pause(0.001)
    
    return landmarks, scan

def run():

    print("Init telemetry server...")
    #tserver = telemetry_server("192.168.1.26", 7001)
    #tserver = telemetry_server("10.3.141.1", 7001)
    tserver = telemetry_server("169.254.106.197", 7001)
    
    counter = 0
    print("Done!")

    plt.ion()
    laser = HokuyoLX()
    ax = plt.subplot(111, projection='polar')
    plot2 = ax.plot([], [], '.')[0]
    plot = ax.plot([], [], 'o')[0]
    
    text = plt.text(0, 1, '', transform=ax.transAxes)
    ax.set_rmax(DMAX)
    ax.grid(True)
    plt.show()
    while plt.get_fignums():
        # Non blocking call
        asyncore.loop(timeout=0, count=1)

        lms, scan = update(laser, plot, plot2,  text)
      
        msg = str(int(counter)) + ';'
        for lm in lms:
            msg += str( float(lm[0]) ) + ';'
            msg += str( float(lm[1]) ) + ';'
        msg += "0"
        #msg = str(int(counter)) + ';'
        #for s in scan:
        #    msg += str( float(s[0]) ) + ';'
        #    msg += str( float(s[1]) ) + ';'
        #msg += "0"

        msg_length = str(len(msg)).ljust(4)
        tserver.sendTelemetry(msg_length)
        tserver.sendTelemetry(msg)
        counter += 1


    laser.close()

if __name__ == '__main__':
    run()

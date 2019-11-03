#-*- coding: utf-8 -*-

########################
# pip install numpy
# pip install matplotlib
########################

# Import
########
import sys
import os
import time
import socket
import datetime
import math
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser
from matplotlib.widgets import Button

# Main procedure
################
def main():
    
    # Check parameters
    ##################
    parser = ArgumentParser()
    parser.add_argument("-p", "--port", dest="port",
                        help="port number", type=int, default=7001)
    parser.add_argument("-d", "--duration", dest="duration",
                        help="max sampling duration in seconds", type=int, default=5)
    parser.add_argument("-i", "--ip", dest="host",
                        help="IP address or host", type=str, default='192.168.1.26') #34
    parser.add_argument("--log",
                        help="Log telemetry in file", action="store_true")
    args = parser.parse_args()

    # Try the connection to the server
    ##################################
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((args.host, args.port))
    except:
        print('Connection #KO# to ' + args.host + ':' + str(args.port))
        sys.exit(0)

    print('Connection OK to ' + args.host + ':' + str(args.port))

    # Create plot context
    plt.ion()
    ax = plt.subplot(111, projection='polar')
    plot = ax.plot([], [], '.')[0]
    ax.set_rmax(10)
    ax.grid(True)
    plt.show()

    # Main while loop
    while plt.get_fignums():
    
        # Wait for length
        for r in range(40):
            data = client.recv(4)
            if len(data) != 0:
                length = int(data.decode("utf-8").replace(' ',''))
                # Wait for the payload
                data = client.recv(length)
                if len(data) != 0:
                    res = data.decode("utf-8").rstrip('\n').rstrip(' ').split(';')
                    m = int( (len(res)-2)/2 )
                    data = np.zeros((m,2))
                    for i in range(0,m):
                        data[i,0] = math.radians(float( res[1+2*i] ))
                        data[i,1] = float( res[2+2*i] )
                    plot.set_data(data.T)
                    print('.')
        
        plt.draw()
        plt.pause(0.001)

# Main
if __name__ == '__main__':    
    main()

# EOF

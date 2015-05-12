#! /usr/bin/python

# Tool for visualizing quaternion as rotated cube

import sys
import math
from ivy.std_api import *
import logging
import getopt

import pygame
import time
import platform
import os

_NAME = 'attitude_viz'

import numpy as np
import matplotlib.pyplot as plt



LAST_DATA=range(0,15)

class Visualization:
    def __init__(self, parent):
         print 'Initialisation visualization'
    def onmsgproc(self, agent, *larg):
        global LAST_DATA
        data = str(larg[0]).split(' ')
        print 'received data' , data, 'And this would mean: ', data[2::]
	print 'LAST_DATA IS NOW: ', LAST_DATA
        LAST_DATA=data[2::]
        print 'RECEIVED LAST DATA: ', LAST_DATA
     
class Visualizer:
    def __init__(self):
        self.visualization = Visualization(self)
        # listen to Ivy
        logging.getLogger('Ivy').setLevel(logging.WARN)
        IvyInit(_NAME,"",0,lambda x, y: y,lambda x, z: z)

        if os.getenv('IVY_BUS') is not None:
            IvyStart(os.getenv('IVY_BUS'))
        else:
            if platform.system() == 'Darwin':
                IvyStart("224.255.255.255:2010")
            else:
                IvyStart()

        # list of all message names
        messages = []
        messages.append("DISTANCE_MATRIX")

        # bind to set of messages (ie, only bind each message once)
        for message_name in set(messages):
            bind_string = "(^.*" + message_name + ".*$)"
            print 'Binding on string: ', bind_string
            IvyBindMsg(self.visualization.onmsgproc, bind_string)

    def OnClose(self):
        IvyStop()



def run():
    
    global LAST_DATA
    window_title = "Sonar_Viz"

   

    visualizer = Visualizer()
    # INITIALISE EVERYTHING HERE BECAUSE PLOTTING CAN ONLY HAPPEN ON THE MAIN THREAD!
    X = range(0,15)
    Y = range(0,15)
    plt.ion()
    graph = plt.plot(X,Y)[0]
    plt.draw()

    try:
        while True:
            time.sleep(.02)

            r = LAST_DATA
            r = (map(abs, map(int,r)))
            theta = np.arange(0,np.pi*2,(2*np.pi)/len(r))
            ax = plt.subplot(111,polar=True)
            ax.clear()
            ax.plot(theta, r, color='r', linewidth=3)
            ax.set_rmax(25.0)
            ax.grid(True)

            plt.draw()

              

    except KeyboardInterrupt:
        print 'Stopping program!'
        visualizer.OnClose()
        return


if __name__ == "__main__":
    run()

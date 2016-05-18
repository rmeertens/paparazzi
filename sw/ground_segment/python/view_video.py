#!/usr/bin/env python

from __future__ import print_function

import sys
from os import path, getenv
import numpy as np
import cv2
import sys
import time
import threading
from os import path, getenv
# from flask import Flask
# # if PAPARAZZI_SRC not set, then assume the tree containing this
# # file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

def message_recv(ac_id, msg):
    print("Received message")

interface = IvyMessagesInterface("VideoOpencvViewer")
cap = cv2.VideoCapture('rtp:0.0.0.0:5000')
cv2.namedWindow('imageframe')
drag_start=None
tracking_state=None
frame=None
selection=None
def onmouse(event, x, y, flags, param):
        global drag_start
        global tracking_state
        global frame
        global selection
        x, y = np.int16([x, y]) # BUG
        # print("On mouse")
        if event == cv2.EVENT_LBUTTONDOWN:
            drag_start = (x, y)
            tracking_state = 0
            return
        if drag_start:
             if flags & cv2.EVENT_FLAG_LBUTTON:
                 h, w = frame.shape[:2]
                 xo, yo = drag_start
                 x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                 x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                 selection = None
                 print("Mouse dragging!")
                 if x1-x0 > 0 and y1-y0 > 0:
                     selection = (x0, y0, x1, y1)
             else:
                 print("Mouse up again")
                 drag_start = None
                 if selection is not None:
                     tracking_state = 1
                     msg2 = PprzMessage("datalink", "VIDEO_SELECTED")
                     msg2['startx'] = selection[0]
                     msg2['starty'] = selection[1]
                     msg2['width'] = selection[2]-selection[0]
                     msg2['height'] = selection[3]-selection[1]
                     msg2['downsized_width'] = frame.shape[0]

                     print("Sending message: %s" % msg2)
                     interface.send(msg2,202)

cv2.setMouseCallback('imageframe', onmouse)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if frame != None:
        cv2.imshow('imageframe',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

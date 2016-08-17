#!/usr/bin/env python
import sys
import time
import threading

from os import path, getenv

# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC")
#PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")

PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage




aircrafts = {}
interface = IvyMessagesInterface("Paparazzi Messages Viewer")
detect_ac_id=15
avoid_ac_id=[16,17]
def message_recv(ac_id, msg):
	if ac_id==detect_ac_id and msg.name=='ROLANDLOC': 
		for rec_ac in avoid_ac_id:
			smsg = PprzMessage("datalink", "MOVE_WP")
			smsg['wp_id'] = 1
			smsg['ac_id'] = rec_ac
			smsg['lat'] = msg['lat']
			smsg['lon'] = msg['lon']
			smsg['alt'] = msg['alt']
			interface.send(smsg)
interface.subscribe(message_recv)


#self.interface.shutdown()

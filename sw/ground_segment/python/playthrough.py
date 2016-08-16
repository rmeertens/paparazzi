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
avoid_ac_id=16
def message_recv(ac_id, msg):
	if ac_id==detect_ac_id and msg.name=='ROLANDLOC': 
	  	#print("Received msg from ", ac_id, " messag: ", msg)
		#print 	dir(msg)
		#print msg.name
		#print msg.msg_id

		#msg = PprzMessage("datalink", "ROLAND_GO_HERE")
		smsg = PprzMessage("datalink", "MOVE_WP")
		smsg['wp_id'] = 1
		smsg['ac_id'] = avoid_ac_id
		smsg['lat'] = msg['lat']
		smsg['lon'] = msg['lon']
		smsg['alt'] = msg['alt']
		
		#print("Sending message: %s" % smsg)
		interface.send(smsg)
#  if ac_id in self.aircrafts and msg.name in self.aircrafts[ac_id].messages:
#    if time.time() - self.aircrafts[ac_id].messages[msg.name].last_seen < 0.2:
#	return

	
interface.subscribe(message_recv)


#self.interface.shutdown()

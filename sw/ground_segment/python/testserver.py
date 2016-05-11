#!/usr/bin/env python
import sys
import time
import threading

from os import path, getenv
from flask import Flask
# if PAPARAZZI_SRC not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_SRC + "/sw/ext/pprzlink/lib/v1.0/python")
PPRZ_HOME = getenv("PAPARAZZI_HOME", PPRZ_SRC)
from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage

from flask import Flask,Response
import json
app = Flask(__name__)


class Message(PprzMessage):
    def __init__(self, class_name, name,msg):
        super(Message, self).__init__(class_name, name)
        self.field_controls = {}
        self.index = None
        self.last_seen = time.clock()
        self.latest_msg = msg 

class Aircraft(object):
    def __init__(self, ac_id):
        self.ac_id = ac_id
        self.messages = {}

aircrafts = {}

@app.route('/knownmessages/<ac_id>')
def get_known_messages(ac_id):
  ac_id = int(ac_id)
  if ac_id in aircrafts:
    knownmessages = []
    for key in aircrafts[ac_id].messages:
      knownmessages.append(key)
    return str(json.dumps(knownmessages))
  return "unknown id"

@app.route('/knownaircrafts')
def get_known_aircrafts():
  knownac = []
  for ac_id in aircrafts:
    knownac.append(ac_id)
  return str(knownac)
  

@app.after_request
def after_request(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE')
    return response

@app.route('/message/<ac_id>/<messagename>')
def get_latest_message(ac_id,messagename):
  # If the message is known, return the latest message
  ac_id = int(ac_id)
  if ac_id in aircrafts:
     if messagename in aircrafts[ac_id].messages:
       return Response(str(aircrafts[ac_id].messages[messagename].latest_msg.to_json()))
     else:
       return "unknown message"
  else:
     return "unknown id"

@app.route("/")
def explain():
    return "TODO: add explanation here!"

def add_new_aircraft(ac_id):
	aircrafts[ac_id] = Aircraft(ac_id)

def add_new_message( aircraft, msg_class, name,msg):
	aircraft.messages[name] = Message(msg_class, name,msg)
interface = IvyMessagesInterface("something")

def message_recv(ac_id, msg):
  if ac_id == 164 or ac_id==1:
    if msg.name== "GPS_INT":
      print "received message"  
      #msg = PprzMessage("ground", "MOVE_WAYPOINT")
      #msg['ac_id'] = ac_id
      #msg['wp_id'] = 1
      #msg['lat'] = 10
      #msg['long'] = 10
      #msg['alt'] = 10
      #print("Sending message: %s" % msg)
      #interface.send(msg)
      msg2 = PprzMessage("datalink", "SOMETHING_DATALINK")
      msg2['hier'] = msg.ecef_x
      msg2['daar'] = msg.ecef_y
      msg2['z'] = msg.ecef_z
      print("Sending message: %s" % msg2)
      interface.send(msg2,1)
      interface.send(msg2,164)
      interface.send(msg2,202)
   
    
  # Possibly add the aircraft to the list
  if ac_id not in aircrafts:
    add_new_aircraft(ac_id)
  aircraft = aircrafts[ac_id]
  # Add the messages and say when last seen
  add_new_message(aircraft, msg.msg_class, msg.name,msg)
  aircrafts[ac_id].messages[msg.name].last_seen = time.time()
  for index in range(0, len(msg.fieldvalues)):
    aircraft.messages[msg.name].field_controls[index]=msg.get_field(index)



#interface = IvyMessagesInterface(message_recv)
interface.subscribe(message_recv)


import zmq
import random
import sys
import time
import json
import math

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:%s" % port)
time.sleep(0.7) # Omdat we anders nog geen connectie hebben. Eerste berichtje(s) gaan verloren richting Zero-MQ

state = {
    # Betekenis van de assen. Zie Readme.md
    'pos': dict(x=0, y=0, z=0, rx=0, ry=0, rz=0) # Set initial position here
}

def inc_pos():
    temp = state['pos']
    temp['x'] += 0.0
    temp['y'] += -0.1
    temp['z'] += 0.0
    temp['rx'] += 0
    temp['ry'] += 0
    temp['rz'] += math.radians(1)
    state['pos'] = temp

def move_init():
    p = state['pos']
    # Initiële waarden
    x = 0.0
    y = 0.0
    z = 0.15
    rx = -90
    ry = 0
    rz = 0
    # Pack
    p['x'] = x
    p['y'] = y 
    p['z'] = z
    p['rx'] = math.radians(rx)
    p['ry'] = math.radians(ry)
    p['rz'] = math.radians(rz)
    s = json.dumps(p)
    print(s)
    socket.send_string(s)


# Move to initial position


move_init()

while False:
    inc_pos()
    p = state['pos']
    s = json.dumps(p)
    print(s)
    socket.send_string(s)
    time.sleep(0.1)


#socket.close()

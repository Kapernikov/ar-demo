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
time.sleep(0.5) # Omdat we anders nog geen connectie hebben. Eerste berichtje(s) gaan verloren richting Zero-MQ

state = {
    # Betekenis van de assen. Zie Readme.md
    'pos': (0.0, 0.0, 2.2, math.radians(-90), math.radians(0), math.radians(180)) # Set initial position here
}

def inc_pos():
    (x,y,z, rx, ry, rz) = state['pos']
    x += 0.0
    y += -0.1
    z += 0.0
    rx += 0
    ry += 0
    rz += math.radians(1)
    state['pos'] = (x,y,z,rx,ry,rz)

def move_init():
    p = state['pos']
    s = json.dumps(p)
    print(s)
    socket.send_string(s)


# Move to initial position
move_init()

while True:
    inc_pos()
    p = state['pos']
    s = json.dumps(p)
    print(s)
    socket.send_string(s)
    time.sleep(0.1)


#socket.close()

import zmq
import random
import sys
import time
import json

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:%s" % port)


state = {
    'pos': (10.0, -10.0, 0.0) # Set initial position here
}

def inc_pos():
    (x,y,z) = state['pos']
    x += 0.0
    y += -0.1
    z += 0.0
    state['pos'] = (x,y,z)

def move_init():
    p = state['pos']
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



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
    'pos': (-1,2)
}

def inc_pos():
    (x,y) = state['pos']
    x += 0.01
    y += 0.01
    state['pos'] = (x,y)


while True:
    inc_pos()
    p = state['pos']
    s = json.dumps(p)
    print(s)
    socket.send_string(s)
    time.sleep(0.1)

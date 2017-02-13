#!/usr/bin/python3

from flask import Flask, send_from_directory, render_template, send_file, g, session, make_response
from flask_socketio import SocketIO, emit
import eventlet.green.zmq as zmq
import eventlet
import json
from functools import wraps, update_wrapper
from datetime import datetime

app = Flask(__name__)
app.secret_key = "boo"
socketio = SocketIO(app)


def nocache(view):
    @wraps(view)
    def no_cache(*args, **kwargs):
        response = make_response(view(*args, **kwargs))
        response.headers['Last-Modified'] = datetime.now()
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, post-check=0, pre-check=0, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '-1'
        return response
        
    return update_wrapper(no_cache, view)


@app.route('/')
def index():
    return send_file('index.html')


@socketio.on('connect')
def start_pushing():
    print("got connect on pos")
    session['running']=True


@socketio.on('disconnect')
def stop_pushing():
    print("disconnecting")
    session['running']=False


state = {
    'pos': (-1,2)
}

def inc_pos():
    (x,y) = state['pos']
    x += 0.01
    y += 0.01
    state['pos'] = (x,y)

def bg_emit():
    inc_pos()
    (x,y) = state['pos']
    socketio.emit('posupdate', dict(x=x,y=y))


def listen():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    #socket.setsockopt(zmq.SUBSCRIBE,"1")
    socket.setsockopt_string(zmq.SUBSCRIBE,'')

    socket.connect ("tcp://localhost:%i" % 5556)
    while True:
        string = socket.recv().decode('utf-8')
        print(string)
        (x,y) = json.loads(string)
        socketio.emit('posupdate', dict(x=x,y=y))


eventlet.spawn(listen)


@app.route("/<path:path>")
@nocache
def send_static(path):
    return send_from_directory('.',path)


if __name__ == "__main__":
    socketio.run(app,host='0.0.0.0',port=5000)


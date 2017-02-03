#!/usr/bin/python3

from flask import Flask, send_from_directory, render_template, send_file,g,session
from flask_socketio import SocketIO, emit
import zmq.green as zmq
import eventlet

app = Flask(__name__)
app.secret_key = "boo"
socketio = SocketIO(app)




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
    while True:
        bg_emit()
        eventlet.sleep(0.07)


eventlet.spawn(listen)


@app.route("/<path:path>")
def send_static(path):
    return send_from_directory('.',path)


if __name__ == "__main__":
    socketio.run(app)


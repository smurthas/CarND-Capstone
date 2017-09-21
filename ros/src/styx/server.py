#!/usr/bin/env python

import socketio
import eventlet
import eventlet.wsgi
import time
import json
import threading
from flask import Flask, render_template
import rospy

from bridge import Bridge
from conf import conf

sio = socketio.Server()
app = Flask(__name__)
bridge = Bridge(conf)
msgs = []

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1
    msgs.append((topic, data))
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

bridge.register_server(send)

publish_i = 0
publish_drop = 2

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable, publish_i, publish_drop
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)

    # only publish 1 out of every publish_drop number of messages
    # bit of a hack to keep the processing load low for this node. otherwise, it
    # will fail to transmit some of the control messages
    publish_i = (publish_i + 1) % publish_drop
    if publish_i == 0:
        bridge.publish_odometry(data)

    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        #sio.emit(topic, data=data, skip_sid=True)
        sio.emit(topic, data=data)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    bridge.publish_camera(data)

def publish_loop():
    rospy.loginfo('publish_loop started')
    rate = rospy.Rate(50) # 50Hz
    while not rospy.is_shutdown():
        if msgs and len(msgs) > 0:
            for i in range(len(msgs)):
                topic, data = msgs.pop(0)
                #sio.emit(topic, data=data, skip_sid=True)
                sio.emit(topic, data=data)
        rate.sleep()


def serve_app(_sio, _app):
    app = socketio.Middleware(_sio, _app)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

if __name__ == '__main__':

    # TODO(smurthas): maybe better to run the server in another thread and
    # publish via a rate loop (e.g. publish_loop) so that this process can use
    # multiple cores?
    #wst = threading.Thread(target=serve_app, args=(sio, app))
    #wst.daemon = True
    #wst.start()
    serve_app(sio, app)

    #publish_loop()

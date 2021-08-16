#!/usr/bin/env python
from flask import Flask, render_template_string, Response
import rospy
from ece3091.msg import CamData
import numpy as np
import cv2
import time

#Most of the code reference from https://blog.miguelgrinberg.com/post/video-streaming-with-flask

INDEX_PAGE = """
<html>
  <head>
    <title>Group 5 Camera Stream</title>
  </head>
  <body>
    <h1>Raw Camera Feed</h1>
    <img src="{{ url_for('video_feed') }}">
  </body>
</html>
"""

app = Flask(__name__)
frame = b""
HEIGHT = None
WIDTH = None
FRAMERATE = None


@app.route('/')
def index():
    return render_template_string(INDEX_PAGE)

def frame_gen():
    global frame
    while True:
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(1/FRAMERATE) #display at 20 frames a sec ish

@app.route('/video_feed')
def video_feed():
    return Response(frame_gen(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def sub_callback(msg):
    global frame
    frame = cv2.imencode('.jpg',
            np.frombuffer(msg.data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3))[1].tobytes()

if __name__ == '__main__':
    #start subscriber
    rospy.init_node('picam_stream', anonymous=False)
    HEIGHT = rospy.get_param('picam/height')
    WIDTH = rospy.get_param('picam/width')
    FRAMERATE = rospy.get_param('picam/framerate')
    rospy.Subscriber('/sensors/picam', CamData, sub_callback)
    #start webserver
    app.run(host='0.0.0.0')


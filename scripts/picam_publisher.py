#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher for the raw pi camera data.

Publishes to:
    /sensors/picam with message format CamData

Written by Josh Cherubino
Last edited 16/08/21 by Josh Cherubino
'''
import rospy
import picamera
import io

#msg type
from ece3091.msg import CamData

#Program variables
#picam resolution. N.B. horizontal resolution must be multiple of 32 and vertical multiple of 16
RESOLUTION = (640, 480) 
RATE = 5 #Hz rate node spins at 

#set values as rosparams so they can be globally accessed in ROS network
rospy.set_param('picam', {'width': RESOLUTION[0], 'height': RESOLUTION[1],
                'framerate': RATE})

camera = picamera.PiCamera(resolution=RESOLUTION)
camera.rotation = 90
camera.start_preview()
stream = io.BytesIO() #IO stream to capture data into

def picam_publisher():
    #small queue to not use too much memory
    pub = rospy.Publisher('/sensors/picam', CamData, queue_size=2)
    #only ever want 1 camera node running so make it non anonymous
    rospy.init_node('picam_publisher', anonymous=False)

    rate = rospy.Rate(RATE) 
    rospy.loginfo('Starting Pi Camera publisher node')

    #register what to do on program shutdown
    rospy.on_shutdown(close)

    while not rospy.is_shutdown():
        #capture to bgr format for opencv compat
        camera.capture(stream, format='bgr') 
        pub.publish(stream.getvalue()) #publish data
        stream.seek(0) #reset stream
        rate.sleep() #wait to run at specified framerate

def close():
    rospy.loginfo('Stopping Pi Camera publisher')
    camera.close()
    stream.close()

if __name__ == '__main__':
    try:
        picam_publisher()
    except rospy.ROSInterruptException:
        pass

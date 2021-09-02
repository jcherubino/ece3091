#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher for the raw distance sensor data.

Publishes to:
    /sensors/distances with message DistanceData

Written by Josh Cherubino
Last edited 29/08/21 by Josh Cherubino
'''

import rospy
import gpiozero
from ece3091.msg import DistanceData

#TODO: Change these values
TRIG_FRONT = 7
TRIG_LEFT = 6
TRIG_RIGHT = 7
ECHO_FRONT = 1
ECHO_LEFT = 10
ECHO_RIGHT = 11

#N.B. gpiozero docs state pigpio is more accurate as it
#uses DMA so will be better at handling precise timings
#NOTE Cannot run for some reasons. Having permission issues so will 
#use standard factory for now
#pigpio_factory = gpiozero.pins.pigpio.PiGPIOFactory()

#set partial to False so we can immediately get readings
SENSOR_KWARGS = dict(max_distance=1.5, #pin_factory=pigpio_factory,
        partial=False)
#try and publish data at the same speed as picam framerate
#otherwise default to 10 Hz
RATE = 50

def distance_publisher():
    pub = rospy.Publisher('/sensors/distances', DistanceData, queue_size=10)
    #can only ever have 1 distance pub
    rospy.init_node('distance_publisher', anonymous=False)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting distance publisher')
    
    front = gpiozero.DistanceSensor(ECHO_FRONT, TRIG_FRONT, **SENSOR_KWARGS)
    #left = gpiozero.DistanceSensor(ECHO_LEFT, TRIG_LEFT, **SENSOR_KWARGS)
    #right = gpiozero.DistanceSensor(ECHO_RIGHT, TRIG_RIGHT, **SENSOR_KWARGS)

    msg = DistanceData()
    while not rospy.is_shutdown():
        #add distance data to message
        #mulitply by 100 to convert to cm.
        msg.front = int(front.distance*100)
        #msg.left = int(left.distance*100)
        #msg.right = int(right.distance*100)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass

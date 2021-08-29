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
TRIG_FRONT_LEFT = 4
TRIG_FRONT_RIGHT = 5
TRIG_BACK_LEFT = 6
TRIG_BACK_RIGHT = 7
ECHO_FRONT_LEFT = 8
ECHO_FRONT_RIGHT = 9
ECHO_BACK_LEFT = 10
ECHO_BACK_RIGHT = 11

#N.B. gpiozero docs state pigpio is more accurate as it
#uses DMA so will be better at handling precise timings
pigpio_factory = gpiozero.pins.pigpio.PiGPIOFactory()

#set partial to False so we can immediately get readings
SENSOR_KWARGS = dict(max_distance=1.5, pin_factory=pigpio_factory,
        partial=False)
#try and publish data at the same speed as picam framerate
#otherwise default to 10 Hz
RATE = rospy.get_param('picam/framerate', 10) 

def distance_publisher():
    pub = rospy.Publisher('/sensors/distances', DistanceData, queue_size=10)
    #can only ever have 1 distance pub
    rospy.init_node('distance_publisher', anonymous=False)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting distance publisher')
    
    fl_sensor = gpiozero.DistanceSensor(ECHO_FRONT_LEFT, TRIG_FRONT_LEFT, **SENSOR_KWARGS)
    fr_sensor = gpiozero.DistanceSensor(ECHO_FRONT_RIGHT, TRIG_FRONT_RIGHT, **SENSOR_KWARGS)
    bl_sensor = gpiozero.DistanceSensor(ECHO_BACK_LEFT, TRIG_BACK_LEFT, **SENSOR_KWARGS)
    br_sensor = gpiozero.DistanceSensor(ECHO_BACK_RIGHT, TRIG_BACK_RIGHT, **SENSOR_KWARGS)

    msg = DistanceData()
    while not rospy.is_shutdown():
        #add distance data to message
        #mulitply by 100 to convert to cm.
        msg.front_left = int(fl_sensor.distance*100)
        msg.front_right = int(fr_sensor.distance*100)
        msg.back_left = int(bl_sensor.distance*100)
        msg.back_right = int(br_sensor.distance*100)

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        distance_publisher()
    except rospy.ROSInterruptException:
        pass

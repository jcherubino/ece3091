#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher for the robot odometry and a service to reset 
the odometry values back to 0. Publishes the distance travelled and orientation of the robot. 
This distance travelled and orientation is relative the midpoint between
the two wheels. Orientation is determine relative to starting position in degrees
with counter clockwise rotation being positive


Publishes to:
    /sensors/odometry with message OdometryData

Offers service 'reset_odometry' with Empty message type that when called
will reset odometry values to 0

Written by Josh Cherubino
Last edited 29/08/21 by Josh Cherubino
'''

import rospy
import gpiozero
import math
from ece3091.msg import OdometryData
from ece3091.srv import Empty, EmptyResponse #empty message type for service call

A_LEFT_PIN = 5
B_LEFT_PIN = 6
A_RIGHT_PIN = 23
B_RIGHT_PIN = 24
GEAR_RATIO  = 344.2
ENCODER_STEPS_PER_REV = 32
STEPS_PER_WHEEL_REV = round(GEAR_RATIO*ENCODER_STEPS_PER_REV)
WHEEL_RADIUS = 5.6/2 #cm 
TURNING_RADIUS = 10.5/2 #cm
RATE = 50

WHEEL_CIRCUMFERENCE = 2*math.pi*WHEEL_RADIUS
TURNING_CIRCLE_CIRCUMFERENCE = 2*math.pi*TURNING_RADIUS

odom = OdometryData()

def handle_odom_reset(req):
    global odom
    odom.distance = 0
    odom.orientation = 0
    rospy.loginfo('Odometry reset')
    return EmptyResponse()

def odometry_node():
    global odom
    pub = rospy.Publisher('/sensors/odometry', OdometryData, queue_size=10)
    #can only ever have 1 encoder pub
    rospy.init_node('odometry_node', anonymous=False)

    #create service to handle resetting odometry
    reset_service = rospy.Service('reset_odometry', Empty, handle_odom_reset)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting odometry node')
    
    #create encoder instances
    encoder_left = gpiozero.RotaryEncoder(A_LEFT_PIN, B_LEFT_PIN, max_steps=STEPS_PER_WHEEL_REV)
    encoder_right = gpiozero.RotaryEncoder(A_RIGHT_PIN, B_RIGHT_PIN, max_steps=STEPS_PER_WHEEL_REV)

    while not rospy.is_shutdown():
        #calculate distance travelled for each encoder
        dist_left = encoder_left.value * WHEEL_CIRCUMFERENCE
        dist_right = encoder_right.value * WHEEL_CIRCUMFERENCE

        #take average of distance delta.
        odom.distance += (dist_left + dist_right)/2.0
        #assume robot rotates about the midpoint between wheels
        left_rot = dist_left/TURNING_CIRCLE_CIRCUMFERENCE*360.0
        right_rot = dist_right/TURNING_CIRCLE_CIRCUMFERENCE*360.0
        odom.orientation += (right_rot - left_rot)/2.0
        #wrap odometry value to always keep within 360 degrees
        #must abs before mod so that we only get magnitude
        odom.orientation = math.copysign(abs(odom.orientation) % 360.0, odom.orientation)

        pub.publish(odom)

        #reset both encoder values to 0 to avoid having to handle wrap-around
        #cases
        encoder_left.value = 0
        encoder_right.value = 0

        rate.sleep()

if __name__ == '__main__':
    try:
        odometry_node()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a service node that when triggerred will actuate the robots
collection mechanism by activating the electromagnet, then rotating the servo
to the desired position and then releasing the electromagnet.

Offers service 'collect' with Trigger message type that when called
will reset odometry values to 0

Written by Josh Cherubino
Last edited 29/08/21 by Josh Cherubino
'''

import rospy
import gpiozero
import time
from std_srvs.srv import Trigger, TriggerResponse

#TODO: Select pins
EM_CONTROL = 4
SERVO_CONTROL = 5
DELAY = 1 #time to wait after turning on/off EM
START_POS = 10 #start position in degrees
END_POS = 170 #end position (where to deposit EM) in degrees
SERVO_RANGE = 180 #max degrees range of servo

#servo_factory = gpiozero.pins.pigpio.PiGPIOFactory()

em = gpiozero.DigitalOutputDevice(EM_CONTROL)
servo = gpiozero.Servo(SERVO_CONTROL)#, factory=servo_factory)

def handle_collect(req):
    rospy.loginfo('Starting collection process')
    #move servo to starting pos
    servo.value = START_POS/SERVO_RANGE
    rospy.loginfo('Servo in starting position')
    #turn on EM
    em.on()
    rospy.loginfo('Activate electromagnet')
    #Wait to grab ball
    time.sleep(DELAY)
    #Rotate servo
    servo.value = END_POS/SERVO_RANGE
    rospy.loginfo('Move to release position')
    #release
    em.off()
    rospy.loginfo('Release ball')
    #wait for ball release
    time.sleep(DELAY)
    #move servo back to starting pos
    servo.value = START_POS/SERVO_RANGE
    rospy.loginfo('Return servo to starting position')
    resp = TriggerResponse()
    resp.success = True
    resp.message = "Collector actuation complete"
    return resp

def collector_node():
    rospy.init_node('collector_node', anonymous=False)

    service = rospy.Service('/actuator/collect', Trigger, handle_collect)

    rospy.loginfo('Starting collector node')
    rospy.spin()
    
if __name__ == '__main__':
    try:
        collector_node()
    except rospy.ROSInterruptException:
        pass

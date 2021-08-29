#!/usr/bin/env python

'''
ECE3091 Group 5

This node interfaces with the motor drivers to control the motors

Publishes to:
    /actuators/motor_cmds

Written by Josh Cherubino
Last edited 29/08/21 by Josh Cherubino
'''

import rospy
import gpiozero
from ece3091.msg import MotorCmd

#TODO: Choose this value
PWM_FREQ = 1000 #hz

#TODO: Set these pins appropriately
PWM_PIN = 10
LEFT_FORWARD = 11
LEFT_REVERSE = 12
RIGHT_FORWARD = 13
RIGHT_REVERSE = 14

class MotorController(object):
    '''
    Class to handle controlling 2 TB6612FNG motor drivers
    to allow control of robot
    '''

    def __init__(self, pwm_pin, left_forward_pin, left_reverse_pin,
            right_forward_pin, right_reverse_pin):
        '''
        Left and Right sides defined as though viewing robot from behind
        looking to front.
        N.B. the 'forward' pins must be high when going forward and 
        'reverse' pins should be high when going in reverse.
        '''
        self.pwm = gpiozero.PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.left_forward = gpiozero.DigitalOutputDevice(left_forward_pin)
        self.left_reverse = gpiozero.DigitalOutputDevice(left_reverse_pin)
        self.right_forward = gpiozero.DigitalOutputDevice(right_forward_pin)
        self.right_reverse = gpiozero.DigitalOutputDevice(right_reverse_pin)

        #define command map
        # 0 --> stop
        # 1 --> forward
        # 2 --> reverse
        # 3 --> left
        # 4 --> right
        self.CMD_MAP = [self.stop, self.forward, self.reverse, self.left, self.right]

    def callback(self, motor_cmd):
        '''
        Callback function that is run with the published MotorCmd instance 
        whenever it is published to /actuators/motor_cmds
        '''
        self.CMD_MAP[motor_cmd.cmd](motor_cmd.speed)
        rospy.loginfo('Recieved command {} with speed {:.2f}'.format(motor_cmd.cmd, motor_cmd.speed))

    def forward(self, speed):
        '''
        Drive robot forward at specified speed
        '''
        #set direction pins
        self.left_forward.on()
        self.left_reverse.off()
        self.right_forward.on()
        self.right_reverse.off()
        self.value = speed

    def reverse(self, speed):
        '''
        Drive robot backward at specified speed
        '''
        #set direction pins
        self.left_forward.off()
        self.left_reverse.on()
        self.right_forward.off()
        self.right_reverse.on()
        self.value = speed

    def stop(self, *args):
        #takes *args for consistent interface
        self.left_forward.off()
        self.left_reverse.off()
        self.right_forward.off()
        self.right_reverse.off()
        self.pwm.value = 0

    def left(self, speed):
        #spin left motor backwards and right motor forwards
        self.left_forward.off()
        self.left_reverse.on()
        self.right_forward.on()
        self.right_reverse.off()
        self.pwm.value = speed

    def right(self, speed):
        #spin right motor backwards and left motor forwards
        self.left_forward.on()
        self.left_reverse.off()
        self.right_forward.off()
        self.right_reverse.on()
        self.pwm.value = speed

def motor_controller_subscriber():

    #create motor controller instance
    motor_controller = MotorController(PWM_PIN, LEFT_FORWARD, LEFT_REVERSE,
        RIGHT_FORWARD, RIGHT_REVERSE)

    #not anonymous. Only allowed 1 motor controller
    rospy.init_node('motor_controller', anonymous=False)
    #register callback as the motor controllers callback which will 
    #set gpio as required
    rospy.Subscriber('/actuators/motor_cmds', MotorCmd, motor_controller.callback)

    #keep script alive and wait for callback to be run until
    #the node is killed
    rospy.spin()

if __name__ == '__main__':
    motor_controller_subscriber()


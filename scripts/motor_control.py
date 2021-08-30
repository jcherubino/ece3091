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
PWM_FREQ = 50000 #hz

#TODO: Set these pins appropriately
PWM_PIN = 13
LEFT_DIR = 27 
RIGHT_DIR = 17

class MotorController(object):
    '''
    Class to handle controlling of motors
    '''

    def __init__(self, pwm_pin, left_dir_pin, right_dir_pin):
        '''
        Left and Right sides defined as though viewing robot from behind
        looking to front.
        N.B. the 'dir' pins must be high when going forward and low when
        in reverse
        '''
        self.pwm = gpiozero.PWMOutputDevice(pwm_pin, frequency=PWM_FREQ)
        self.left_dir = gpiozero.DigitalOutputDevice(left_dir_pin, active_high=True)
        self.right_dir = gpiozero.DigitalOutputDevice(right_dir_pin, active_high=True)

        #define command map
        # 0 --> stop
        # 1 --> forward
        # 2 --> reverse
        # 3 --> cw
        # 4 --> ccw
        self.CMD_MAP = [self.stop, self.forward, self.reverse, self.cw, self.ccw]

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
        self.left_dir.off()
        self.right_dir.off()
        self.pwm.value = speed

    def reverse(self, speed):
        '''
        Drive robot backward at specified speed
        '''
        #set direction pins
        self.left_dir.on()
        self.right_dir.on()
        self.pwm.value = speed

    def stop(self, *args):
        #takes *args for consistent interface
        self.pwm.value = 0

    def cw(self, speed):
        #spin left motor backwards and right motor forwards
        self.left_dir.on()
        self.right_dir.off()
        self.pwm.value = speed

    def ccw(self, speed):
        #spin right motor backwards and left motor forwards
        self.left_dir.off()
        self.right_dir.on()
        self.pwm.value = speed

def motor_controller_subscriber():

    #create motor controller instance
    motor_controller = MotorController(PWM_PIN, LEFT_DIR, RIGHT_DIR)

    #not anonymous. Only allowed 1 motor controller
    rospy.init_node('motor_controller', anonymous=False)
    #register callback as the motor controllers callback which will 
    #set gpio as required
    rospy.Subscriber('/actuators/motor_cmds', MotorCmd, motor_controller.callback)

    rospy.loginfo('Started motor control node')

    #keep script alive and wait for callback to be run until
    #the node is killed
    rospy.spin()

if __name__ == '__main__':
    motor_controller_subscriber()


#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher outputs motor cmds by taking in its odometry data
and the feature map built using sensor data

Publishes to:
    /actuators/motor_cmds

Subscribes to:
    /planner/feature_map
    /sensors/odometry_xy

Written by Josh Cherubino
Last edited 06/09/21 by Josh Cherubino
'''

import rospy
import math
from ece3091.msg import MotorCmd, FeatureMap, OdometryDataXY

RATE = 50
Kp = 0.4
ANGLE_TOL = 2 #tolerate +/- degree of error
DISTANCE_TOL = 1 #tolerate +/- cm of error
MIN_PROXIMITY = 10 #if obstacle within this radius must circumvent.
CIRCUMVENT_TURN = 50 #abs angle to rotate if obstacle detected
CIRCUMVENT_DIST = 20 #distance to travel before attempting to re-orient
CIRCUMVENT_SPEED = 0.1

class PathPlanner(object):
    '''
    Class to plan a path given a feature map and current x y coordintes and orientation.

    Implements a state machine to control what type of movement the robot does
    at a particular instant
    '''
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.target_x = None
        self.target_y = None
        self.obstacle_x = []
        self.obstacle_y = []

        #states for state machine
        self.ALIGN = 0
        self.APPROACH = 1
        self.CIRCUMVENT = 2
        self.ARRIVED = 3
        self.state = self.ALIGN

        #table to map current state to function to execute
        self.STEP_TABLE = [self.align, self.approach, self.circumvent, self.arrived]

        #motor cmds
        self.STOP = 0
        self.FORWARD = 1
        self.REVERSE = 2
        self.CW = 3
        self.CCW = 4

        self.align_target = None #target for alignment value
        self.align_direction = None
        self.circumvent_state = None

    def odom_callback(self, odom_data):
        '''
        Update odom
        '''
        self.x = odom_data.x
        self.y = odom_data.y
        self.orientation = odom_data.orientation

    def fmap_callback(self, fmap):
        '''
        Update feature map
        N.B. targets are in 'global' coordinate space
        and obstacles in robot coordinate space.
        '''
        if fmap.target_x == 0 and fmap.target_y == 0:
            self.target_x, self.target_y = None, None
        else:
            self.target_x = fmap.target_x
            self.target_y = fmap.target_y

        self.obstacle_x = fmap.obstacle_x
        self.obstacle_y = fmap.obstacle_y

    def proportional_control(self, target, current):
        '''
        Proportional speed control function
        that clips output in 0 to 1 range
        '''
        try:
            error = abs(target - current)/target
        except ZeroDivisionError:
            return 0.0
        #apply Kp or clip values if necessary
        return max(0.0, min(Kp*error, 1.0))
    
    def check_align(self):
        '''
        Helper function to check alignment
        Returns align_target, align_direction tuple
        '''
        align_target, _ = self.calculate_align()
        if abs(self.orientation - align_target) > ANGLE_TOL:
            return False
        return True

    def calculate_align(self):
        '''
        Helper function to calculate alignment to target
        '''
        delta_x = abs(self.target_x - self.x)
        delta_y = abs(self.target_y - self.y)
        try:
            align_target =  math.degrees(math.atan(delta_y/delta_x))
        except ZeroDivisionError:
            align_target = 0

        #calculate shorter rotate direction
        theta_cw = abs((self.orientation - align_target) % 360.0)
        theta_ccw = abs((360 - self.orientation + align_target) % 360.0)
        #take shorter path
        if theta_cw < theta_ccw:
            align_direction = self.CW
        else:
            align_direction = self.CCW
        return align_target, align_direction

    def target_distance(self):
        '''
        Helper function to calculate distance to target
        '''
        return math.sqrt((self.x - self.target_x)**2 + (self.y - self.target_y)**2)

    def align(self):
        '''
        Action to take in ALIGN state
        '''
        #N.B. for now align doesn't care what is to either side as there is 
        #only 1 distance sensor connected.
        #attempt to align with current target using rotate commands
        
        #no target 
        if self.target_x is None:
            #for now just do nothing.
            #clean up align state vars
            #self.align_target = None
            #self.align_direction = None
            #self.state = self.ARRIVED
            #rospy.loginfo('No target. Moving to ARRIVED state')
            return MotorCmd(self.STOP, 0)

        if self.align_target is None:
            #calculate alignment
            self.align_target, self.align_direction = self.calculate_align()
            rospy.loginfo('Move to {} orientation in {} direction to align.'.format(self.align_target,
                self.align_direction))

        #once aligned within tolerance advance to APPROACH state
        if abs(self.align_target - self.orientation) < ANGLE_TOL:
            #clean up align state vars
            self.align_target = None
            self.align_direction = None
            rospy.loginfo('Aligned. Moving to APPROACH state')
            self.state = self.APPROACH
            #do nothing this iteration
            return MotorCmd(self.STOP, 0)

        #otherwise must rotate
        speed = self.proportional_control(self.align_target, self.orientation)
        return MotorCmd(self.align_direction, speed)

    def check_arrived(self):
        '''
        Helper method to see if we have arrived at our destination
        Returns True if arrived and False otherwise
        '''
        if abs(self.x - self.target_x) <= DISTANCE_TOL and \
                abs(self.y - self.target_y) <= DISTANCE_TOL:
            return True
        return False

    def check_overshoot(self):
        '''
        Helper to see if we have overshot target
        '''
        EPS = 0.001
        #if we have not overshot the target, then if we move by epsilon
        #in current direction then error should reduce.
        cur_error = self.target_distance()

        #move by epsilon forward along current path
        eps_x = EPS*math.cos(math.radians(self.orientation)) + self.x
        eps_y = EPS*math.sin(math.radians(self.orientation)) + self.y
        eps_error = math.sqrt((eps_x - self.target_x)**2 + (eps_y - self.target_y)**2)

        if eps_error > cur_error:
            return True
        return False

    def approach(self):
        '''
        Action to take in APPROACH state
        Note this doesn't account for overshoot
        '''
        #check if we've arrived
        if self.check_arrived():
            rospy.loginfo('Arrived.')
            self.state = self.ARRIVED
            return MotorCmd(self.STOP, 0)
        
        #check if overshot target
        if self.check_overshoot():
            rospy.logwarn('Target overshot. Re-aligning')
            self.state = self.ALIGN
            return MotorCmd(self.STOP, 0)

        #check if collision imminent. only need to check in front of us.
        for obstacle_dist in self.obstacle_y:
            if obstacle_dist < MIN_PROXIMITY:
                rospy.logwarn('Collision imminent. Moving to circumvent')
                self.state = self.CIRCUMVENT
                return MotorCmd(self.STOP, 0) 

        #check if alignment is off.
        if self.check_align() == False:
            #if we have lost alignment, re-align
            rospy.logwarn('Alignment off. Re-aligning')
            self.state = self.ALIGN
            return MotorCmd(self.STOP, 0)

        #else move in straight line.
        speed_x = self.proportional_control(self.target_x, self.x)
        speed_y = self.proportional_control(self.target_y, self.y)

        return MotorCmd(self.FORWARD, min(Kp, math.sqrt(speed_x**2 + speed_y**2)))
        
    def circumvent(self):
        '''
        Action to take in CIRCUMVENT state
        '''
        #check if we have just entered circumvent state
        if self.circumvent_state is None:
            self.circumvent_state = self.CCW #rotate state
            self.circumvent_delta = 0 #circumvent delta
            self.prev_orientation = self.orientation
            return MotorCmd(self.STOP, 0) 

        #if we are rotating, continue to rotate until we can't see
        #obstacle within stop radius
        if self.circumvent_state == self.CCW:
            #check if we need to continue rotating
            if abs(self.orientation - self.prev_orientation) < CIRCUMVENT_TURN:
                return MotorCmd(self.CCW, CIRCUMVENT_SPEED) 
            #otherwise must now move forward specified distance
            self.circumvent_state = self.FORWARD
            self.circumvent_delta = 0
            self.prev_x = self.x
            self.prev_y = self.y
            return MotorCmd(self.STOP, 0) 

        #update circumvent state to see if we have moved far enough
        delta_x, delta_y = self.x - self.prev_x, self.y - self.prev_y
        self.prev_x, self.prev_y = self.x, self.y
        self.circumvent_delta += math.sqrt(delta_x**2 + delta_y **2)

        #if we have moved far enough, re-align
        if self.circumvent_delta >= CIRCUMVENT_DIST:
            self.state = self.ALIGN
            self.circumvent_state = None 
            rospy.loginfo('Object circumvented. Re-aligning')
            return MotorCmd(self.STOP, 0)

        #otherwise continue in same direction
        return MotorCmd(self.FORWARD, CIRCUMVENT_SPEED)

    def arrived(self):
        '''
        Action to take in arrived state
        '''
        #if self.target_x is not None:
            #new target mve to align
            #self.state = self.ALIGN
            #rospy.loginfo('Obtained target. Moving to align')
        return MotorCmd(self.STOP, 0)

    def step(self):
        '''
        Given current state and available data, execute one 'step' in appropriate
        direction
        '''
        #defer to state callbacks to handle determining next step
        return self.STEP_TABLE[self.state]()

def path_planner_node():
    pub = rospy.Publisher('/actuators/motor_cmds', MotorCmd, queue_size=2)
    rospy.init_node('path_planner_node', anonymous=False)

    planner = PathPlanner() 

    fmap_sub = rospy.Subscriber('/planner/feature_map', FeatureMap, planner.fmap_callback)
    odom_sub = rospy.Subscriber('/sensors/odometry_xy', OdometryDataXY, planner.odom_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting path planner node')
    
    while not rospy.is_shutdown():
        msg = planner.step() #take step in most appropriate direction
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        path_planner_node()
    except rospy.ROSInterruptException:
        pass


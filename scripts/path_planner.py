#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher outputs motor cmds by taking in obstacles
and targets generated by computer vision ndoe

Publishes to:
    /actuators/motor_cmds

Subscribes to:
    /planner/obstacles
    /planner/targets

Written by Josh Cherubino
Last edited 29/09/21 by Josh Cherubino
'''

import rospy
import math
import random
import time
from ece3091.msg import MotorCmd, Obstacles, Targets

RATE = rospy.get_param('/picam/framerate')
SEARCH_SPEED = 1
ALIGN_SPEED = 0.3
APPROACH_SPEED = 0.9
COLLECT_SPEED = 1
CIRCUMVENT_SPEED = 0.3

SEARCH_TURN_LOOPS = 2 #how many loops to turn for in zig-zag search pattern
SEARCH_TURN_FREQ = 12 #how often to turn in zig zag
ALIGN_TOL = 2 #+/- cm alignment tolerance for targets
ARRIVE_TOL = 16
MIN_OBSTACLE_DISTANCE = 18 #object can't be closer than this 
COLLECT_LOOPS = 10 #how many times to 'step' before leaving collect state
SEARCH_WAIT_LOOPS = 5 #how many loops to wait after entering search in case we just
#lost reading for a short amount of time

class PathPlanner(object):
    '''
    Class to plan a path given obstacles and targets in relative xy coordinates to front of robot

    Implements a state machine to control what type of movement the robot does
    at a particular instant
    '''
    def __init__(self):
        self.targets = None
        self.obstacles = None
        self.collect_idx = None
        self.search_idx = None
        self.search_last_turn = 0

        #states for state machine
        self.SEARCH = 0 #no targets. search arena for possible targets
        self.ALIGN = 1 #target found but not in front of robot
        self.APPROACH = 2 #target found and aligned
        self.CIRCUMVENT = 3 #obstacle too close to robot
        self.COLLECT = 4 #target in correct position. Must pick up
        self.state = self.SEARCH #initially searching for target

        #table to map current state to function to execute
        self.STEP_TABLE = [self.search, self.align, self.approach, self.circumvent,
                self.collect]

        #motor cmds
        self.STOP = 0
        self.FORWARD = 1
        self.REVERSE = 2
        self.CW = 3
        self.CCW = 4

        self.search_dir = self.CCW

    def search(self):
        '''
        Search state.
        '''
        if self.collision_imminent():
            rospy.logwarn('Collision imminent. Circumventing')
            self.state = self.CIRCUMVENT
            self.search_idx = None
            self.search_last_turn = 0
            return MotorCmd(self.STOP, 0.0)

        if self.search_idx is None:
            self.search_idx = 0

        #if we have a target
        if self.targets is not None and self.targets.x:
            #move to align state
            rospy.loginfo('Target(s) found ({}, {}). Aligning'.format(self.targets.x, 
                self.targets.y))
            self.state = self.ALIGN
            self.search_idx = None
            self.search_last_turn = 0
            return MotorCmd(self.STOP, 0.0)
        
        #check if we have to keep waiting
        if self.search_idx <= SEARCH_WAIT_LOOPS:
            self.search_idx += 1
            return MotorCmd(self.STOP, 0.0)

        '''
        #drive in zig-zag
        self.search_idx += 1
        rospy.logdebug('Search index: {}'.format(self.search_idx))
        #change direction and turn this way for 1 step
        if self.search_idx % SEARCH_TURN_FREQ == 0:
            #toggle direction
            self.search_dir = self.CCW if self.search_dir == self.CW else self.CW
            self.search_last_turn = self.search_idx
            return MotorCmd(self.search_dir, SEARCH_SPEED)
            
        #first time we are zigging - in this case turn half the distance 
        if self.search_last_turn == SEARCH_TURN_FREQ:
            #keep turning for multiple steps to make larger zig zags
            if self.search_last_turn + SEARCH_TURN_LOOPS > self.search_idx:
                rospy.logdebug('zigging half distance')
                return MotorCmd(self.search_dir, SEARCH_SPEED)

        #otherwise, to make zag put us in opposite direction to zig, we
        #have to turn for twice as long
        if self.search_last_turn + 2*SEARCH_TURN_LOOPS > self.search_idx:
            rospy.logdebug('zigging')
            return MotorCmd(self.search_dir, SEARCH_SPEED)
        
        rospy.logdebug('Searching forward')
        '''
        return MotorCmd(self.FORWARD, SEARCH_SPEED)

    def align(self):
        #N.B. Assume collision can't occur when we rotate on spot so never check
        if self.targets is None or not self.targets.x:
            #target lost
            rospy.loginfo('Target(s) lost. Searching')
            self.state = self.SEARCH
            return MotorCmd(self.STOP, 0.0)
        #check align
        target = self.select_target()
        if self.aligned(target):
            rospy.loginfo('Aligned. Approaching {}'.format(target))
            self.state = self.APPROACH
            return MotorCmd(self.STOP, 0.0)
        
        #If we reach this point. Must continue aligning
        align_dist = target[0]
        if align_dist < 0:
            direction = self.CCW
        else:
            direction =  self.CW

        return MotorCmd(direction, ALIGN_SPEED)

    def aligned(self, target):
        '''
        Helper function to calculate alignment to target
        '''
        #scale alignment requirements depending on how far we are
        #as ratio to what we consider 'arrived'
        tolerance = target[1]/ARRIVE_TOL*ALIGN_TOL
        if abs(target[0]) < tolerance:
            rospy.logdebug('Aligned, alignment: {} with tolerance {}'.format(abs(target[0]),
                tolerance))
            return True
        return False

    def select_target(self):
        '''
        Method to return the selected target the robot will navigate towards
        given its current targets
        '''
        min_distance = float('inf')
        closest_target = None
        #Choose closest target
        for x, y in zip(self.targets.x, self.targets.y):
            dist = self.distance(x, y)
            if dist < min_distance:
                min_distance = dist
                closest_target = (x, y)
        return closest_target

    def approach(self):
        #if we have an obstacle
        if self.collision_imminent():
            rospy.logwarn('Collision imminent. Circumventing')
            self.state = self.CIRCUMVENT
            return MotorCmd(self.STOP, 0.0)
        
        if self.targets is None or not self.targets.x:
            #target lost
            rospy.loginfo('Target(s) lost. Searching')
            self.state = self.SEARCH
            return MotorCmd(self.STOP, 0.0)

        #check arrived
        target = self.select_target()
        if self.arrived(target):
            rospy.loginfo('Arrived. Collecting')
            self.state = self.COLLECT
            return MotorCmd(self.STOP, 0.0)
        
        #check alignment has not shifted
        if not self.aligned(target):
            rospy.logwarn('Alignment lost. Re-aligning')
            self.state = self.ALIGN
            return MotorCmd(self.STOP, 0.0)

        #rospy.logdebug('Y Distance to target : {}'.format(target[1]))
        #Otherwise drive in straight line
        return MotorCmd(self.FORWARD, APPROACH_SPEED)

    def arrived(self, target):
        '''
        Helper function to decide if we have arrived at a target
        '''
        if target[1] <= ARRIVE_TOL:
            rospy.logdebug('Arrived. Target y dist: {}'.format(target[1]))
            return True
        return False

    def circumvent(self):
        if not self.collision_imminent():
            rospy.loginfo('Object avoided. Searching')
            self.state = self.SEARCH
            #invert search dir so next time we zig its in opposite direction
            #will avoid getting stuck in infinite loop with object hopefully
            #self.search_dir = self.CCW if self.search_dir == self.CW else self.CW
            return MotorCmd(self.STOP, 0.0)
        
        #just turn until we can't see the points anymore
        #choose turning direction based on closest obstacle
        min_dist = float('inf')
        direction = None
        for lx, ly, rx, ry in zip(self.obstacles.lx, self.obstacles.ly, 
                self.obstacles.rx, self.obstacles.ry):
            ldist = self.distance(lx, ly)
            rdist = self.distance(rx, ry)
            #only care about closer corner
            rospy.logdebug('ldist: {}. rdist: {}'.format(ldist, rdist))
            if ldist < min_dist:
                min_dist = ldist
                direction = self.CCW
            if rdist < min_dist:
                min_dist = rdist
                direction = self.CW
        
        ##we want to turn away from closest corner so invert dir
        #turn_dir = self.CCW if direction == self.CW else self.CW
        rospy.logdebug('Turn dir: {}'.format('CCW' if direction == self.CCW else 'CW'))

        return MotorCmd(direction, CIRCUMVENT_SPEED)

    def collect(self):
        #initialise
        if self.collect_idx is None:
            self.collect_idx = 0
        
        #check if done
        if self.collect_idx >= COLLECT_LOOPS:
            self.collect_idx = None
            self.state = self.SEARCH
            rospy.loginfo('Collection done. Moving to search')
            return MotorCmd(self.STOP, 0.0)
        
        self.collect_idx += 1
        return MotorCmd(self.FORWARD, COLLECT_SPEED)

    def collision_imminent(self):
        '''
        Detect of obstacle collision is imminent
        '''
        #object base is between (lx, ly) and (rx, ry)
        if self.obstacles is None:
            return False

        for lx, ly, rx, ry in zip(self.obstacles.lx, self.obstacles.ly, 
                self.obstacles.rx, self.obstacles.ry):
            #calculate distance of extremities. If any is too close, don't need to check any further
            #rospy.logdebug('Left dist: {}, Right dist: {}'.format(self.distance(lx, ly),
            #   self.distance(rx, ry)))
            if self.distance(lx, ly) <= MIN_OBSTACLE_DISTANCE or self.distance(rx, ry) <= MIN_OBSTACLE_DISTANCE:
                return True
        return False
    
    @staticmethod
    def distance(x, y):
        '''
        Helper static method to calculate distance to x, y coordinate
        '''
        return math.sqrt(x**2 + y**2)

    def obstacle_callback(self, obstacles):
        '''
        Update obstacles
        NB obstacles store list of x, y coordinates
        of bottom left and right corner of detected obstacles
        '''
        self.obstacles = obstacles

    def target_callback(self, targets):
        '''
        Update targets
        NB targets are list of x, y coordinates representing centre of detected
        targets
        '''
        self.targets = targets

    def step(self):
        '''
        Given current state and available data, execute one 'step' in appropriate
        direction
        '''
        #defer to state callbacks to handle determining next step
        return self.STEP_TABLE[self.state]()

def path_planner_node():
    pub = rospy.Publisher('/actuators/motor_cmds', MotorCmd, queue_size=2)
    rospy.init_node('path_planner_node', anonymous=False, log_level=rospy.INFO)

    planner = PathPlanner() 

    obstacle_sub = rospy.Subscriber('/planner/obstacles', Obstacles, planner.obstacle_callback)
    target_sub = rospy.Subscriber('/planner/targets', Targets, planner.target_callback)

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


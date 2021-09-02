#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher that publishes MotorCmds to the motor_controller
node in order to travel to a set of waypoints.

Publishes to:
    /actuators/motor_cmds with message MotorCmd

Subscribes to:
    /sensors/odometry with message OdometryData

Services:
    'waypoint_clear': Empty srv type. Calling clears the waypoint list
    'waypoint_add': Waypoints srv type. All specified waypoints will be added to end of waypoint list

Written by Josh Cherubino
Last edited 30/08/21 by Josh Cherubino
'''

import rospy
import gpiozero
from ece3091.msg import MotorCmd, OdometryData
from ece3091.srv import Waypoints, WaypointsResponse, Empty, EmptyResponse
import collections #import deque object for efficient waypoint manipulation

RATE = 50
Kp = 0.5 #proportionality coefficient for proportional speed control when approaching waypoints
TOLERANCE = 2 #percentage tolerance on closeness to waypoint values
SPEED = 0.25

class WaypointController(object):
    '''
    Class that manages a set of waypoints and the current odometry
    data to determine what motor commands to output
    '''
    def __init__(self):
        #list of cmds for each waypoint (corresponding to direction - see motor_control.py)
        self.cmds = collections.deque()
        #list of endpoints relating to odometry that specify when waypoint reached
        self.endpoints = collections.deque()
        #list of strings ('distance' or 'orientation') specifynig what the corersponding
        #endpoint is measured in terms of.
        self.units = collections.deque()

        self.cur_cmd = None
        self.cur_endpoint = None
        self.cur_unit = None

        #vars to track current odometry
        self.distance = 0
        self.orientation = 0

    def clear(self, *args):
        '''
        Method to clear all current waypoints.
        Takes any args so can be called as a ros-service callback
        or called directly
        '''
        self.cmds.clear()
        self.endpoints.clear()
        self.units.clear()
        self.cur_cmd = None
        self.cur_endpoint = None
        self.cur_unit = None
        return EmptyResponse()

    def add_waypoints(self, waypoints):
        '''
        Method to add a set of waypoints to the current waypoints
        '''
        #must convert to list
        self.cmds.extend(list(bytearray(waypoints.cmds)))
        self.endpoints.extend(waypoints.endpoints)
        self.units.extend(waypoints.units)
        #if currently have no active waypoint, try and advance
        if self.cur_cmd == None:
            self.advance()

        rospy.loginfo('Cmds: {}'.format(self.cmds))
        rospy.loginfo('Endpoints: {}'.format(self.endpoints))
        rospy.loginfo('Units: {}'.format(self.units))
        #return response
        return WaypointsResponse()

    def odom_callback(self, odom_data):
        '''
        Callback to update odom values when called
        '''
        self.distance = odom_data.distance
        self.orientation = odom_data.orientation

    def reset_odometry(self):
        '''
        Method to call the 'reset_odometry' service to clear
        odometry values back to 0. Called after a waypoint reached
        '''
        rospy.wait_for_service('reset_odometry')
        try:
            service = rospy.ServiceProxy('reset_odometry', Empty)
            resp = service() #call service
            rospy.loginfo('Odometry reset')
        except rospy.ServiceException as e:
            #if resetting odometry fails, the waypoint controller
            #will not work so log fatal error.
            rospy.logfatal("Reset odometry failed: {}".format(e))

    def advance(self):
        '''
        Move to next waypoint if possible.
        If not, set cur_cmd, cur_endpoint and cur_unit to 0
        '''
        try:
            #pop from front of queue to process waypoints
            #in the order they were given
            self.cur_cmd = self.cmds.popleft()
            self.cur_endpoint = self.endpoints.popleft()
            self.cur_unit = self.units.popleft()
            rospy.loginfo('Advanced to next waypoint. Cmd: {} Endpoint: {:.2f} Unit: {}'.format(self.cur_cmd,
                self.cur_endpoint, self.cur_unit))
        #no more waypoints
        except IndexError:
            self.cur_cmd = None
            self.cur_endpoint = None
            self.cur_unit = None
            rospy.loginfo('Reached all waypoints')

    def arrived(self):
        '''
        Check if we have arrived at the next waypoint
        '''
        #get current relevant odometry value
        current_value = getattr(self, self.cur_unit) 
        #see if we are close enough to value (within our tolerance)
        if abs(self.cur_endpoint - current_value) <= \
                abs(self.cur_endpoint*TOLERANCE/100):
            #if so we have arrived
            rospy.loginfo('Arrived at current waypoint')
            return True
        return False

    def step(self):
        '''
        Method to advance the robot towards the next waypoint.
        Returns a MotorCmd to drive the motor with the appropriate value
        '''
        if self.cur_cmd == None:
            return MotorCmd(0, 0) #move nowhere
        
        #check if waypoint reached
        if self.arrived():
            self.reset_odometry()
            self.advance()
            return MotorCmd(0, 0) #advance to next waypoint to be handled next loop

        #otherwise, move towards waypoint using proportional speed control
        #calculate how far away we are as proportion of target
        error = abs(self.cur_endpoint - getattr(self, self.cur_unit))/abs(self.cur_endpoint)
        #our speed is then our Kp * error.
        speed = max(0.0, min(Kp*error, 1.0)) # clamp to between 0 and 1 range for motor_control node
        return MotorCmd(self.cur_cmd, speed)

def waypoint_node():
    pub = rospy.Publisher('/actuators/motor_cmds', MotorCmd, queue_size=10)
    #can only ever have waypoint node
    rospy.init_node('waypoint_node', anonymous=False)

    #create waypoint controller instance
    waypoint_controller = WaypointController()
    
    sub = rospy.Subscriber('/sensors/odometry', OdometryData, waypoint_controller.odom_callback)

    clear_service = rospy.Service('waypoint_clear', Empty, waypoint_controller.clear)
    add_service = rospy.Service('waypoint_add', Waypoints, waypoint_controller.add_waypoints)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting waypoint node')
    
    #loop using waypoint controller to determine most appropriate motor command.
    while not rospy.is_shutdown():
        msg = waypoint_controller.step()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        waypoint_node()
    except rospy.ROSInterruptException:
        pass


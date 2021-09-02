#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher that publishes the feature map of targets and 
obstacles it has built using distance sensor data and odometry data. Will eventually
integrate camera data as well

Publishes to:
    /planner/feature_map

Subscribes to:
    /sensors/distances
    /sensors/odometry_xy

Written by Josh Cherubino
Last edited 30/08/21 by Josh Cherubino
'''

import rospy
import math
from ece3091.msg import MotorCmd, FeatureMap, OdometryDataXY

RATE = 50

class PathPlanner(object):
    '''
    Class to plan a path given a feature map and current x y coordintes and orientation.
    '''
    def __init__(self):
        self.x = 0
        self.y = 0
        self.orientation = 0
        self.target_x = []
        self.target_y = []
        self.obstacle_x = []
        self.obstalce_y = []

        self.ALIGN = 0
        self.APPROACH = 1
        self.CIRCUMVENT = 2
        self.ARRIVED
        self.state = self.ALIGN

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
        '''
        self.target_x = fmap.target_x
        self.target_y = fmap.target_y
        self.obstacle_x = fmap.obstacle_x
        self.obstacle_y = fmap.obstacle_y

    def step(self):
        '''
        Given the current feature map and position of the robot, take a step in
        the best direction
        '''
        #attempt to align with target in state ALIGN
        #Once aligned (within tolerance) move state to APPROACH

        #if this will cause a collision, set state to CIRCUMVENT and 
        #move away from obstacle (in direction that will take robot closer to target given robot orientation)
        #Once certain distance travelled, move state back to ALIGN

        #In approach state, 
        #approach target in straight line. If we are no longer pointing towards target
        #move back to ALIGN state

        #if target reached, then move to state ARRIVED

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
        feature_map_node()
    except rospy.ROSInterruptException:
        pass


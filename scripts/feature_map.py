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
from ece3091.msg import DistanceData, FeatureMap, OdometryDataXY

RATE = 50

class FeatureMapper(object):
    '''
    Class to build a feature map from surrounding data
    '''
    def __init__(self, target_x=None, target_y=None):
        #feature map internally stored as an array of sparse coordinates
        #the robot is always defined at the centre of the grid and coordinates
        #are relative to this.
        #y coordinates are defined 'in front' of the robot and x coordinates are positive
        #to the right of the robot (from behind)
        if target_x:
            self.target_x = target_x
            self.target_y = target_y
        else:
            self.target_x = []
            self.target_y = []
        self.obstacle_x = []
        self.obstacle_y = []
        self.cur_x = 0
        self.cur_y = 0
        self.cur_orientation = 0
    
    def transform_front(self, value):
        '''
        Transform value from robot local coordinate space into coordinate
        space in axes from starting position
        Return x, y tuple
        '''
        theta = math.radians(self.cur_orientation)
        return (value*math.cos(theta), value*math.sin(theta))

    def transform_left(self, value):
        '''
        Transform value from left of robot (at 90 degree angle to front)
        into relative coordinate space (from starting position)
        Returns x,y tuple
        '''
        phi = math.radians(90 - self.cur_orientation)
        return (-value*math.cos(phi),value*math.sin(phi))

    def transform_right(self, value):
        '''
        Transform value from right of robot (at 90 degree angle to front)
        into relative coordinate space (from starting position)
        Returns x,y tuple
        '''
        phi = math.radians(90 - self.cur_orientation)
        return (value*math.cos(phi), -value*math.sin(phi))

    def distance_callback(self, distance_data):
        '''
        Callback from distance sensors
        '''
        #update obstacles map
        #assume each detected distance value is a single point in feature map
        #must also use robot orientation to transform points into starting coordinate system
        #front

        #distance data must be non zero or obstacles will be added on rover
        delta_front = self.transform_front(distance_data.front) 
        self.obstacle_x.append(self.cur_x + delta_front[0])
        self.obstacle_y.append(self.cur_y + delta_front[1])
        #left
        #temporarily comment out while no left and right sensors.
        #delta_left = self.transform_left(distance_data.left)
        #self.obstacle_x.append(self.cur_x + delta_left[0])
        #self.obstacle_y.append(self.cur_y + delta_left[1])
        #right
        #delta_right = self.transform_right(distance_data.right)
        #self.obstacle_x.append(self.cur_x + delta_right[0])
        #self.obstacle_y.append(self.cur_y + delta_right[1])

    def odom_callback(self, odometry_data):
        '''
        Callback from odometry updates
        '''
        self.cur_x = odometry_data.x
        self.cur_y = odometry_data.y
        self.cur_orientation = odometry_data.orientation

    def pop(self):
        '''
        Method to give the current feature map and clear its contents
        Return in order (targets, obstacles)
        '''
        msg = FeatureMap(self.target_x, self.target_y, self.obstacle_x, self.obstacle_y)
        #N.B. for now keep targets array intact
        #can now safely reset obstacles
        self.obstacle_x = []
        self.obstacle_y = []
        return msg

def feature_map_node():
    pub = rospy.Publisher('/planner/feature_map', FeatureMap, queue_size=2)
    rospy.init_node('feature_map_node', anonymous=False)

    #create mapper with hardcoded target
    mapper = FeatureMapper(target_x=[30], target_y=[30])

    distance_sub = rospy.Subscriber('/sensors/distances', DistanceData, mapper.distance_callback)
    odom_sub = rospy.Subscriber('/sensors/odometry_xy', OdometryDataXY, mapper.odom_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting feature map node')
    
    while not rospy.is_shutdown():
        msg = mapper.pop() #get latest feature map and reset for next iteration
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        feature_map_node()
    except rospy.ROSInterruptException:
        pass


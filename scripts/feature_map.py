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
import gpiozero
from ece3091.msg import DistanceData, FeatureMap, OdometryDataXY

RATE = 10

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
    
    def distance_callback(self, distance_data):
        '''
        Callback from distance sensors
        '''
        #update obstacles map
        #assume each detected distance value is a single point in feature map
        #front
        self.obstacle_x.append(self.cur_x)
        self.obstacle_y.append(self.cur_y + distance_data.front)
        #left
        self.obstacle_x.append(self.cur_x - distance_data.left)
        self.obstacle_y.append(self.cur_y)
        #right
        self.obstacle_x.append(self.cur_x + distance_data.left)
        self.obstacle_y.append(self.cur_y)

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
        msg = FeatureMap(self.target_x, self.target_y, self.obstacle_x, self.obstacle_y,
                self.cur_orientation)
        #N.B. for now keep targets array intact
        #can now safely reset obstacles
        self.obstacles = []
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
    
    #loop using waypoint controller to determine most appropriate motor command.
    while not rospy.is_shutdown():
        msg = mapper.pop() #get latest feature map and reset for next iteration
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        feature_map_node()
    except rospy.ROSInterruptException:
        pass


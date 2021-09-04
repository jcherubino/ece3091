#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher that publishes the feature map of targets and 
obstacles it has built using distance sensor data and odometry data. Will eventually
integrate camera data as well

Without camera data, this node doesn't need to do a whole lot.

Publishes to:
    /planner/feature_map

Subscribes to:
    /sensors/distances

Written by Josh Cherubino
Last edited 30/08/21 by Josh Cherubino
'''

import rospy
import math
from ece3091.msg import DistanceData, FeatureMap

RATE = 50

class FeatureMapper(object):
    '''
    Class to build a feature map from surrounding data. Builds feature map
    in local robot coordintes (i.e. x y system centered on front of robot
    with rotation of robot)
    '''
    def __init__(self, target_x, target_y):
        #feature map internally stored as an array of sparse coordinates
        #y coordinates are defined 'in front' of the robot and x coordinates are positive
        #to the right of the robot (from behind)
        #HOWEVER, target is in the x,y coordinate plane relative to the starting position.
        self.target_x = target_x
        self.target_y = target_y
        self.obstacle_x = []
        self.obstacle_y = []
    
    def distance_callback(self, distance_data):
        '''
        Callback from distance sensors
        '''
        #update obstacles map
        #assume each detected distance value is a single point in feature map
        #front
        #distance data must be non zero or obstacles will be added on rover
        self.obstacle_x.append(0)
        self.obstacle_y.append(distance_data.front)
        #left
        #temporarily comment out while no left and right sensors.
        #self.obstacle_x.append(-distance_data.left)
        #self.obstacle_y.append(0)
        #right
        #self.obstacle_x.append(distance_data.right)
        #self.obstacle_y.append(0)

    def pop(self):
        '''
        Method to give the current feature map and clear its contents
        Return in order (targets, obstacles)
        '''
        msg = FeatureMap(self.target_x, self.target_y, self.obstacle_x, self.obstacle_y)
        #N.B. for now keep targets intact
        #can now safely reset obstacles
        self.obstacle_x = []
        self.obstacle_y = []
        return msg

def feature_map_node():
    pub = rospy.Publisher('/planner/feature_map', FeatureMap, queue_size=2)
    rospy.init_node('feature_map_node', anonymous=False)

    #create mapper with hardcoded target
    mapper = FeatureMapper(target_x=30, target_y=30)

    distance_sub = rospy.Subscriber('/sensors/distances', DistanceData, mapper.distance_callback)

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


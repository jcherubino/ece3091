#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher that publishes the detected obstacles and targets
from the computer vision analysis of a single frame. The obstacles and targets
are published in x, y coordinates relative to the robots current position.

Publishes to:
    /planner/obstacles
    /planner/targets

Subscribes to:
    /sensors/picam

Written by Josh Cherubino
Last edited 20/09/21 by Josh Cherubino
'''

import rospy
import math
from ece3091.msg import CamData, Points
import numpy as np
import cv2 as cv

obstacles_x = []
obstacles_y = []
targets_x = []
targets_y = []

MIN_OBSTACLE_CONTOUR_AREA = 10000 #minimum pixel area that contour must exceed to be considered obstacle
OBSTACLE_BLUR_KERNEL_SZ = 31 #size of medianblur kernel for obstacle detection
RATE = 20
HEIGHT = None
WIDTH = None

def frame_callback(cam_data):
    global obstacles_x, obstacles_y, targets_x, targets_y
    
    #convert raw data into cv object (which is actually just numpy array)
    #data is already in BGR order.
    img = np.frombuffer(cam_data.data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
    
    #update obstacles and targets
    obstacles_x, obstacles_y = detect_obstacles(img)
    targets_x, targets_y = detect_targets(img)

def detect_obstacles(img):
    #detects obstacles and returns array of points. Note that points are paired where
    #the ith and i + 1 points represent the closest left and right corners of an obstacle
    blurred = cv.medianBlur(img, OBSTACLE_BLUR_KERNEL_SZ) #smooth image to only 'see' big objects 
    #apply OTSU thresholding to find large foreground objects
    _, thresholded = cv.threshold(blurred, 0, 255, cv.THRESH_BINARY|cv.THRESH_OTSU)
    #find contours. N.B. function modifies input thresholded image
    contours, _ = cv.findContours(thresholded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    x_corner = [], y_corner = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area > MIN_OBSTACLE_CONTOUR_AREA:
            #perceive object as set of points along 'bottom' of bounding rectangle
            #of the contour
            x, y, w, h = cv.boundingRect(contour)
            x_corner.extend([x, x+w])
            y_corner.extend([y+h, y+h])
    
    return pixel_to_relative_coords(x_corner, y_corner)

def detect_targets(img, x, y):
    pass

def pixel_to_relative_coords(x, y):
    #convert list of x,y points in pixel-space to coordinates in 
    #relative position to robot
    #TODO: Do transform
    return x, y

def frame_process_node():
    obstacle_pub = rospy.Publisher('/planner/obstacles', Points, queue_size=2)
    target_pub = rospy.Publisher('/planner/targets', Points, queue_size=2)
    rospy.init_node('frame_process_node', anonymous=False)

    HEIGHT = rospy.get_param('picam/height')
    WIDTH = rospy.get_param('picam/width')

    cam_sub = rospy.Subscriber('/sensors/picam', CamData, frame_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting frame process node')
    
    obstacle_msg = Points()
    target_msg = Points()

    while not rospy.is_shutdown():
        obstacle_msg.x = obstacles_x
        obstacle_msg.y = obstacles_y
        obstacle_pub.publish(obstacle_msg)
        target_msg.x = target_x
        target_msg.y = target_y
        target_pub.publish(target_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        frame_process_node()
    except rospy.ROSInterruptException:
        pass


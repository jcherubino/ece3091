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
from ece3091.msg import CamData, Obstacles, Targets
import numpy as np
import cv2 as cv

obstacles = Obstacles()
targets = Targets()
output_features = False

MIN_OBSTACLE_CONTOUR_AREA = 10000 #minimum pixel area that contour must exceed to be considered obstacle
OBSTACLE_BLUR_KERNEL_SZ = 31 #size of medianblur kernel for obstacle detection
RATE = 20
HEIGHT = None
WIDTH = None

def frame_callback(cam_data):
    global obstacles, targets
    
    #convert raw data into cv object (which is actually just numpy array)
    #data is already in BGR order.
    img = np.frombuffer(cam_data.data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
    
    if output_features:
        feature_img = img.copy()

    #update obstacles and targets
    obstacles.lx, obstacles.ly, obstacles.rx, obstacles.ry = detect_obstacles(img)
    targets.x, targets.y = detect_targets(img)

def detect_obstacles(img):
    global output_features
    blurred = cv.medianBlur(img, OBSTACLE_BLUR_KERNEL_SZ) #smooth image to only 'see' big objects 
    #apply OTSU thresholding to find large foreground objects
    _, thresholded = cv.threshold(blurred, 0, 255, cv.THRESH_BINARY|cv.THRESH_OTSU)
    #find contours. N.B. function modifies input thresholded image
    contours, _ = cv.findContours(thresholded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    lx = []
    ly = []
    rx = []
    ry = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area > MIN_OBSTACLE_CONTOUR_AREA:
            #perceive object as set of points along 'bottom' of bounding rectangle
            #of the contour. N.B. Only need to store corners.
            rect = cv.minAreaRect(contour)
            box = cv.boxPoints(rect)
            #store points
            lx.append(box[0][0]),ly.append(box[0][1])
            rx.append(box[3][0]),ry.append(box[3][1])

            #draw obstacle bounding box in red
            if output_features:
                global feature_img
                cv.drawContours(feature_img,[box],0,(0,0,255),2)  

    lx, ly = pixel_to_relative_coords(lx, ly)
    rx, ry = pixel_to_relative_coords(rx, ry)
    return lx, ly, rx, ry

def detect_targets(img):
    global output_features
    #convert to greyscale for hough circles
    greyscale = cvt.cvtColor(img, cv.COLOR_BGR2GRAY)
    circles = cv.HoughCircles(greyscale, cv.HOUGH_GRADIENT, 1, 25, param1=65,
            param2=37.5, minRadius=10, maxRadius=150)
    #extract circle centers
    x, y = circles[0, :, 0].tolist(), circles[0, :, 1].tolist()

    #Draw outer circle in green
    if output_features:
        global feature_img
        for i in circles[0, :]:
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    return pixel_to_relative_coords(x, y)

def pixel_to_relative_coords(x, y):
    #convert list of x,y points in pixel-space to coordinates in 
    #relative position to robot
    #TODO: Do transform
    return x, y

def frame_process_node():
    global obstacles, targets
    obstacle_pub = rospy.Publisher('/planner/obstacles', Obstacles, queue_size=2)
    target_pub = rospy.Publisher('/planner/targets', Targets, queue_size=2)

    #turn on feature viewing
    output_features = rospy.get_param('/planner/show_feature_view', False)
    if output_features == True:
        feature_pub = rospy.Publisher('/planner/feature_view', CamData, queue_size=2)

    rospy.init_node('frame_process_node', anonymous=False)

    HEIGHT = rospy.get_param('picam/height')
    WIDTH = rospy.get_param('picam/width')

    cam_sub = rospy.Subscriber('/sensors/picam', CamData, frame_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting frame process node')
    
    while not rospy.is_shutdown():
        obstacle_pub.publish(obstacles)
        target_pub.publish(targets)
        if output_features:
            feature_pub.publish(feature_img.flatten())
        rate.sleep()

if __name__ == '__main__':
    try:
        frame_process_node()
    except rospy.ROSInterruptException:
        pass


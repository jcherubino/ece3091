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
import os
import rospy
import math
from ece3091.msg import CamData, Obstacles, Targets
import numpy as np
import cv2 as cv

obstacles = Obstacles()
targets = Targets()
MIN_OBSTACLE_CONTOUR_AREA = 10000 #minimum pixel area that contour must exceed to be considered obstacle
OBSTACLE_BLUR_KERNEL_SZ = 31 #size of medianblur kernel for obstacle detection
RATE = rospy.get_param('picam/framerate')
HEIGHT = rospy.get_param('picam/height')
WIDTH = rospy.get_param('picam/width')
output_features = True
feature_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
CAM_HEIGHT = 16.9 #height of camera above ground
CAM_ANGLE_X = 62.2 #degrees
CAM_ANGLE_Y = 48.8 #degrees

def frame_callback(cam_data):
    global obstacles, targets, feature_img
    
    #convert raw data into cv object (which is actually just numpy array)
    #data is already in BGR order.
    img = np.frombuffer(cam_data.data, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
    
    if output_features:
        feature_img = img.copy()

    #update obstacles and targets
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    obstacles.lx, obstacles.ly, obstacles.rx, obstacles.ry = detect_obstacles(img)
    targets.x, targets.y = detect_targets(img)

def detect_obstacles(img):
    global output_features
    blurred = cv.medianBlur(img, OBSTACLE_BLUR_KERNEL_SZ) #smooth image to only 'see' big objects 
    #apply OTSU thresholding to find large foreground objects
    _, thresholded = cv.threshold(blurred, 0, 255, cv.THRESH_BINARY|cv.THRESH_OTSU)
    thresholded = cv.bitwise_not(thresholded)
    #find contours. N.B. function modifies input thresholded image
    _,contours, _ = cv.findContours(thresholded, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

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
            lx.append(box[1][0]),ly.append(box[1][1])
            rx.append(box[0][0]),ry.append(box[0][1])
            
            #draw obstacle bounding box in red
            if output_features:
                global feature_img
                box = np.int0(box)
                cv.drawContours(feature_img,[box],0,(0,0,255),2)  

    lx, ly = pixel_to_relative_coords(lx, ly)
    rx, ry = pixel_to_relative_coords(rx, ry)
    return lx, ly, rx, ry

def detect_targets(img):
    global output_features
    #convert to greyscale for hough circles
    circles = cv.HoughCircles(img, cv.HOUGH_GRADIENT, 1, 25, param1=65,
            param2=37.5, minRadius=10, maxRadius=150)
    #extract circle centers
    if circles is not None:
        x, y = circles[0, :, 0].tolist(), circles[0, :, 1].tolist()
    else:
        x, y, = [], []

    #Draw outer circle in green
    if output_features and circles is not None:
        global feature_img
        for i in circles[0, :]:
            cv.circle(feature_img,(i[0],i[1]),i[2],(0,255,0),2)
    #return pixel_to_relative_coords(x, y)
    return x,y

def pixel_to_relative_coords(x, y):
    #convert list of x,y points in pixel-space to coordinates in 
    #relative position to robot
    #https://docs.google.com/document/d/1sCGtqg6PN32aOoAip6fPdeA9rgEFPXTpvX24FOmiZdM/edit
    #shift centre of frame to be (0,0) and invert y to be postiive is up
    x_t = np.array(x) - WIDTH/2
    y_t = -(np.array(x) - HEIGHT/2) 
    y_dist = CAM_HEIGHT*np.tan(np.radians(60) + np.radians(y_t/HEIGHT*CAM_ANGLE_Y))
    x_dist = y_dist*np.tan(np.radians(x_t/WIDTH*CAM_ANGLE_X))
    return x_dist.tolist(), y_dist.tolist()

def frame_process_node():
    global obstacles, targets
    obstacle_pub = rospy.Publisher('/planner/obstacles', Obstacles, queue_size=2)
    target_pub = rospy.Publisher('/planner/targets', Targets, queue_size=2)

    rospy.init_node('frame_process_node', anonymous=False)

    cam_sub = rospy.Subscriber('/sensors/picam', CamData, frame_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting frame process node')
    
    frame_count = 0

    while not rospy.is_shutdown():
        obstacle_pub.publish(obstacles)
        target_pub.publish(targets)
        if output_features:
            if frame_count > 0 and frame_count % 50 == 0:
                #save frame for debugging
               cv.imwrite('/home/pi/processed_frames/{}.bmp'.format(frame_count), feature_img)
            frame_count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        frame_process_node()
    except rospy.ROSInterruptException:
        pass


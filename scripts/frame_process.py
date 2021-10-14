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
            row_indices = np.argsort(box[:,1])[2:]
            col_indices = np.argsort(box[row_indices,0])

            lx.append(box[row_indices][col_indices[0]][0])
            ly.append(box[row_indices][col_indices[0]][1])
            rx.append(box[row_indices][col_indices[1]][0])
            ry.append(box[row_indices][col_indices[1]][1])
            
            #draw obstacle bounding box in red
            if output_features:
                global feature_img
                box = np.int0(box)
                cv.drawContours(feature_img,[box],0,(0,0,255),2)  

                cv.circle(feature_img,(lx[-1], ly[-1]), 10,(255,0,0),2)
                cv.circle(feature_img,(rx[-1], ry[-1]), 10,(255,0,0),2)

    rospy.logdebug('Obstacles: lx: {} ly: {} rx: {} ry: {}'.format(lx, ly, rx, ry))
    lx, ly = pixel_to_relative_coords(lx, ly)
    rx, ry = pixel_to_relative_coords(rx, ry)
    return lx, ly, rx, ry

def detect_targets(initialimage):
    # Initial HSV GUI slider values to load on program start.
    icol = (147, 0, 0, 154, 255, 255)    # preset value
    lowHue = icol[0]
    lowSat = icol[1]
    lowVal = icol[2]
    highHue = icol[3]
    highSat = icol[4]
    highVal = icol[5]
    frameBGR = cv2.GaussianBlur(initialimage, (5, 5), 0)
    # HSV (Hue, Saturation, Value).
    # Convert the frame to HSV colour model.
    hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)   
    # HSV values to define a colour range.
    colorLow = np.array([lowHue,lowSat,lowVal])
    colorHigh = np.array([highHue,highSat,highVal])
    mask = cv2.inRange(hsv, colorLow, colorHigh)
    # Show the first mask
    #cv2.imshow('mask-plain', mask)
     
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    
    # Show morphological transformation mask
    #cv2.imshow('mask', mask)
        
    # Put mask over top of the original image.
    result = cv2.bitwise_and(initialimage, initialimage, mask = mask)
     
    # Show final output image
    #cv2.imshow('colorTest', result)
    cv2.imwrite('test1.png',result)
    
    
    # Windows file path
    # Change here for camera or video input
    imageinput = result
    frame = cv2.imread('test1.png')
    # grayscale image
    gray=cv2.cvtColor(imageinput,cv2.COLOR_BGR2GRAY)
    
    # medianblur if needeed
    #img = cv2.medianBlur(gray,5)
    img = gray
    location=[0,0]
    # first time circle detection
    circles= cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=28,minRadius=0,maxRadius=100)
    
    if circles is not None:
        #print(len(circles[0]))
        for circle in circles[0]:
            #print(circle[2])
            x=int(circle[0])
            y=int(circle[1])
            r=int(circle[2])
            img=cv2.circle(imageinput,(x,y),r,(0,0,255),-1)
            
        #cv2.imshow('res',img)
        # create mask for image cut
        hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(hsv2, (0, 255, 255), (0, 255, 255))
        # cut the detected parts
        result2 = cv2.bitwise_and(frame, frame, mask = mask2)
    
        #cv2.imshow('result', result2)
        #cv2.imshow('mask2', mask2)
            
        # second time circle detection
        gray2 = cv2.cvtColor(result2,cv2.COLOR_BGR2GRAY)
        img2 = cv2.medianBlur(gray2,5)
        
        circles2 = cv2.HoughCircles(img2,cv2.HOUGH_GRADIENT,1.2,100,param1=200,param2=35,minRadius=0,maxRadius=150)
        if circles2 is not None and len(circles2[0]) == 1:
            location = circles2[0][0][0:2]
            #print(location)
            for circle2 in circles2[0]:
                    #print(circle2[2])
                    x=int(circle2[0])
                    y=int(circle2[1])
                    r=int(circle2[2])
                    img2=cv2.circle(result2,(x,y),r,(0,0,255),-1)               
                    #cv2.imshow('res2',img2)
        elif len(circles2[0]) != 1:   
            print ("more than one circle are detected") 
            
        else: print ("no circle detected")
    else: 
        print ("no circle detected")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print(location[0])
    print(location[1])
    return locationpixel_to_relative_coords(location[0], location[1])

def pixel_to_relative_coords(x, y):
    #convert list of x,y points in pixel-space to coordinates in 
    #relative position to robot
    #https://docs.google.com/document/d/1sCGtqg6PN32aOoAip6fPdeA9rgEFPXTpvX24FOmiZdM/edit
    #shift centre of frame to be (0,0) and invert y to be postiive is up
    x_t = np.array(x) - WIDTH/2
    y_t = -(np.array(y) - HEIGHT/2) 
    y_dist = CAM_HEIGHT*np.tan(np.radians(60) + np.radians(y_t/HEIGHT*CAM_ANGLE_Y))
    x_dist = y_dist*np.tan(np.radians(x_t/WIDTH*CAM_ANGLE_X))
    return x_dist.tolist(), y_dist.tolist()

def frame_process_node():
    global obstacles, targets
    obstacle_pub = rospy.Publisher('/planner/obstacles', Obstacles, queue_size=2)
    target_pub = rospy.Publisher('/planner/targets', Targets, queue_size=2)

    rospy.init_node('frame_process_node', anonymous=False, log_level=rospy.INFO)

    cam_sub = rospy.Subscriber('/sensors/picam', CamData, frame_callback)

    rate = rospy.Rate(RATE)
    rospy.loginfo('Starting frame process node')
    
    frame_count = 0

    while not rospy.is_shutdown():
        obstacle_pub.publish(obstacles)
        target_pub.publish(targets)
        if output_features:
            if frame_count > 0 and frame_count % 100 == 0:
                #save frame for debugging
               cv.imwrite('/home/pi/processed_frames/{}.bmp'.format(frame_count), feature_img)
            frame_count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        frame_process_node()
    except rospy.ROSInterruptException:
        pass


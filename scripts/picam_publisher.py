#!/usr/bin/env python

'''
ECE3091 Group 5

This node is a publisher for the raw pi camera data.

Publishes to:
    /sensors/picam with message format CamData

Written by Josh Cherubino
Last edited 16/08/21 by Josh Cherubino
'''
import rospy
import picamera
import cv2 as cv
import numpy as np
import math

#msg type
from ece3091.msg import Obstacles, Targets

#Program variables
#picam resolution. N.B. horizontal resolution must be multiple of 32 and vertical multiple of 16
HEIGHT = 480
WIDTH = 640
RATE = 10 #Hz rate node spins at 
rospy.set_param('/rate', RATE)

MIN_OBSTACLE_CONTOUR_AREA = 10000 #minimum pixel area that contour must exceed to be considered obstacle
OBSTACLE_BLUR_KERNEL_SZ = 31 #size of medianblur kernel for obstacle detection
SAVE_OUTPUT = False
feature_img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
CAM_HEIGHT = 16.9 #height of camera above ground
CAM_ANGLE_X = 62.2 #degrees
CAM_ANGLE_Y = 48.8 #degrees

LOWHUE = 0
LOWSAT = 100
LOWVAL = 100
HIGHHUE = 180
HIGHSAT = 180
HIGHVAL = 165
COLOR_LOW = np.array([LOWHUE,LOWSAT,LOWVAL])
COLOR_HIGH = np.array([HIGHHUE,HIGHSAT,HIGHVAL])
KERNEL = cv.getStructuringElement(cv.MORPH_ELLIPSE, (18, 18))

#set values as rosparams so they can be globally accessed in ROS network
#rospy.set_param('picam', {'width': RESOLUTION[0], 'height': RESOLUTION[1],
#                'framerate': RATE})

camera = picamera.PiCamera(resolution=(WIDTH, HEIGHT), framerate=90)
camera.rotation = 90
camera.start_preview()

def detect_obstacles(img):
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
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
            if SAVE_OUTPUT:
                global feature_img
                box = np.int0(box)
                cv.drawContours(feature_img,[box],0,(0,0,255),2)  

                cv.circle(feature_img,(lx[-1], ly[-1]), 10,(255,0,0),2)
                cv.circle(feature_img,(rx[-1], ry[-1]), 10,(255,0,0),2)

    rospy.logdebug('Obstacles: lx: {} ly: {} rx: {} ry: {}'.format(lx, ly, rx, ry))
    lx, ly = pixel_to_relative_coords(lx, ly)
    rx, ry = pixel_to_relative_coords(rx, ry)
    return lx, ly, rx, ry

def detect_targets(img):
    # Convert the frame to HSV colour model.
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)   
    # HSV values to define a colour range.
    mask = cv.inRange(hsv, COLOR_LOW, COLOR_HIGH)

    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, KERNEL)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, KERNEL)
    # Put mask over top of the original image.
    masked = cv.bitwise_and(img, img, mask = mask)

    #gray scale image
    gray = cv.cvtColor(masked,cv.COLOR_BGR2GRAY)

    #find circles
    circles = cv.HoughCircles(gray,cv.HOUGH_GRADIENT,1,20,
                param1=50,param2=30,minRadius=5,maxRadius=500)

    #extract circle centers
    if circles is not None:
        x, y = circles[0, :, 0].tolist(), circles[0, :, 1].tolist()
    else:
        x, y, = [], []

    #Draw outer circle in green
    if SAVE_OUTPUT and circles is not None:
        global feature_img
        for i in circles[0, :]:
            cv.circle(feature_img,(i[0],i[1]),i[2],(0,255,0),2)
            cv.circle(feature_img, (i[0],i[1]), 2, (0, 0, 255), 2)
    rospy.logdebug('Target centers: x:  {} y: {}'.format(x, y))
    return pixel_to_relative_coords(x, y)

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

def picam_publisher():
    global feature_img
    obstacle_pub = rospy.Publisher('/planner/obstacles', Obstacles, queue_size=2)
    target_pub = rospy.Publisher('/planner/targets', Targets, queue_size=2)
    #only ever want 1 camera node running so make it non anonymous
    rospy.init_node('picam_publisher', anonymous=False)

    rate = rospy.Rate(RATE) 
    rospy.loginfo('Starting Pi Camera publisher node')

    #register what to do on program shutdown
    rospy.on_shutdown(close)
    
    frame_count = 0

    while not rospy.is_shutdown():
        obstacles = Obstacles()
        targets = Targets()
        img = np.empty((HEIGHT*WIDTH*3), dtype=np.uint8)
        #capture to bgr format for opencv compat
        camera.capture(img, format='bgr') 
        #reshape into appropriate form
        img = img.reshape((HEIGHT, WIDTH, 3))
        if SAVE_OUTPUT:
            feature_img = img.copy()
        #update obstacles and targets
        obstacles.lx, obstacles.ly, obstacles.rx, obstacles.ry = detect_obstacles(img)
        if frame_count % 2 == 0:
            targets.x, targets.y = detect_targets(img)

        #publish new data
        obstacle_pub.publish(obstacles)
        target_pub.publish(targets)

        if SAVE_OUTPUT:
            if frame_count > 0 and frame_count % 10 == 0:
                #save frame for debugging
               cv.imwrite('/home/pi/processed_frames/{}.bmp'.format(frame_count), feature_img)
               cv.imwrite('/home/pi/raw_frames/{}.bmp'.format(frame_count), img)
            frame_count += 1
        rate.sleep() #wait to run at specified framerate

def close():
    rospy.loginfo('Stopping Pi Camera publisher')
    camera.close()

if __name__ == '__main__':
    try:
        picam_publisher()
    except rospy.ROSInterruptException:
        pass

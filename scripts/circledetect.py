import  cv2
import numpy as np


def targetdetect(address):
    # Initial HSV GUI slider values to load on program start.
    icol = (147, 0, 0, 154, 255, 255)    # preset value
    initialimage = cv2.imread(address)
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
    cv2.imshow('mask-plain', mask)
     
    kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
    
    # Show morphological transformation mask
    cv2.imshow('mask', mask)
        
    # Put mask over top of the original image.
    result = cv2.bitwise_and(initialimage, initialimage, mask = mask)
     
    # Show final output image
    cv2.imshow('colorTest', result)
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
            
        cv2.imshow('res',img)
        # create mask for image cut
        hsv2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask2 = cv2.inRange(hsv2, (0, 255, 255), (0, 255, 255))
        # cut the detected parts
        result2 = cv2.bitwise_and(frame, frame, mask = mask2)
    
        cv2.imshow('result', result2)
        cv2.imshow('mask2', mask2)
            
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
                    cv2.imshow('res2',img2)
        elif len(circles2[0]) != 1:   print ("nore than one circle are detected")
        else: print ("no circle detected")
    else: 
        print ("no circle detected")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return location



filename = 'C:/Users/Aufb/3091/image3.jpg'
target = targetdetect(filename)
print (target)
#!/usr/bin/env python
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Image preocessing libray for Red Cat      #
# robot follow a line task                  #
#                                           #
# Author: Adalberto Oliveira                #
# Mastering in robotic - PUC-Rio            #
# Version: 1.0                              #
# Date: 16-11-2018                          #
#                                           #
#############################################

import rospy, cv2, time, tf, sys, math
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Pose2D
from cv_bridge import CvBridge
from std_msgs.msg import Bool



def bgr2ohta(img):

	rows, col, channels = img.shape
	R = img[0:rows, 0:col,2]
	G = img[0:rows, 0:col,1]
	B = img[0:rows, 0:col,0]

	I1_prime = R-G
	I2_prime = R-B
	I3_prime = (2*G - R - B)/2

	ohta = [I1_prime,
			I2_prime,
			I3_prime]

	return ohta

def ohta2mask(ohta):

	mask = 'received' 
	
	blur1 = cv2.GaussianBlur(ohta[0],(21,21),20)
	th,mask1 = cv2.threshold(blur1,
	                        0,255,
	                        cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	blur2 = cv2.GaussianBlur(ohta[2],(21,21),20)
	th,mask2 = cv2.threshold(blur2,
	                        0,255,
	                        cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	mask2 = ~mask2
	mask = mask1 & mask2
	
	return mask

def findSlope(contour, cv_output):
	
	area_1 = 0
	area_2 = 0
	moment_1 = []
	moment_2 = []
	Cont_1 = []
	Cont_2 = []

	centroid_1 = [0,0]
	centroid_2 = [0,0]   
	
	for c in contour:
		M = cv2.moments(c)
		if (M["m00"] > area_2):
			if (M["m00"] > area_1):
				area_2 = area_1
				moment_2 = moment_1
				moment_1 = M

				area_1 = M["m00"]
				Cont_2 = Cont_1
				Cont_1 = [c]
			else:
				area_2 = M["m00"]
				moment_2 = M
				Cont_2 = [c]
	
	if area_1 > 500:
		centroid_1[0] = int(moment_1["m10"]/moment_1["m00"])
		centroid_1[1] = int(moment_1["m01"]/moment_1["m00"])
		cv2.circle(cv_output, 
					(centroid_1[0], centroid_1[1]), 
					7, (255,0,0),-1)
		cv2.drawContours(cv_output, Cont_1 ,-1,(0,255,0),1)

	if area_2 > 500:
		centroid_2[0] = int(moment_2["m10"]/moment_2["m00"])
		centroid_2[1] = int(moment_2["m01"]/moment_2["m00"])
		cv2.circle(cv_output, 
					(centroid_2[0], centroid_2[1]), 
					7, (0,0,255),-1)
		cv2.drawContours(cv_output, Cont_2 ,-1,(0,255,0),1)
	
	cv2.line(cv_output,
			(centroid_1[0],centroid_1[1]),
			(centroid_2[0],centroid_2[1]),(255,0,0), 2)

	
	if centroid_1[0] > 320:
		l = centroid_1
		r = centroid_2  
		centroid_1 = r
		centroid_2 = l
	
	slope_x = centroid_2[0] - centroid_1[0]
	slope_y = centroid_2[1] - centroid_1[1]
	slope = math.atan2(slope_y,slope_x )

	return slope, cv_output

def hsv2mask(img,low, upper):

	mask = cv2.inRange(img, low, upper)
	blur1 = cv2.GaussianBlur(mask,(21,21),20)

	return mask


def nothing(x):
   pass
	
'''
def process_image():
    global I1_prime
    global I3_prime    
    global cv_image_rgb
    global cmd_vel
    
    fonte = cv2.FONT_HERSHEY_PLAIN
    cmdvel = Twist()
    cv_output = cv_image_rgb
    
    blur = cv2.GaussianBlur(I1_prime,(21,21),20)
    th,mask = cv2.threshold(blur,
                            0,255,
                            cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    
    #mask = ~mask
    
    lines = cv2.Canny(blur, 0,150, apertureSize = 3)
    lines_values = cv2.HoughLines(mask,1,np.pi/180, 380)
    print lines_values[0]
    
    #for l in range(0, len(lines_values)):
    for r,theta in lines_values[0]: 
        a = np.cos(theta) 
        b = np.sin(theta) 
        x0 = a*r 
        y0 = b*r 
        x1 = int(x0 + 1000*(-b)) 
        y1 = int(y0 + 1000*(a)) 
        x2 = int(x0 - 1000*(-b)) 
        y2 = int(y0 - 1000*(a)) 
        cv2.line(cv_output,(x1,y1), (x2,y2), (0,0,255),1)
    
    
    contour = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
    contour = contour[1]
    #print len(contour[1])
    m = 0
    Contour = []
    cx = 0
    cy = 0
    theta = 0
    vel = 0

    
    for c in contour:
        M = cv2.moments(c)
        if M["m00"] > m:
            m = M["m00"]
            Contour = [c]
            cx = int(M["m10"]/M["m00"])
            cy = int(M["m01"]/M["m00"])
        theta = round(-math.atan2((cx - 320),(cy - 240)) * 0.3,2)
        cmdvel.linear.x = vel
        cmdvel.angular.z = 0

    cv2.circle(cv_output, (cx, cy), 7, (0,0,255),-1)
    cv2.drawContours(cv_output, Contour,-1,(0,255,0),1)
    cv2.line(cv_output, (cx,cy), (320,480),(255,0,0), 2)
    #print cx, cy, theta, m, th

    cv2.putText(cv_output, 
                'Theta: '+str(theta) ,(cx-20, cy-20), 
                fonte, 2, (0,0,0),2)
    
    
    #cv2.imshow('I1 prime',lines)
    cv2.imshow('Mask',mask)
    cv2.imshow('RGB output',cv_output)
    cv2.waitKey(1)      
    #print cmdVel
    #cmd_vel.publish(cmdvel)
'''
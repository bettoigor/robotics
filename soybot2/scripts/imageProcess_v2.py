#!/usr/bin/env python
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Controller module class for 				#
# Agribot                   				#
#                                           #
# Author: Adalberto Oliveira                #
# Mastering in robotic - PUC-Rio            #
# Version: 2.1.0610                         #
# Date: 10-6-2020                           #
#                                           #
#############################################

import rospy, time, angles, math, tf2_geometry_msgs, tf, sys, cv2, copy
import numpy as np
from skimage.measure import ransac, LineModelND, CircleModel
from scipy.misc import toimage
from math import sin, cos, tan, atan2
from numpy import pi

#--------------------------------------------------------------------

def get_mask(in_img,kernel,inter):
    '''
    Creates the mask using ExGExR from the input image receiving the
    kernel size and number of iteration to dilate algorithm
    '''
    # image parameters
    rows, col, channels = in_img.shape

    # Image blurring 
    #blur_img = cv2.GaussianBlur(in_img,(5,5),20)

    # Spliting image layers
    (B, G, R) = cv2.split(in_img)
    
    # Normalizing the image layer 
    R_norm = R/float(np.max(R))
    G_norm = G/float(np.max(G))
    B_norm = B/float(np.max(B))

    # Creating the features
    ExG = 2*G_norm - R_norm - B_norm
    ExGExR = ExG - (1.4*R_norm - G_norm)

    # Creating the binary mask
    th = 0.02   # threshold
    mask = copy.deepcopy(ExGExR)
    mask[mask < th] = 0
    mask[mask >= th] = 1
    mask_out = np.ones([rows,col],dtype=np.uint8)
    mask_out = np.multiply(mask_out,mask)*255
    mask_out = np.uint8(mask_out * 255)


    # Improving the mask with binary operations 
    kernel = kernel = np.ones((kernel),np.uint8)
    mask_out = cv2.dilate(mask_out,kernel,iterations=inter)


    return mask_out


#--------------------------------------------------------------------

    
def get_line_1(cv_input,kernel, inter): #, prime_input, prime2):
    '''
    Image process method for feature extration using ExGExR and RANSAC
    for line extration. This function generates a single line using the 
    points from each plant and creates a central line as reference.
    '''

    # Getting image parameters
    cv_output = cv_input.copy()
    bk_h,bk_w,bk_d = cv_output.shape
    black_img = np.zeros([bk_h,bk_w],dtype=np.uint8)
    
    # getting binary mask    
    mask_out = get_mask(cv_input,kernel,inter)


    # Fiding mask contours
    contour = cv2.findContours(mask_out, cv2.RETR_TREE,
                                cv2.CHAIN_APPROX_SIMPLE)

    contour = contour[1]
    
    
    # Getting plants centroides
    centroid_x = []
    centroid_y = []

    for c in contour:
        M = cv2.moments(c)
        # Size Threshold 
        if(M["m00"] > 200): 
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
            lim_r = int(bk_w * 0.8) 
            lim_l = int(bk_w * 0.2)
            if (cX >= lim_l) and (cX <= lim_r):
                centroid_x.append([cX])
                centroid_y.append([cY])
            
                # draw the contour of the plants and its center
                cv2.drawContours(cv_output, [c], -1, (0, 255, 0), 2)
                cv2.circle(cv_output, (cX, cY), 3, (255, 0, 0), -1)
            

    # Preparing RANSAC dataset for fitting
    centroid_x = np.array(centroid_x)
    centroid_y = np.array(centroid_y)
    centroids = np.column_stack([centroid_y, centroid_x])

    # Filtering
    try:
        centroids = np.unique(centroids,axis=0)
        centroids = centroids[centroids[:,1].argsort()]    
    except:
        pass

    # Finding the end of the rows
    row_end = True if (centroids.shape[0] < 1) else False
    

    # For LEFT side
    trial_num = 100
    min_sample_num = 2
    if len(centroids) >= 2:
        # Creating model
        # robustly fit line only using inlier data with RANSAC algorithm
        model_robust, inliers = ransac(centroids, LineModelND, 
                                            min_samples=min_sample_num,
                                            residual_threshold=1, 
                                            max_trials=trial_num)

       
        # Points for prediction
        y_i = 0
        y_f = bk_h-1

        try:
            # Prediction
            x_i = int(model_robust.predict(y_i)[1])
            x_f = int(model_robust.predict(y_f)[1])
            
            # Drawing the line
            cv2.line(cv_output, (x_i,y_i),(x_f,y_f), (0,0,255),2)
            theta = math.atan2((y_f-y_i),(x_f-x_i))
        except:
            y_i = 0
            y_f = bk_h-1
            x_i = 0
            x_f = 0

    else:
        y_i = 0
        y_f = bk_h-1
        x_i = 0
        x_f = 0


    # Defining the medium line
    x_c_i = x_i
    x_c_f = x_f
    y_c_i = 0
    y_c_f = bk_h-1

    # Creating the point P
    P = (int(x_c_f),y_c_f)
    cv2.circle(cv_output, P, 5, (255, 255,0), -1)
    
    # Creating the central point
    X = (x_c_i + x_c_f)*0.5
    Y = y_c_f*0.5
    cv2.circle(cv_output, (int(X), int(Y)), 5, (0, 255, 255), -1)
    
    # Finding theta
    theta = round(math.atan2((x_c_f - x_c_i),(y_c_f - y_c_i)),4)

    # Drawing the reference line
    cv2.line(cv_output, (int(x_c_i),y_c_i),(int(x_c_f),y_c_f), (0,0,0),2)
    cv2.line(black_img, (int(x_c_i),y_c_i),(int(x_c_f),y_c_f), (255,255,255),1)
    
    '''
    If you want to use the Hough tranformation on the black image, uncoment this lines
    '''
    lines = cv2.HoughLines(black_img,1,np.pi/180,10)
    try:
        for rho_h, theta_h in lines[0]:
            rho_h = abs(rho_h) - int(bk_w/2)   
            theta_h  
        H = [rho_h, theta_h]
    except:
        H = [0,0]


    # coordinates of interest points
    U_ = X - int(bk_w/2)        # central point
    P_ = P[0] - int(bk_w/2)     # bottom point (P)
    
    # Drawing the central line
    cv2.line(cv_output, (int(bk_w/2),0),(int(bk_w/2),bk_h), (255,0,0),2)
    mask_show = cv2.cvtColor(mask_out, cv2.COLOR_GRAY2BGR)
    
    return cv_output, mask_out, theta, U_, row_end, H

#--------------------------------------------------------------------


def image_reshape(img, size):
    '''
    Reshape the image, creating a Region of Interest (ROI) with desired size
    in pfraction of origimage.
    '''
    # Desired output size
    h = size[0]
    w = size[1]

    # creating a deep copy of the input image
    img_in = copy.deepcopy(img)

    # image parameters
    img_h,img_w,img_d = img_in.shape
    rs_h = int(img_h*h)#0.7)
    rs_w = int(img_w*w)#0.45)
    rs_d = img_d 

    # reshaped image
    img_reshaped = np.zeros([rs_h,rs_w,rs_d],dtype=np.uint8)

    R = img_in[img_h-rs_h:img_h, int((img_w-rs_w)/2):img_w-int((img_w-rs_w)/2), 2]
    G = img_in[img_h-rs_h:img_h, int((img_w-rs_w)/2):img_w-int((img_w-rs_w)/2), 1]
    B = img_in[img_h-rs_h:img_h, int((img_w-rs_w)/2):img_w-int((img_w-rs_w)/2), 0]

    img_reshaped[:,:,0] = B
    img_reshaped[:,:,1] = G
    img_reshaped[:,:,2] = R

    return img_reshaped
#!/usr/bin/env python
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Control node of Agricultural Robot Soybot	#
#                                           #
# Author: Adalberto Oliveira                #
# Doctoral in robotic - PUC-Rio             #
# Version: 3.24.10.1                        #
# Date: 2020-10-24                          #
#                                           #
#############################################

import rospy, time, angles,copy, math, tf2_geometry_msgs, tf, sys, cv2
import tf2_ros as tf2
import numpy as np
from numpy import sin, cos
import imageProcess_v2 as imp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def callback_img(msg,position):
    global bridge
    global cv_image_center
    global cv_image_left
    global cv_image_right
    global SIZE_c
    global SIZE_l
    global SIZE_r

    # receive the ros image mesage and convert to bgr, ohta and hsv  
    input_img = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")   

    # resize image
    scale_percent = 0.5 # percent of original size
    width = int(input_img.shape[1] * scale_percent)
    height = int(input_img.shape[0] * scale_percent)
    dim = (width, height)
    resized_img = cv2.resize(input_img, dim, interpolation = cv2.INTER_AREA)

    cv_image_center = imp.image_reshape(resized_img,SIZE_c)

#--------------------------------------------------------------------

def line_follow(u_img, row_end):
    
    global vel_lin

    Vel = Twist()
    
    K_u = 0.002

    Vel.linear.x = vel_lin 
    
    Vel.angular.z = -K_u*u_img

    if abs(Vel.angular.z) > 0.5: Vel.angular.z = np.sign(Vel.angular.z) * 0.5
    
    if row_end:
        Vel.linear.x = 0.001
        Vel.angular.z = 0.1
    
   
    return Vel

#--------------------------------------------------------------------


def img_proc():

    global cmd_vel_pub 
    global tfBuffer
    global mov
    global in_goal
    global inGoal
    global cv_image_center
    global cv_image_left
    global cv_image_right
    global robot_name
    global previous_error
    global current_error

    # Node name
    rospy.init_node('img_proc', anonymous=True) 
    
    # Publishers
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('center_image_raw', Image, callback_img)

    # Control parameters
    rate = rospy.Rate(15) 
    cmdVel = Twist()

    # Transformation parameters
    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)
    time.sleep(4)

    print 'Start image processing and robot control...'
    
    while not rospy.is_shutdown():
        
        # Segmenting the ground
       
        # Processing image for self control with one crop row
        cv_output, mask_show, theta, rho, row_end, H = imp.get_line_1(cv_image_center,5,4)
        
        u = line_follow(rho ,row_end)
        cmd_vel_pub.publish(u)


        # Show image
        cv2.imshow('Robot view ('+robot_name+')',cv_output)
        
        cv2.waitKey(2)
        
        rate.sleep()

    task_pub.publish(0)
    cmdVel.linear.x = 0
    cmdVel.angular.z = 0
    cmd_vel_pub.publish(cmdVel)

#--------------------------------------------------------------------

######### Main #########
# Global Variables
bridge = CvBridge()
robot_name = str(sys.argv[1])
vel_lin = float(sys.argv[2])
Version = '2.12'
SIZE_c = [1, 0.5]

# Initial Banner
print '#############################################'
print '#                                           #'
print '# Control node for Agricultural Robot       #'
print '#                                           #'
print '# Author: Adalberto Oliveira                #'
print '# Doctoral in robotic - PUC-Rio             #'
print '#                                           #'
print '#    2020-10-24                             #'
print '#                                           #'
print '# Robot name: ',robot_name,'                    #'
print '# Linear Velocity:', vel_lin,'                     #'
print '#                                           #'
print '#############################################'



if __name__ == '__main__':

    img_proc()
    '''
    try:
        img_proc()
    except:
        cv2.destroyAllWindows()         
        print 'Node ended.'
    '''

# End Main

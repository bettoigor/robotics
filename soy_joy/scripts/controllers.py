#!/usr/bin/env python
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Controller module class for 				#
# RedCat Agribot            				#
#                                           #
# Author: Adalberto Oliveira                #
# Mastering in robotic - PUC-Rio            #
# Version: 1.2                              #
# Date: 2-13-2019                           #
#                                           #
#############################################

import rospy, time, angles, math, tf2_geometry_msgs, tf, sys
from geometry_msgs.msg import Twist, Pose2D, PointStamped 
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros as tf2


def hello_word(msg):
    print msg


def polar_ctrl_2(position, destination, K_polar):
    # Polar coordinate control function

    # remaping input variables
    x = destination.point.x
    y = destination.point.y 
    theta = position.theta
    theta_goal = round(destination.point.z, 3)

    # Remaping the gain variables
    k_rho = K_polar[0]
    k_alpha = K_polar[1]
    k_beta = K_polar[2]

    # Computing the coordinate transformation
    rho = math.sqrt(x **2 + y**2)
    alpha = math.atan2(y, x)
    beta = round(theta_goal,3) 

    # Defining the position threshold 
    if abs(rho) < 0.15:
        alpha = 0
        rho = 0

    # Computin the control law
    v = (k_rho * rho)
    w = ((k_alpha * alpha) + (k_beta * beta))

    agl = [rho, alpha, beta]    # Secondarie monitiring parameters
    U_uni = [v,w, agl]

    return U_uni


def polar_ctrl(position, destination, K_polar):
    # Polar coordinate control function

    delta_x = destination.point.x
    delta_y = destination.point.y
    k_rho = K_polar[0]
    k_alpha = K_polar[1]
    k_beta = K_polar[2]
    
    rho = round(math.sqrt(math.pow(delta_x,2) + math.pow(delta_y,2)),2)
    #alpha = -angles.shortest_angular_distance(position.theta, math.atan2(delta_y, delta_x))
    alpha =  math.atan2(delta_y, delta_x) - position.theta
    #beta =  position.thet=a
    beta =  - position.theta - math.atan2(delta_y, delta_x)
    
    #v = k_rho * rho * math.cos(alpha)
    '''
    if alpha == 0:
        w = (k_alpha * alpha) + (k_beta * beta)
    else:
        w = k_alpha * alpha + k_rho * ( ((math.sin(alpha) * math.cos(alpha) )/ alpha) * (alpha + (k_beta * beta)))
    '''
    v = k_rho * rho
    w = (k_alpha * alpha) + (k_beta * beta)

    U_uni = [v,w]

    return U_uni

def bicycle_ctrl(U_uni, Bi_parm, PID_v):
    
    K = Bi_parm[0]
    l = Bi_parm[1]
    v_uni = U_uni[0]
    w_uni = U_uni[1]

    v_bi = v_uni # + (0.000001 * integral_v) + (0.015 * derivative_v)
    w_bi = K * (math.atan2((w_uni * l), v_bi))
    
    U_bi = [v_bi, w_bi]

    return U_bi

def PID(K_pid, PID_v):

    last_proportional_v = PID_v[0]
    integral_v = PID_v[1]
    derivative_v = PID_v[2]

    if PID_v[1] > 1000:
        PID_v[1] = 0

    integral_v = PID_v[1] + PID_v[3]
    derivative_v = PID_v[3] - PID_v[0]
    last_proportional_v = PID_v[3]

    v =  K_pid[0] * PID_v[3] + (K_pid[1] * integral_v) + (K_pid[2] * derivative_v)

    PID_v = [last_proportional_v, integral_v, derivative_v, v]
    
    return PID_v

def cartesian_ctrl(destination, K_cart):

    v = K_cart[0] * (math.sqrt(destination.point.x ** 2 + destination.point.y ** 2))
    w = K_cart[1] * (math.atan2(destination.point.y, destination.point.x))

    U_uni = [v, w]
    return U_uni

    pass



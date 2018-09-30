#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


Rc_y = Matrix([[cos(-pi/2),     0,      sin(-pi/2),     0],
              [0,               1,      0,              0],
              [-sin(-pi/2),     0,      cos(-pi/2),     0],
              [0,               0,      0,              1]])

Rc_z = Matrix([[cos(pi),        -sin(pi),   0,  0],
              [sin(pi),         cos(pi),    0,  0],
              [0,               0,          1,  0],
              [0,               0,          0,  1]])

R_corr = Rc_z * Rc_y


def rot_x(q):
    R_x = Matrix([[1, 0, 0, 0],
                  [0, cos(q), -sin(q), 0],
                  [0, sin(q), cos(q), 0],
                  [0, 0,      0,    1]])
    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q), 0],
                  [0, 1, 0, 0],
                  [-sin(q), 0, cos(q), 0],
                  [0, 0, 0, 1]])
    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0, 0],
                  [sin(q), cos(q), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return R_z


# Create Modified DH parameters
alpha0 = 0
a0 = 0
d1 = 0.75
alpha1 = -pi / 2
a1 = 0.35
d2 = 0
alpha2 = 0
a2 = 1.25
d3 = 0
alpha3 = -pi / 2
a3 = -0.054
d4 = 1.50
alpha4 = pi / 2
a4 = 0
d5 = 0
alpha5 = -pi / 2
a5 = 0
d6 = 0
alpha6 = 0
a6 = 0
d7 = 0.303
q7 = 0
t = sqrt(a3 ** 2 + d4 ** 2)
offset = -atan2(a3, d4)


# Define Modified DH Transformation matrix
def dh_matrix(q, a, d, alpha):
    return Matrix([[cos(q), -sin(q), 0, a],
                   [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                   [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                   [0, 0, 0, 1]])


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # Create individual transformation matrices
        T0_1 = dh_matrix(q1, a0, d1, alpha0)
        T1_2 = dh_matrix(q2 - pi / 2, a1, d2, alpha1)
        T2_3 = dh_matrix(q3, a2, d3, alpha2)
        T3_4 = dh_matrix(q4, a3, d4, alpha3)
        T4_5 = dh_matrix(q5, a4, d5, alpha4)
        T5_6 = dh_matrix(q6, a5, d6, alpha5)
        T6_G = dh_matrix(q7, a6, d7, alpha6)

        # Extract rotation matrices from the transformation matrices
        #
        #
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = (rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr).evalf(5)
            EE = Matrix([px, py, pz])
            wc = EE - (d6 + d7) * Rrpy[:3, 2]

            # Calculate joint angles using Geometric IK method
            xc = wc[0]
            yc = wc[1]
            zc = wc[2]

            dir = atan2(yc, xc)
            x = sqrt(xc ** 2 + yc ** 2)
            x_1 = x - a1
            x_2 = x + a1
            z = zc - d1

            solvable_1 = False
            solvable_2 = False
            if sqrt(x_1 ** 2 + z ** 2) <= (a2 + t):
                solvable_1 = True
            if sqrt(x_2 ** 2 + z ** 2) <= (a2 + t):
                solvable_2 = True

            theta1_1 = None
            theta2_1 = None
            theta3_1 = None
            total_1 = 0
            if solvable_1 is True:
                s3_off_1 = (a2 ** 2 + t ** 2 - x_1 ** 2 - z ** 2) / (2 * a2 * t)
                c3_off_1 = sqrt(1 - s3_off_1 ** 2)
                theta3_1 = atan2(s3_off_1, c3_off_1) - offset
                k1_1 = a2 - t * s3_off_1
                k2_1 = t * c3_off_1
                theta2_1 = atan2(x_1, z) - atan2(k2_1, k1_1)
                theta1_1 = dir
                total_1 = abs(theta1_1.evalf()) + abs(theta2_1.evalf()) + abs(theta3_1.evalf())
            theta1_2 = None
            theta2_2 = None
            theta3_2 = None
            total_2 = 0
            if solvable_2 is True:
                s3_off_2 = (a2 ** 2 + t ** 2 - x_2 ** 2 - z ** 2) / (2 * a2 * t)
                c3_off_2 = sqrt(1 - s3_off_2 ** 2)
                theta3_2 = -atan2(s3_off_2, c3_off_2) - offset - pi
                k1_2 = a2 - t * s3_off_2
                k2_2 = t * c3_off_2
                theta2_2 = -(atan2(x_2, z) - atan2(k2_2, k1_2))
                if dir < 0:
                    theta1_2 = dir + pi
                else:
                    theta1_2 = dir - pi
                total_2 = abs(theta1_2.evalf()) + abs(theta2_2.evalf()) + abs(theta3_2.evalf())
            theta1 = theta1_1
            theta2 = theta2_1
            theta3 = theta3_1
            # if (solvable_1 == True and solvable_2 == True) and (total_1 >= total_2):
            #     theta1 = theta1_2
            #     theta2 = theta2_2
            #     theta3 = theta3_2

            T0_3 = T0_1 * T1_2 * T2_3
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
            R0_3 = T0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * Rrpy

            alpha = atan2(R3_6[2, 2], -R3_6[0, 2])
            beta = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            gamma = atan2(-R3_6[1, 1], R3_6[1, 0])

            flipped_alpha = alpha + pi
            flipped_beta = -beta
            flipped_gamma = gamma + pi

            if -pi < gamma < pi:
                theta4 = alpha
                theta5 = beta
                theta6 = gamma
            else:
                theta4 = flipped_alpha
                theta5 = flipped_beta
                theta6 = flipped_gamma

            # if lasts < flipped_lasts:
            #     theta4 = alpha
            #     theta5 = beta
            #     theta6 = gamma
            # else:
            #     theta4 = flipped_alpha
            #     theta5 = flipped_beta
            #     theta6 = flipped_gamma

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()

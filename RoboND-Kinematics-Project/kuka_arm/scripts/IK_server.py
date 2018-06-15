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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        
        # Create symbols
	alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
	a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
	d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
	q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
	
	#  Modified DH parameters
	s = {a0 : 0,      alpha0 : 0,     d1 : 0.75,
	     a1 : 0.35,   alpha1 : -pi/2, d2 : 0,     q2 : q2 - pi/2,
	     a2 : 1.25,   alpha2 :  0,    d3 : 0,
	     a3 : -0.054, alpha3 : -pi/2, d4 : 1.5,
             a4 : 0,      alpha4 : pi/2,  d5 : 0,
	     a5 : 0,      alpha5 : -pi/2, d6 : 0,
             a6 : 0,      alpha6 : 0,     d7 : 0.303, q7: 0}
	#
	#
	#Modified DH Transformation matrix
	
	T0_1 = Matrix([[cos(q1),             -sin(q1),              0,            a0],
	               [sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
		       [sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
		       [0,                    0,                    0,            1]])
	T0_1 = T0_1.subs(s)

	T1_2 = Matrix([[cos(q2),             -sin(q2),              0,            a1],
	               [sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
		       [sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
		       [0,                    0,                    0,            1]])
	T1_2 = T1_2.subs(s)

	T2_3 = Matrix([[cos(q3),             -sin(q3),              0,            a2],
	               [sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
		       [sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
		       [0,                    0,                    0,            1]])
	T2_3 = T2_3.subs(s)

	T3_4 = Matrix([[cos(q4),             -sin(q4),              0,            a3],
	               [sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
		       [sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
		       [0,                    0,                    0,            1]])
	T3_4 = T3_4.subs(s)

	T4_5 = Matrix([[cos(q5),             -sin(q5),              0,            a4],
	               [sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
		       [sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
		       [0,                    0,                    0,            1]])
	T4_5 = T4_5.subs(s)

	T5_6 = Matrix([[cos(q6),             -sin(q6),              0,            a5],
	               [sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
		       [sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
		       [0,                    0,                    0,            1]])
	T5_6 = T5_6.subs(s)

	T6_7 = Matrix([[cos(q7),             -sin(q7),              0,            a6],
	               [sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
		       [sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
		       [0,                    0,                    0,            1]])
	T6_7 = T6_7.subs(s)

	# Create individual transformation matrices
	T0_2 = simplify(T0_1 * T1_2)
	T0_3 = simplify(T0_2 * T2_3)
	T0_4 = simplify(T0_3 * T3_4)
	T0_5 = simplify(T0_4 * T4_5)
	T0_6 = simplify(T0_5 * T5_6)
	T0_7 = simplify(T0_6 * T6_7)
	

	#Rotation Matrix  
        x_angle, y_angle, z_angle = symbols('x_angle y_angle z_angel')	
	
	R_Z = Matrix([[ cos(z_angle), -sin(z_angle), 0, 0],
		      [ sin(z_angle), cos(z_angle),  0, 0],
		      [ 0, 	      0, 	     1, 0],
		      [ 0 ,           0,             0, 1]])

	R_Y = Matrix([[ cos(y_angle), 0, sin(y_angle), 0],
		      [ 0,         1, 0,               0],
		      [-sin(y_angle), 0, cos(y_angle), 0],
		      [ 0,            0, 0,            1]])

	R_X = Matrix([[1, 0,            0,             0],
		      [0, cos(x_angle), -sin(x_angle), 0],
		      [0, sin(x_angle), cos(x_angle),  0],
		      [0, 0,            0,             1]])
	
        #Correction matrix, This matrix is you to transform Gazebo coordinate system to DH coordinate system 
	
	R_corr = simplify(R_Z * R_Y)
	

	T_total = simplify(T0_7 * R_corr)
	#
	#
	# Extract rotation matrices from the transformation matrices
	R0_1 = T0_1[:3,:3]
	R0_2 = T0_2[:3,:3]
	R0_3 = T0_3[:3,:3]
	R0_4 = T0_4[:3,:3]
	R0_5 = T0_5[:3,:3]
	R0_6 = T0_6[:3,:3]
	R0_7 = T0_7[:3,:3]
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
	    # r_corr is the correction matrix require to transform from gazebo coordinate system to DH coordinate system
	    r_corr = R_corr[:3,:3]
	    r_corr = r_corr.evalf(subs = {y_angle: -pi/2, z_angle: pi})
	    r_x = R_X[:3,:3]
	    r_y = R_Y[:3,:3]
	    r_z = R_Z[:3.:3]
            
            # Rotation matrix for end effector (In Gazebo coordinate systme) 
            Rrpy = simplify(r_z*r_y*r_x)

	    R_EE_URDF = Rrpy.evalf(subs = {x_angel: roll, y_angle: pitch, z_angle: yaw})
	    
            # Rotation matrix for end effector (In DH coordinate system)	
	    R_EE_DH = R_EE_URDF * r_corr
	    
            # Wrist center position
	    WC_pos = Matrix([[px],[py],[pz]]) - d7 * R_EE_DH[:,2]

	    # Calculate joint angles using Geometric IK method
            # Look the the triangle abc image in writup 
            # A,B and C are the sides of triangle 
	    theta1 = atan2(WC_pos[1], WC_pos[0])
	    A = 1.5009 
	    C = 1.25
	    B = (((WC_pos[0]**2 + WC_pos[1]**2)**(0.5)-0.35)**2 + (WC_pos[2] - 0.75)**2)**(0.5)#0.35 is a1, 0.75 is the z-coordinate of joint 2 
            # Standered cos rule to find the angles of triangle 
	    a_angle = acos((-A**2 + B**2 + C**2) / (2 * B * C))
	    b_angle = acos((A**2 - B**2 + C**2) / (2 * A * C))
            c_angle = acos((A**2 + B**2 - C**2) / (2 * B * A))

	    theta2 = pi/2 - a_angle - atan2(WC_pos[2] - 0.75, (WC_pos[0]**2 + WC_pos[1]**2)**(0.5) - 0.35) 
	    theta3 = (pi/2 - 0.036)- b_angle # pi/2 - 0.036 is the initial angle between the side A and side B (look the dig1 in readme file)

	    R0_3_val = R0_3.evalf(subs = {q1:theta1, q2:theta2, q3:theta3}) 
	    
            #R3_6 is the rotation matrix for 3rd joint to 6th joint coordinate system  
            R3_6 = R0_3_val.inv('LU') * R_EE_DH
	    
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta5 = atan2((R3_6[0,2]**2 + R3_6[2,2]**2)**0.5, R3_6[1,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
	    
	    #Calculating the error in the position of end-effector
	    EE = T_total.evalf(subs = {q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})
	    EE_pos_cal = [EE[0,3],EE[1,3],EE[2,3]]

	    ee_x_e = abs(EE_pos_cal[0]-px)
            ee_y_e = abs(EE_pos_cal[1]-py)
            ee_z_e = abs(EE_pos_cal[2]-pz)
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
            print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
            print ("End effector error for y position is: %04.8f" % ee_y_e)
            print ("End effector error for z position is: %04.8f" % ee_z_e)
            print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

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

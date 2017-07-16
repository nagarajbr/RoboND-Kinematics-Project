#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols
    
            # Link offset symbols for the DH parameter table
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            # Link length symbols for the DH parameter table
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            # Twist angles for links as symbols for the DH table parameters
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            # Joint angle symbols. The q's are basically the theta angles that we will refer to in subsequent discussions
            q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    
            # Modified DH Transormation Matrix

            dh = {alpha0:     0, a0:      0, d1:  0.75,
                alpha1: -pi/2, a1:   0.35, d2:     0,  q2: q2-pi/2,
                alpha2:     0, a2:   1.25, d3:     0,
                alpha3: -pi/2, a3: -0.054, d4:  1.50,
                alpha4:  pi/2, a4:      0, d5:     0,
                alpha5: -pi/2, a5:      0, d6:     0,
                alpha6:     0, a6:      0, d7: 0.303,  q7: 0 }

            # Create individual transformation matrices
            

            #### Generalized formula to develop individual transformation matrices
            
            def generateMatrix(alpha, a, q, d):
                m =  Matrix([[            cos(q),           -sin(q),           0,             a],
                               [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                               [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                                [                 0,                 0,           0,             1]])

                return m
            
            #### Individual axis rotations
            
            rot_Z = Matrix([[  cos(q), -sin(q),       0 ],
                              [  sin(q),  cos(q),       0 ],
                              [       0,       0,       1 ]]
                         
            rot_Y = Matrix([[  cos(q),       0,  sin(q) ],
                              [       0,       1,       0 ],
                              [ -sin(q),       0,  cos(q) ]])
            
            def rotateZ(q):
                R_z = Matrix([[  cos(q), -sin(q),       0 ],
                              [  sin(q),  cos(q),       0 ],
                              [       0,       0,       1 ]]
                return R_z
                             
            def rotateY(q):
                R_y = Matrix([[  cos(q),       0,  sin(q) ],
                              [       0,       1,       0 ],
                              [ -sin(q),       0,  cos(q) ]])
                return R_y
            
            # Accounts for difference in Gripper reference frame
                         
            R_corr = simplify(rot_Z * rot_Y)
            
            T0_1 = generateMatrix (alpha0, a0, q1, d1).subs(dh)    """Output:
                                                                            Matrix([
                                                                            [cos(q1), -sin(q1), 0,    0],
                                                                            [sin(q1),  cos(q1), 0,    0],
                                                                            [      0,        0, 1, 0.75],
                                                                            [      0,        0, 0,    1]])"""
            T1_2 = generateMatrix(alpha1, a1, q2, d2).subs(dh)
            T2_3 = generateMatrix(alpha2, a2, q3, d3).subs(dh)
            T3_4 = generateMatrix(alpha3, a3, q4, d4).subs(dh)
            T4_5 = generateMatrix(alpha4, a4, q5, d5).subs(dh) 
            T5_6 = generateMatrix(alpha5, a5, q6, d6).subs(dh)
            # Joint 6 to Gripper Transform
            T6_G = generateMatrix(alpha6, a6, q7, d7).subs(dh)
            T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)
            
            # Extract end-effector position and orientation from request
            
            T_total = simplify(T0_G * R_corr)
            
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
                         
            pg = Matrix([[px], [py], [pz]])
     
            # Calculate joint angles using Geometric IK method
            
                         
            #### Step 2:
                         
            dG = dh[d7]
            R0_G = tf.transformations.quaternion_matrix([req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            O5_0 = PG_0 - Matrix(dG*R0_G)
                         
            #### Step 3:
            
            theta1 = atan2(WC_0[1], WC_0[0])
            PO2 = Matrix([[dh[ai]], [0], [dh[d1]]])
            # Rotate PO2 w.r.t. Z by theta1
            PO2 = rotateZ(theta1)* PO2
            # dh[a2] is the length between the wrist center (O5) and O2
            O2_O3 = dh[a2]
                             
            # To calculate the length between the wrist center(O5) and O2
            PO2_O5 = O5_0 - PO2
            O2_O5 = PO2_O5.norm()
                             
            # Distance between O3 and O5/WC = (d4^2 + a3^2)
            O3_O5 = np.sqrt(dh[d4]*dh[d4] + dh[a3]*dh[a3])
                             
            
            # Offset angle between the Y3 axis line(O3, O5), angleO3_offset
            angleO3_offset = atan2(dh[a3], dh[d4])   

            # angle(O3, O2, O5), q21 in figure.
            angle_O2 = np.arccos(float((O2_O3**2 + O2_O5**2 - O3_O5**2)/(2*O2_O3*O2_O5)))

            # angle(O2, O3, O5)
            angle_O3 = np.arccos(float((O2_O3**2 + O3_O5**2 - O2_O5**2)/(2*O2_O3*O3_O5)))

            # Let q1 = theta1
            Z2_Primary = T0_2.subs({q1:theta1}).dot([[0], [0], [1], [0]])
            Z2_Rot =  tf.transformations.rotation_matrix(-angle_O2, Z2_Primary)
            theta2Direction = Z2_Rot * PO2_O5
            X2_Primary = T0_2.subs({q1:theta1, q2:0}).dot([[1], [0], [0], [0]])
            theta2 = np.arccos(float(np.dot(X2_Primary, theta2Direction[0:4])))
            theta3 = ((pi/2 + angleO3_offset) - angle_O3).evalf()

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
                             
            #### Step 4
                             
            R0_3 = T0_3[0:3,0:3]
            R3_6 = (T3_4*T4_5*T5_6)[:3,:3]
            R3_6 = R3_6.subs({q1: theta1, q2:theta2, q3: theta3})
            R0_G = R0_3 * R3_6 * R_corr
            R3_6 = R0_3.transpose() * R0_G* R_corr.transpose()
                             
            R0_3 = T0_3[0:3,0:3]
            R3_6 = R0_3.transpose() * Matrix(R0_g)* R_corr.transpose()
            
            #### Step 5
                             
            theta4 = atan2(R3_6[2,2], -R3_6[0, 2]).evalf() 
            theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf() 
            theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2]).evalf()

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

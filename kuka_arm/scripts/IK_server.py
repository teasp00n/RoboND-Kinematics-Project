#!/usr/bin/env python2

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya, Troy Patrick

# import modules
from kuka_arm.srv import CalculateIKResponse, CalculateIK
from mpmath import radians
from sympy import sqrt, symbols, pi, Matrix, cos, sin, atan2, acos
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import tf


def build_transform(a, d, q, alpha):
    return Matrix([
        [ cos(q),              -sin(q),             0,           a ],
        [ sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d ],
        [ sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),  cos(alpha * d) ],
        [ 0,                   0,                   0,           1 ]
    ])


# NOTE: all of this initialisation need only happen once so we include it
# outside of the service request handler

# link lengths
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

# link offsets
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

# joint angles
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

# twist angles
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# DH parameter table
dh_table = {
    alpha0:       0,  a0:      0,  d1:  0.75,  q1: q1,
    alpha1: -pi / 2,  a1:   0.35,  d2:     0,  q2: -pi / 2 + q2,
    alpha2:       0,  a2:   1.25,  d3:     0,  q3: q3,
    alpha3: -pi / 2,  a3: -0.054,  d4:  1.50,  q4: q4,
    alpha4:  pi / 2,  a4:      0,  d5:     0,  q5: q5,
    alpha5: -pi / 2,  a5:      0,  d6:     0,  q6: q6,
    alpha6:       0,  a6:      0,  d7: 0.303,  q7: 0
}

# transformation matrices between frames
t0_1 = build_transform(a0, d1, q1, alpha0).subs(dh_table)
t1_2 = build_transform(a1, d2, q2, alpha1).subs(dh_table)
t2_3 = build_transform(a2, d3, q3, alpha2).subs(dh_table)
t3_4 = build_transform(a3, d4, q4, alpha3).subs(dh_table)
t4_5 = build_transform(a4, d5, q5, alpha4).subs(dh_table)
t5_6 = build_transform(a5, d6, q6, alpha5).subs(dh_table)
t6_7 = build_transform(a6, d7, q7, alpha6).subs(dh_table)

# Base to end-effector transform
t0_7 = t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7

# roll, pitch, yaw respectively
r, p, y = symbols('r p y')

# build out the rotation matrices for x, y, and z respectively
rot_x = Matrix([
    [ 1, 0,      0 ],
    [ 0, cos(r), -sin(r) ],
    [ 0, sin(r), cos(r) ]
])

rot_y = Matrix([
    [ cos(p),  0, sin(p) ],
    [ 0,       1, 0 ],
    [ -sin(p), 0, cos(p) ]
])

rot_z = Matrix([
    [ cos(y), -sin(y), 0 ],
    [ sin(y), cos(y),  0 ],
    [ 0,      0,       1 ]
])

# multiply them together to yield the cumulative rotation matrix
rot_7 = rot_z * rot_y * rot_x

# apply the correction to the rotation matrix. this adjusts the values into our coordinate space
rot_correction = rot_z.subs(y, radians(180)) * rot_y.subs(p, radians(-90))
rot_7 = rot_7 * rot_correction


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):

            # read in the positions
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # read in the roll, pitch, yaw as euler angles from the received quaternion
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [ req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w ])

            rot_ee = rot_7.subs({ 'r': roll, 'p': pitch, 'y': yaw })

            end_effector = Matrix([ [px], [py], [pz] ])
            wrist_center = end_effector - (0.303) * rot_ee[:, 2]

            theta1 = atan2(wrist_center[1], wrist_center[0])

            # SSS triangles for thetas 2 and 3
            side_a = 1.50
            side_b = sqrt(pow((sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35), 2) + pow((wrist_center[2] - 0.75), 2))
            side_c = 1.25

            # solve the angles in the triangle
            A = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            B = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            D = atan2(wrist_center[2] - 0.75, sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35)

            # we know the angles above plus our angles less any offsets should add to 90 degrees
            theta2 = pi / 2 - (A + D)
            theta3 = pi / 2 - (B + 0.036)  # due to the 0.054 sag in link 4

            # build the rotation matrix from 0 to 3 and sub in our thetas for the first 3 joints
            rot0_3 = t0_1[0:3, 0:3] * t1_2[0:3, 0:3] * t2_3[0:3, 0:3]
            rot0_3 = rot0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # because the matrix is orthonormal we can use transpose instead of inv for the rotation matrix from 3 to 6
            rot3_6 = rot0_3.transpose() * rot_ee

            # solving the euler angles for the rotation matrix from 3 to 6
            theta4 = atan2(rot3_6[2, 2], -rot3_6[0, 2])
            theta5 = atan2(sqrt(rot3_6[0, 2] * rot3_6[0, 2] + rot3_6[2, 2] * rot3_6[2, 2]), rot3_6[1, 2])
            theta6 = atan2(-rot3_6[1, 1], rot3_6[1, 0])

            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.positions = [
                theta1, theta2, theta3, theta4, theta5, theta6
            ]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo(
            "length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()


if __name__ == "__main__":
    IK_server()

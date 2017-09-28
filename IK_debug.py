#!/usr/bin/env python2

from sympy import sqrt, symbols, pi, Matrix, cos, sin, atan2, acos
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:
              [
                  [
                      [2.16135, -1.42635, 1.55109],
                      [0.708611, 0.186356, -0.157931, 0.661967]
                  ],
                  [1.89451, -1.44302, 1.69366],
                  [-0.65, 0.45, -0.36, 0.95, 0.79, 0.49]
              ],
              2:
              [
                  [
                      [-0.56754, 0.93663, 3.0038],
                      [0.62073, 0.48318, 0.38759, 0.480629]
                  ],
                  [-0.638, 0.64198, 2.9988],
                  [-0.79, -0.11, -2.33, 1.94, 1.14, -3.68]
              ],
              3:
              [
                  [
                      [-1.3863, 0.02074, 0.90986],
                      [0.01735, -0.2179, 0.9025, 0.371016]
                  ],
                  [-1.1669, -0.17989, 0.85137],
                  [-2.99, -0.12, 0.94, 4.06, 1.29, -4.12]
              ],
              4: [],
              5: []}


def build_transform(a, d, q, alpha):
    return Matrix([
        [ cos(q),              -sin(q),             0,           a ],
        [ sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d ],
        [ sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),  cos(alpha * d) ],
        [ 0,                   0,                   0,           1 ]
    ])


def test_code(test_case):
    # Set up code
    # Do not modify!
    x = 0

    class Position:
        def __init__(self, EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]

    class Orientation:
        def __init__(self, EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self, position, orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position, orientation)

    class Pose:
        def __init__(self, comb):
            self.poses = [comb]

    def build_transform(a, d, q, alpha):
        return Matrix([
            [ cos(q),              -sin(q),             0,           a ],
            [ sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d ],
            [ sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),  cos(alpha * d) ],
            [ 0,                   0,                   0,           1 ]
        ])

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

    req = Pose(comb)
    start_time = time()

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

    fk = t0_7.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    your_wc = [wrist_center[0], wrist_center[1], wrist_center[2]]  # <--- Load your calculated WC values in this array
    your_ee = [fk[0, 3], fk[1, 3], fk[2, 3]]  # <--- Load your calculated end effector value from your forward kinematics

    # Error analysis
    print("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time() - start_time))

    # Find WC error
    if not(sum(your_wc) == 3):
        wc_x_e = abs(your_wc[0] - test_case[1][0])
        wc_y_e = abs(your_wc[1] - test_case[1][1])
        wc_z_e = abs(your_wc[2] - test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print("\nWrist error for x position is: %04.8f" % wc_x_e)
        print("Wrist error for y position is: %04.8f" % wc_y_e)
        print("Wrist error for z position is: %04.8f" % wc_z_e)
        print("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1 - test_case[2][0])
    t_2_e = abs(theta2 - test_case[2][1])
    t_3_e = abs(theta3 - test_case[2][2])
    t_4_e = abs(theta4 - test_case[2][3])
    t_5_e = abs(theta5 - test_case[2][4])
    t_6_e = abs(theta6 - test_case[2][5])
    print("\nTheta 1 error is: %04.8f" % t_1_e)
    print("Theta 2 error is: %04.8f" % t_2_e)
    print("Theta 3 error is: %04.8f" % t_3_e)
    print("Theta 4 error is: %04.8f" % t_4_e)
    print("Theta 5 error is: %04.8f" % t_5_e)
    print("Theta 6 error is: %04.8f" % t_6_e)
    print("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have multiple positions. It is best to add your forward kinematics to \
           \nconfirm whether your code is working or not**")
    print(" ")

    # Find FK EE error
    if not(sum(your_ee) == 3):
        ee_x_e = abs(your_ee[0] - test_case[0][0][0])
        ee_y_e = abs(your_ee[1] - test_case[0][0][1])
        ee_z_e = abs(your_ee[2] - test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print("End effector error for y position is: %04.8f" % ee_y_e)
        print("End effector error for z position is: %04.8f" % ee_z_e)
        print("Overall end effector offset is: %04.8f units \n" % ee_offset)


if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])

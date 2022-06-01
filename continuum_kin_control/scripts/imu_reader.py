#!/usr/bin/env python

# Listens to IMU data on the 'pubimu' topic and publishes joint angles on the 'joint_state_commands' topic for joint_state_publisher + URDF visualization

import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from imu_data.msg import imu_angles
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply, quaternion_matrix
from sklearn.neighbors import NearestNeighbors

count = 0
N = 8
rate_HZ = 10

q0 = Quaternion(0, 0, 0, 1)
q1 = Quaternion(0, 0, 0, 1)
q2 = Quaternion(0, 0, 0, 1)
q3 = Quaternion(0, 0, 0, 1)
q5 = Quaternion(0, 0, 0, 1)
q4 = Quaternion(0, 0, 0, 1)
q6 = Quaternion(0, 0, 0, 1)
q7 = Quaternion(0, 0, 0, 1)
q0_orig = Quaternion(0, 0, 0, 1)
q1_orig = Quaternion(0, 0, 0, 1)
q2_orig = Quaternion(0, 0, 0, 1)
q3_orig = Quaternion(0, 0, 0, 1)
q4_orig = Quaternion(0, 0, 0, 1)
q5_orig = Quaternion(0, 0, 0, 1)
q6_orig = Quaternion(0, 0, 0, 1)
q7_orig = Quaternion(0, 0, 0, 1)

y_joints = []
z_joints = []
joint_limits = 0.3


# k nearest neighbors algorithm to 
with open("src/continuum_kin_control/scripts/R_sample.csv") as file_name:
    R_sample = np.loadtxt(file_name, delimiter=",")


knn = NearestNeighbors(n_neighbors=1)
knn.fit(R_sample)


joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)

def joint_message(message, z_joints, x_joints):
        message.name = []
        message.position = []

        for i in range(N-1):

            message.name.append("joint" + str(i+1) + "z")
            message.name.append("joint" + str(i+1) + "x")
            message.position.append(z_joints[i])
            message.position.append(x_joints[i])

def callback(data):
    global q0
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    global q7
    global q0_orig
    global q1_orig
    global q2_orig
    global q3_orig
    global q4_orig
    global q5_orig
    global q6_orig
    global q7_orig
    global count

    if (count==0):
        # qbase_orig = Quaternion(data.x0, data.y0, data.z0, data.w0)
        q0_orig = Quaternion(data.x0, data.y0, data.z0, data.w0)
        q1_orig = Quaternion(data.x1, data.y1, data.z1, data.w1)
        q2_orig = Quaternion(data.x2, data.y2, data.z2, data.w2)
        q3_orig = Quaternion(data.x3, data.y3, data.z3, data.w3)
        q4_orig = Quaternion(data.x4, data.y4, data.z4, data.w4)
        q5_orig = Quaternion(data.x5, data.y5, data.z5, data.w5)
        q6_orig = Quaternion(data.x6, data.y6, data.z6, data.w6)
        q7_orig = Quaternion(data.x7, data.y7, data.z7, data.w7)
        count = 1

    q0 = Quaternion(data.x0, data.y0, data.z0, data.w0)
    q1 = Quaternion(data.x1, data.y1, data.z1, data.w1)
    q2 = Quaternion(data.x2, data.y2, data.z2, data.w2)
    q3 = Quaternion(data.x3, data.y3, data.z3, data.w3)
    q4 = Quaternion(data.x4, data.y4, data.z4, data.w4)
    q5 = Quaternion(data.x5, data.y5, data.z5, data.w5)
    q6 = Quaternion(data.x6, data.y6, data.z6, data.w6)
    q7 = Quaternion(data.x7, data.y7, data.z7, data.w7)


def listener():
    global y_joints
    global z_joints
    global q0
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    global q7
    # global qbase_orig
    global q0_orig
    global q1_orig
    global q2_orig
    global q3_orig
    global q4_orig
    global q5_orig
    global q6_orig
    global q7_orig

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)

    rate = rospy.Rate(rate_HZ)

    while not rospy.is_shutdown():
    
        q_orig = [q0_orig, q1_orig, q2_orig, q3_orig, q4_orig, q5_orig, q6_orig, q7_orig]
        q_new = [q0, q1, q2, q3, q4, q5, q6, q7]

        q_tared =[]
        q_adjusted = []

        for i in range(N):
            q_tared.append(q_new[i]*q_orig[i].inverse)

        y_joints = []
        z_joints = []

        for i in range(N-1):
            print("link number: " + str(i+1))
            q_relative = q_tared[i]*q_tared[i+1].inverse
            # print(q_relative)


            R = q_relative.rotation_matrix

            R_test = np.array([
                R[0][0],
                R[1][0],
                R[2][0],
                R[0][1],
                R[1][1],
                R[2][1],
                R[0][2],
                R[1][2],
                R[2][2]
                ])

            R_test = np.reshape(R_test,(1,-1))

            idx = knn.kneighbors(R_test, return_distance=False)

            Rk = R_sample[idx]

            R_final = np.array([
            [Rk[0][0][0], Rk[0][0][3], Rk[0][0][6]],
            [Rk[0][0][1], Rk[0][0][4], Rk[0][0][7]],
            [Rk[0][0][2], Rk[0][0][5], Rk[0][0][8]]
            ])

            R_final = R

            print("R_final: " + str(R_final))

            y_measured = -1*np.arcsin(R_final[1][0])
            z_measured = np.arcsin(R_final[1][2])

            y_angle = y_measured
            z_angle = z_measured

            if y_angle > joint_limits:
                y_angle = joint_limits
            elif y_angle < -joint_limits:
                y_angle = -joint_limits

            if z_angle > joint_limits:
                z_angle = joint_limits
            elif z_angle < -joint_limits:
                z_angle = -joint_limits

            print("y_angle: " + str(y_angle))
            print("z_angle: " + str(z_angle))
            y_joints.append(y_angle)
            z_joints.append(z_angle)

        joint_state_data = JointState()
        joint_message(joint_state_data, z_joints, y_joints)
        joint_state_pub.publish(joint_state_data)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('imu_sim', anonymous=True)
        rospy.Subscriber('pubimu', imu_angles, callback)
        listener()
    except rospy.ROSInterruptException:
        pass

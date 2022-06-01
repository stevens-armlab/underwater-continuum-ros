#!/usr/bin/env python

#Upload quaternions_to_ros.ino to Teensy
import time
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from imu_data.msg import imu_angles
from pyquaternion import Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply, quaternion_matrix
from scipy.spatial.transform import Rotation as R

#Retrieves data from subscribed topic (pubimu)
count = 0
N = 8
rate_HZ = 200

quat0 = [0,0,0,1]
quat1 = [0,0,0,1]
quat2 = [0,0,0,1]
quat3 = [0,0,0,1]
quat4 = [0,0,0,1]
quat5 = [0,0,0,1]
quat6 = [0,0,0,1]
quat7 = [0,0,0,1]
quat_list = [quat0, quat1, quat2, quat3, quat4, quat5, quat6, quat7]
jointx_list = []
jointz_list = []

rospy.init_node('imu_sim', anonymous=True)

def callback(data):
    global qbase
    global q0
    global q1
    global q2
    global q3
    global q4
    global q5
    global q6
    global q7
    global qbase_orig
    global q0_orig
    global q1_orig
    global q2_orig
    global q3_orig
    global q4_orig
    global q5_orig
    global q6_orig
    global q7_orig
    global q7rot

    global count

    q7rot = Quaternion(axis=[1,0,0],angle=np.pi/2)
    while True:
        if (count==1):
            break
        else:
            qbase_orig = Quaternion(data.x0, data.y0, data.z0, data.w0)
            q0_orig = Quaternion(data.x0, data.y0, data.z0, data.w0)
            q1_orig = Quaternion(data.x1, data.y1, data.z1, data.w1)
            q2_orig = Quaternion(data.x2, data.y2, data.z2, data.w2)
            q3_orig = Quaternion(data.x3, data.y3, data.z3, data.w3)
            q4_orig = Quaternion(data.x4, data.y4, data.z4, data.w4)
            q5_orig = Quaternion(data.x5, data.y5, data.z5, data.w5)
            q6_orig = Quaternion(data.x6, data.y6, data.z6, data.w6)
            q7_orig = Quaternion(data.x7, data.y7, data.z7, data.w7)
            # q7_orig = q7_orig*q7rot.inverse
            count = 1

    global quat_list
    global R_list
    global jointx_list
    global jointz_list
    imu = Imu()



    def joint_message(message, z_joints, x_joints):
        message.name = []
        message.position = []

        for i in range(7):
            #z_joints = z_joints[:8]
            #x_joints = x_joints[:8]
            message.name.append("joint" + str(i+1) + "z")
            message.name.append("joint" + str(i+1) + "x")
            message.position.append(z_joints[i])
            message.position.append(x_joints[i])
            #print(i)
            #print(z_joints[i])
            #print(x_joints[i])
            #time.sleep(1)

    joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)

    q0 = Quaternion(data.x0, data.y0, data.z0, data.w0)
    q1 = Quaternion(data.x1, data.y1, data.z1, data.w1)
    q2 = Quaternion(data.x2, data.y2, data.z2, data.w2)
    q3 = Quaternion(data.x3, data.y3, data.z3, data.w3)
    q4 = Quaternion(data.x4, data.y4, data.z4, data.w4)
    q5 = Quaternion(data.x5, data.y5, data.z5, data.w5)
    q6 = Quaternion(data.x6, data.y6, data.z6, data.w6)
    q7 = Quaternion(data.x7, data.y7, data.z7, data.w7)
    # q7 = q7*q7rot.inverse

    q_orig = [q0_orig, q1_orig, q2_orig, q3_orig, q4_orig, q5_orig, q6_orig, q7_orig]
    q_new = [q0, q1, q2, q3, q4, q5, q6, q7]

    q_tared =[]
    q_adjusted = []
    z_joints = []
    x_joints = []

    # print(q7rot)
    #Tare all of the imus
    for i in range(N):
            q_tared.append(q_new[i]*q_orig[i].inverse)
            #print("q" + str(i))
            #print(q_tared[i])
            #print("")

    # print q_tared[0]
    # print q_tared[1]
    for i in range(N-1):
        q_relative = q_tared[i]*q_tared[i+1].inverse
        # if i == 6:
            # q_relative = q_relative*q7rot
        (roll, pitch, yaw) = euler_from_quaternion(q_relative.elements)

        #z_joints.append(roll/(np.pi/.3))
        #x_joints.append(yaw/(np.pi/.3))

        #Roll and pitch are in radians by default
        #print(q_relative)
        #print(roll)
        #print(yaw

        #Adjust range of data.z and data.x to be between -0.3 and 0.3 (the current limits of the urdf)
        scale_factor = 2.8
        if (roll*(180/np.pi)) > 0:
            z_joints.append((roll-np.pi)/(np.pi/scale_factor))
        else:
            z_joints.append((roll+np.pi)/(np.pi/scale_factor))
        #z_joints.append((roll)/(np.pi/.3))
        x_joints.append(-yaw/(np.pi/scale_factor))
        #time.sleep(1)

    # z_joints.insert(0, z_joints[1]) #I am cheating here! This is inserting angle for first joint
    # x_joints.insert(0, x_joints[1]) #until we have an imu installed in the end link
    # print q_relative[0]
    joint_state_data = JointState()
    joint_message(joint_state_data, z_joints, x_joints)
    joint_state_pub.publish(joint_state_data)

def listener():
    rospy.Subscriber('pubimu', imu_angles, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

#Prevents the buffer from being overloaded with messages
while not rospy.is_shutdown():
    	rate.sleep()

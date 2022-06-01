#!/usr/bin/env python

##Upload quaternions_to_ros.ino to Teensy
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

joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)

def callback(data):
    global quat0
    global quat1
    global quat2
    global quat3
    global quat4
    global quat5
    global quat6
    global quat7
    global quat_list
    global R_list
    global jointx_list
    global jointz_list

    quat0 = [data.x0, data.y0, data.z0, data.w0]
    quat1 = [data.x1, data.y1, data.z1, data.w1]
    quat2 = [data.x2, data.y2, data.z2, data.w2]
    quat3 = [data.x3, data.y3, data.z3, data.w3]
    quat4 = [data.x4, data.y4, data.z4, data.w4]
    quat5 = [data.x5, data.y5, data.z5, data.w5]
    quat6 = [data.x6, data.y6, data.z6, data.w6]
    quat7 = [data.x7, data.y7, data.z7, data.w7]
    quat_list = [quat0, quat1, quat2, quat3, quat4, quat5, quat6, quat7]
    R_list = []
    jointx_list = []
    jointz_list = []

    for i in range(N):
        r = R.from_quat(quat_list[i])
        R_list.append(r.as_matrix())

    for i in range(N-1):
        R_y = np.matrix([[0,0,1],[0,1,0],[-1,0,0]])
        R_rel = np.dot(R_y, np.dot(np.linalg.inv(R_list[i]), R_list[i+1]))
        r = R.from_dcm(R_rel)
        euler = r.as_euler('xyz')
        if euler[0] > np.pi*0.8:
            roll = np.pi - euler[0]
        elif euler[0] < -np.pi*0.8:
            roll = np.pi + euler[0]
        else:
            roll = euler[0]

        if euler[2] > np.pi*0.8:
            yaw = np.pi - euler[2]
        elif euler[2] < -np.pi*0.8:
            yaw = np.pi + euler[2]
        else:
            yaw = euler[0]

        jointx_list.append(roll)
        jointz_list.append(yaw)



def joint_message(message, jointx, jointz):
    message.name = []
    message.position = []

    for i in range(N-1):
        message.name.append("joint" + str(i+1) + "z")
        message.name.append("joint" + str(i+1) + "x")
        message.position.append(jointz[i])
        message.position.append(jointx[i])




def listener():
    rate = rospy.Rate(rate_HZ)

    while not rospy.is_shutdown():
        global quat_list
        global jointx_list
        global jointz_list

        joint_state_data = JointState()
        if len(jointx_list) == N-1:
            joint_message(joint_state_data, jointx_list, jointz_list)
            joint_state_pub.publish(joint_state_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('imu_joint_publisher', anonymous=True)
        rospy.Subscriber('pubimu', imu_angles, callback)
        listener()
    except rospy.ROSInterruptException:
        pass


#Prevents the buffer from being overloaded with messages

#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from teensy_quad_motor_msgs.msg import MotorData, QuadMotorData
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from inverse_function import inverse, pose
from qdot_function import qdot
from continuum_kin_control.msg import TrajMsg2
from potentiometer_data.msg import potData

rate_HZ = 200
l = 18
r = 21
n = 8
pitch = 4 # mm/rev
period = 50
max_vel = 30 # rpm
phi1 = 0;
theta1 = 0;
phi2 = 0;
theta2 = 0;


def traj_cb(data):
	global phi1
	global theta1
	global phi2
	global theta2

	phi1 = data.Phi1
	theta1 = data.Theta1
	phi2 = data.Phi2
	theta2 = data.Theta2


def joint_message(message, jointz, jointx, arm):
	message.name = []
	message.position = []

	for i in range(n):
		message.name.append("arm" + str(arm) + "joint" + str(i+1) + "z")
		message.name.append("arm" + str(arm) + "joint" + str(i+1) + "x")
		message.position.append(jointz)
		message.position.append(jointx)


def follow_trajectory():
	global phi1
	global theta1
	global phi2
	global theta2

	rospy.init_node('motor_command_test', anonymous=True)

	arm1_joint_state_pub = rospy.Publisher('arm1_joint_states_command', JointState, queue_size=1)
	arm2_joint_state_pub = rospy.Publisher('arm2_joint_states_command', JointState, queue_size=1)
	rospy.Subscriber('trajectory_cmd', TrajMsg2, traj_cb)

	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():
		arm1_jointz = theta1/(n-1)*np.cos(phi1)
		arm1_jointx = theta1/(n-1)*np.sin(phi1)
		arm1_joint_state_data = JointState()
		joint_message(arm1_joint_state_data, arm1_jointz, arm1_jointx, 1)
		arm1_joint_state_pub.publish(arm1_joint_state_data)

		arm2_jointz = theta2/(n-1)*np.cos(phi2)
		arm2_jointx = theta2/(n-1)*np.sin(phi2)
		arm2_joint_state_data = JointState()
		joint_message(arm2_joint_state_data, arm2_jointz, arm2_jointx, 2)
		arm2_joint_state_pub.publish(arm2_joint_state_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		follow_trajectory()
	except rospy.ROSInterruptException:
		pass

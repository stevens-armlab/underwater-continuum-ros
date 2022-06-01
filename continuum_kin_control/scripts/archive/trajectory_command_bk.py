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
from continuum_kin_control.msg import TrajMsg
from potentiometer_data.msg import potData

motor_pos = [0,0,0,0]
motor_cmd = [0,0,0,0]
pot_volt = [0,0,0,0]
pot_pos = [0,0,0,0]
home = 1.9*100/3.3

l = 18
r = 21
n = 8
pitch = 4 # mm/rev
period = 50
max_vel = 30 # rpm
phi = 0;
theta = 0;


def motor_pos_cb(data):
	for i in range(4):
		motor_pos[i] = data.motors[i].position


def traj_cb(data):
	global phi
	global theta

	phi = data.Phi
	theta = data.Theta


def pot_cb(data):
	global pot_volt
	global pot_pos
	pot_volt = [data.Pot1, data.Pot2, data.Pot3, data.Pot4]
	pot_pos = [x*100/3.3 for x in pot_volt]


def check_position_error(motor_cmd):
	pos_error = [0,0,0,0]
	for idx, value in enumerate(motor_cmd):
		pos_error[idx] = np.abs(motor_cmd[idx] - motor_pos[idx])

	if max(pos_error) > 45:
		return False
	else:
		return True


def pose_message(message, position, orientation):
	message.position.x = position[0]
	message.position.y = position[1]
	message.position.z = position[2]

	message.orientation.x = orientation[0]
	message.orientation.y = orientation[1]
	message.orientation.z = orientation[2]
	message.orientation.w = orientation[3]


def joint_message(message, jointz, jointx):
	message.name = []
	message.position = []

	for i in range(n):
		message.name.append("joint" + str(i+1) + "z")
		message.name.append("joint" + str(i+1) + "x")
		message.position.append(jointz)
		message.position.append(jointx)


def follow_trajectory():
	global phi
	global theta
	rospy.init_node('motor_command_test', anonymous=True)

	motor_cmd_pub = rospy.Publisher('quad_motor_command', QuadMotorData, queue_size=1)
	cmd_reached_pub = rospy.Publisher('cmd_reached', Bool, queue_size=1)
	joint_state_pub = rospy.Publisher('joint_states_command', JointState, queue_size=1)
	pose_pub = []
	for i in range(n):
		topic_name = 'pose_arm/link' + str(i+1)
		pose_pub.append(rospy.Publisher(topic_name, Pose, queue_size=1))

	rospy.Subscriber('motor_positions', QuadMotorData, motor_pos_cb)
	rospy.Subscriber('trajectory_cmd', TrajMsg, traj_cb)
	rospy.Subscriber('potpub', potData, pot_cb)

	rate_HZ = 200
	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():
		Q_desired = inverse(l, r, n, theta, phi) # returns the motor positions to achiejuve the desired theta/phi, in mmfor idx, value in enumerate(motor_cmd)
		motor_cmd_data = QuadMotorData()
		for idx, value in enumerate(motor_cmd):
			motor_cmd[idx] = motor_pos[idx] + 360/pitch*(Q_desired[idx] + pot_pos[idx] - home)
			motor_cmd_data.motors[idx].position = motor_cmd[idx]
		motor_cmd_pub.publish(motor_cmd_data)


		# we need to check if the current motor_cmd would be too far to give
		cmd_reached = Bool()
		if check_position_error(motor_cmd):
			cmd_reached.data = True
		else:
			cmd_reached.data = False
		cmd_reached_pub.publish(cmd_reached)


		pose_test = pose(l, r, n, theta, phi)
		pose_data = []
		for i in range(n):
			pose_data.append(Pose())
			pose_message(pose_data[i], pose_test[0][:,i], pose_test[1][:,i])
		for i in range(n):
			pose_pub[i].publish(pose_data[i])


		jointz = theta/(n-1)*np.cos(phi)
		jointx = theta/(n-1)*np.sin(phi)
		joint_state_data = JointState()
		joint_message(joint_state_data,jointz,jointx)
		joint_state_pub.publish(joint_state_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		follow_trajectory()
	except rospy.ROSInterruptException:
		pass

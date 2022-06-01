#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Bool
from continuum_kin_control.msg import TrajMsg2
from geometry_msgs.msg import Twist

l = 18
r = 21
n = 8
pitch = 4 # mm/rev
period = 50
max_vel = 30 # rpm
cmd_reached = False
theta1 = 0
phi1 = 0
theta2 = 0
phi2 = 0
key_up = False
key_down = False
key_left = False
key_right = False
thetadot = 0.1
phidot = 0.1


def create_trajectory():
	global theta1
	global phi1
	global theta2
	global phi2
	global cmd_reached

	rospy.init_node('trajectory_planner', anonymous=True)

	traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg2, queue_size=1)
	
	rate_HZ = 200
	dt = 1.0/rate_HZ
	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():
		theta1 = np.pi/4
		theta2 = np.pi/4

		phi1 = phi1 + 0.01
		phi2 = phi2 + 0.01

		traj_cmd_data = TrajMsg2()
		traj_cmd_data.Phi1 = phi1
		traj_cmd_data.Theta1 = theta1
		traj_cmd_data.Phi2 = phi2
		traj_cmd_data.Theta2 = theta2

		traj_cmd_pub.publish(traj_cmd_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		create_trajectory()
	except rospy.ROSInterruptException:
		pass

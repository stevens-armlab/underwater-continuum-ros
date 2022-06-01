#!/usr/bin/env python

import numpy as np
import rospy
import time
from std_msgs.msg import Bool
from continuum_kin_control.msg import TrajMsg
# from jacobian_function import jacobian
from position_input import closest_position
from trajectory_input import polynomial_trajectory
from inverse_function import pose

l = 18
r = 21
n = 8

thetaf = 0
phif = 0

def create_trajectory():

	global cmd_reached
	global thetaf
	global phif
	
	rospy.init_node('trajectory_planner', anonymous=True)

	traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg, queue_size=1)

	rate_HZ = 100
	dt = 1.0/rate_HZ
	rate = rospy.Rate(rate_HZ) # 10hz
	t_start = rospy.get_time()
	while not rospy.is_shutdown():

		traj_cmd_data = TrajMsg()

		theta0 = thetaf
		phi0 = phif

		xdes = input("Enter X value (mm):")
		ydes = input("Enter Y value (mm):")
		zdes = input("Enter Z value (mm):")
		psi = closest_position(xdes, ydes, zdes)
		thetaf = psi.item(0)
		phif = psi.item(1)
		X = pose(l, r, n, thetaf, phif)

		print("Actual goal pose: " + str(X) + " mm")
		print("Goal psi: " + str(psi))
		polynomial_trajectory(theta0, phi0, thetaf, phif)

		# print(psi)
		# print(X)

		# traj_cmd_data.Phi = phi
		# traj_cmd_data.Theta = theta

		# traj_cmd_pub.publish(traj_cmd_data)

		time.sleep(1)
		rate.sleep()


if __name__ == '__main__':
	try:
		create_trajectory()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python

# Generates 5th order polynomial to desired trajectory


import numpy as np
import rospy
from std_msgs.msg import Bool
from continuum_kin_control.msg import TrajMsg
from inverse_function import inverse


# T_theta = 20
# T_phi = 20
cmd_reached = False
theta = 0
phi = 0

l = 18
r = 21
n = 8

phi0 = 0
theta0 = 0
phif = 0
thetaf = 0

max_rpm = 104*0.4 # rev/min
pitch = 1.270 # 1/4-20 thread pitch in mm/rev
max_vel = max_rpm*pitch/60 # mm/s

def max_joint(theta0, phi0, thetaf, phif):
	N = 200
	theta = np.linspace(theta0, thetaf, num=N)
	phi = np.linspace(phi0, phif, num=N)

	qmin = inverse(l, r, n, theta0, phi0)
	qmax = inverse(l, r, n, theta0, phi0)
	qdiff = [0]*4

	for i in range(0, N):
		Q = inverse(l, r, n, theta[i], phi[i])
		for j in range(0,4):
			if Q[j] > qmax[j]:
				qmax[j] = Q[j]
			if Q[j] < qmin[j]:
				qmin[j] = Q[j]


	for j in range(0,4):
		qdiff[j] = qmax[j] - qmin[j]

	return np.amax(qdiff)

def polynomial_trajectory(theta0, phi0, thetaf, phif):
	global cmd_reached
	traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg, queue_size=1)
	traj_cmd_data = TrajMsg()

	max_motor = max_joint(theta0, phi0, thetaf, phif)
	T_phi = max_motor/max_vel
	T_theta = T_phi

	print("Maximum motor displacement: " + str(max_motor) + " mm")
	print("Polynomial transition time: " + str(T_phi) + " s")

	phi_reached = False
	theta_reached = False
	cmd_reached = False
	# print("Moving to phi = " + str(round(phif,2)) + " rad, theta = " + str(round(thetaf,2)) + " rad...")
	start = rospy.get_time()

	a0 = phi0
	a3 = -10*(phi0 - phif)/T_phi**3
	a4 = 15*(phi0 - phif)/T_phi**4
	a5 = -6*(phi0 - phif)/T_phi**5

	b0 = theta0
	b3 = -10*(theta0 - thetaf)/T_theta**3
	b4 = 15*(theta0 - thetaf)/T_theta**4
	b5 = -6*(theta0 - thetaf)/T_theta**5

	while 1:
		now = rospy.get_time()
		t_phi = now - start
		t_theta = t_phi

		if t_phi < T_phi:
			phi = a0 + a3*t_phi**3 + a4*t_phi**4 + a5*t_phi**5
		else:
			phi = phif
			phi_reached = True

		if t_theta < T_theta:
			theta = b0 + b3*t_theta**3 + b4*t_theta**4 + b5*t_theta**5
		else:
			theta = thetaf
			theta_reached = True

		traj_cmd_data.Theta = theta
		traj_cmd_data.Phi = phi
		traj_cmd_pub.publish(traj_cmd_data)

		if (phi_reached == True and theta_reached == True):
			cmd_reached = True
			break


def create_trajectory():
	# global theta
	# global phi
	global thetaf
	global phif
	# global cmd_reached
	# global joy_lr
	# global joy_ud
	# global kill
	# global kill_button
	# global kill_switch
	# global timecount
	
	rospy.init_node('trajectory_planner', anonymous=True)

	# traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg, queue_size=1)

	rate_HZ = 100
	dt = 1.0/rate_HZ
	rate = rospy.Rate(rate_HZ) # 10hz
	t_start = rospy.get_time()
	while not rospy.is_shutdown():

		phi0 = phif
		theta0 = thetaf

		phif = input("Enter phi value (deg):")*np.pi/180
		thetaf = input("Enter theta value (deg):")*np.pi/180

		polynomial_trajectory(theta0, phi0, thetaf, phif)

		# if cmd_reached:
		# 	phif = input("Enter phi value (deg):")*np.pi/180
		# 	thetaf = input("Enter theta value (deg):")*np.pi/180
		# 	phi0 = phi
		# 	theta0 = theta

		# 	phi_reached = False
		# 	theta_reached = False
		# 	cmd_reached = False
		# 	print("Moving to phi = " + str(round(phif,2)) + " rad, theta = " + str(round(thetaf,2)) + " rad...")
		# 	start = rospy.get_time()
		# 	a0 = phi0
		# 	a3 = -10*(phi0 - phif)/T_phi**3
		# 	a4 = 15*(phi0 - phif)/T_phi**4
		# 	a5 = -6*(phi0 - phif)/T_phi**5

		# 	b0 = theta0
		# 	b3 = -10*(theta0 - thetaf)/T_theta**3
		# 	b4 = 15*(theta0 - thetaf)/T_theta**4
		# 	b5 = -6*(theta0 - thetaf)/T_theta**5
		
		# now = rospy.get_time()

		# t_phi = now - start
		# t_theta = t_phi

		# if t_phi < T_phi:
		# 	phi = a0 + a3*t_phi**3 + a4*t_phi**4 + a5*t_phi**5
		# else:
		# 	phi = phif
		# 	phi_reached = True


		# if t_theta < T_theta:
		# 	theta = b0 + b3*t_theta**3 + b4*t_theta**4 + b5*t_theta**5
		# else:
		# 	theta = thetaf
		# 	theta_reached = True


		# if (phi_reached == True and theta_reached == True):
		# 	cmd_reached = True


		# traj_cmd_data.Phi = phi
		# traj_cmd_data.Theta = theta

		# print("Phi: " + str(phi))
		# print("Theta: " + str(theta))
		# traj_cmd_pub.publish(traj_cmd_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		create_trajectory()
	except rospy.ROSInterruptException:
		pass

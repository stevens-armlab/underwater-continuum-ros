#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Bool
from continuum_kin_control.msg import TrajMsg
# from sensor_msgs.msg import Joy

l = 18
r = 21
n = 8
pitch = 4 # mm/rev
period = 2000
max_vel = 30 # rpm
cmd_reached = False
theta = 0
phi = 0
joy_lr = 0
joy_ud = 0
thetadot = 0.2
phidot = 0.1
kill = False
kill_button = False
kill_count = False
home_button = False

theta1 = 0*np.pi/180
phi1 = 0*np.pi/180
theta2 = 90*np.pi/180
phi2 = 0*np.pi/180
theta3 = 90*np.pi/180
phi3 = 90*np.pi/180
theta4 = 0*np.pi/180
phi4 = 90*np.pi/180

timecount = 0


def cmd_reached_cb(data):
	cmd_reached = data


# def joy_cb(data):
# 	global joy_lr
# 	global joy_ud
# 	global kill_button
# 	global home_button

# 	joy_lr = data.axes[0]
# 	joy_ud = data.axes[1]
# 	kill_button = data.buttons[0]
# 	home_button = data.buttons[3]


def create_trajectory():
	global theta
	global phi
	global cmd_reached
	global joy_lr
	global joy_ud
	global kill
	global kill_button
	global kill_switch
	global timecount
	
	rospy.init_node('trajectory_planner', anonymous=True)

	traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg, queue_size=1)

	rospy.Subscriber('cmd_reached', Bool, cmd_reached_cb)
	# rospy.Subscriber('joy', Joy, joy_cb)
	
	rate_HZ = 200
	dt = 1.0/rate_HZ
	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():
		traj_cmd_data = TrajMsg()
		timecount = timecount + 1

		if timecount <= period:
			phi = phi1 + (phi2 - phi1)*timecount/period
			theta = theta1 + (theta2 - theta1)*timecount/period
		elif timecount <= 2*period:
			phi = phi2 + (phi3 - phi2)*(timecount - period)/period
			theta = theta2 + (theta3 - theta2)*(timecount - period)/period
		elif timecount <= 3*period:
			phi = phi3 + (phi4 - phi3)*(timecount - 2*period)/period
			theta = theta3 + (theta4 - theta3)*(timecount - 2*period)/period
		elif timecount <= 4*period:
			phi = phi4 + (phi1 - phi4)*(timecount - 3*period)/period
			theta = theta4 + (theta1 - theta4)*(timecount - 3*period)/period
		else:
			timecount = 0
		# phiinput = input("Enter phi value: ")
		# thetainput = input("Enter theta value: ")

		traj_cmd_data.Phi = phi
		traj_cmd_data.Theta = theta


		traj_cmd_pub.publish(traj_cmd_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		create_trajectory()
	except rospy.ROSInterruptException:
		pass

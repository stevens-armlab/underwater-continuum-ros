#!/usr/bin/env python

# Listens to joystick commands from 'joy' topic and publishes to 'trajectory_cmd' topic

import numpy as np
import rospy
from std_msgs.msg import Bool
from continuum_kin_control.msg import TrajMsg
from sensor_msgs.msg import Joy

l = 18
r = 21
n = 8
pitch = 4 # mm/rev
period = 50
max_vel = 30 # rpm
cmd_reached = False
theta = 0
phi = 0
joy_lr = 0
joy_ud = 0
thetadot = 0.3
phidot = 0.3
kill = False
kill_button = False
kill_count = False
home_button = False

traj_cmd_pub = rospy.Publisher('trajectory_cmd', TrajMsg, queue_size=1)
traj_cmd_data = TrajMsg()

def shutdownhook():
	global traj_cmd_pub
	global traj_cmd_data

	print("shutting down...")

	traj_cmd_data.Phi = 0
	traj_cmd_data.Theta = 0
		
	traj_cmd_pub.publish(traj_cmd_data)
	print("Trajectory command reset!")


def cmd_reached_cb(data):
	cmd_reached = data


def joy_cb(data):
	global joy_lr
	global joy_ud
	global kill_button
	global home_button

	joy_lr = data.axes[6]
	joy_ud = data.axes[7]
	kill_button = data.buttons[0]
	home_button = data.buttons[3]


def create_trajectory():
	global theta
	global phi
	global traj_cmd_data
	global traj_cmd_pub
	global cmd_reached
	global joy_lr
	global joy_ud
	global kill
	global kill_button
	global kill_switch
	
	rospy.init_node('trajectory_planner', anonymous=True)
	rospy.Subscriber('cmd_reached', Bool, cmd_reached_cb)
	rospy.Subscriber('joy', Joy, joy_cb)
	
	rate_HZ = 200
	dt = 1.0/rate_HZ
	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():
		if kill_button:
			print("kill switch activated")
			if not kill_switch:
				kill = not kill

			kill_switch = True
		else:
			kill_switch = False
			print("kill switch disabled")


		if not kill:
			
			theta = theta + thetadot*dt*joy_ud
			phi = phi + phidot*dt*joy_lr

			if theta > np.pi/2:
				theta = np.pi/2
			if theta < 0:
				theta = 0

			if home_button == True:
				theta = 0
				phi = 0

		traj_cmd_data.Phi = phi
		traj_cmd_data.Theta = theta


		traj_cmd_pub.publish(traj_cmd_data)

		rate.sleep()


if __name__ == '__main__':
	try:
		create_trajectory()
		rospy.on_shutdown(shutdownhook)
	except rospy.ROSInterruptException:
		pass

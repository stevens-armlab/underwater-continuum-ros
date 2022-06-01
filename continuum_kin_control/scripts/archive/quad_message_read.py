#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from teensy_quad_motor_msgs.msg import MotorData, QuadMotorData

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

motor_pos = [0,0,0,0]
motor_cmd = [0,0,0,0]

t_pos = []
t_cmd = []
pos1 = []
pos2 = []
pos3 = []
pos4  = []
cmd1 = []
cmd2 = []
cmd3 = []
cmd4  = []



def motor_pos_cb(data):
	for i in range(4):
		motor_pos[i] = data.motors[i].position

def motor_cmd_cb(data):
	for i in range(4):
		motor_cmd[i] = data.motors[i].position

def readMessages():
	plt.close('all')

	rospy.init_node('message_reader',anonymous=True)
	rospy.Subscriber('motor_positions', QuadMotorData, motor_pos_cb)
	rospy.Subscriber('quad_motor_command',QuadMotorData, motor_cmd_cb)

	rate_HZ = 100
	rate = rospy.Rate(rate_HZ)
	start_time = rospy.get_time()

	while not rospy.is_shutdown():
		current_time = rospy.get_time() - start_time
		
		ani1 = FuncAnimation(plt.gcf(),plotMotor1,interval=rate_HZ)
		# ani2 = FuncAnimation(fig,plotMotor2,interval=rate_HZ)
		
		plt.show()
		rate.sleep()


def plotMotor1(i):
	t = rospy.get_time()
	t_pos.append(t)
	t_cmd.append(t)

	pos1.append(motor_pos[0])
	cmd1.append(motor_cmd[0])

	plt.cla()
	
	plt.plot(t_pos, pos1)
	plt.plot(t_cmd, cmd1)

def plotMotor2(i):
	t = rospy.get_time()
	# t_pos.append(t)
	# t_cmd.append(t)

	pos2.append(motor_pos[1])
	cmd2.append(motor_cmd[1])

	# plt.figure(2)
	plt.cla()
	plt.plot(t_pos, pos2)
	plt.plot(t_cmd, cmd2)



if __name__ == '__main__':
	try:
		readMessages()
	
	except rospy.ROSInterruptException:
		pass

# plt.style.use('fivethirtyeight')

# x_vals = []
# y_vals = []

# index = count()

# def animate(i):
# 	x_vals.append(next(index))
# 	y_vals.append(random.randint(0,5))

# 	plt.cla()
# 	plt.plot(x_vals, y_vals)


# ani = FuncAnimation(plt.gcf(), animate, interval=1000)

# plt.tight_layout
# plt.show()

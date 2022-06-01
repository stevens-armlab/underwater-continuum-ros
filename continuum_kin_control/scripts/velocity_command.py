#!/usr/bin/env python

# Listens to 'trajectory_cmd' topic and publishes to 'quad_motor_command' topic.

import numpy as np
import rospy
from std_msgs.msg import String, Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import JointState
from teensy_quad_motor_msgs.msg import MotorData, QuadMotorData
# from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from inverse_function import inverse, pose
from qdot_function import qdot
from continuum_kin_control.msg import TrajMsg
from potentiometer_data.msg import potData

n_motors = 4
motor_pos = [0]*n_motors
motor_cmd = [0]*n_motors
motor_home = [0]*n_motors
pos_des = [0]*n_motors
pot_volt = [0]*n_motors
pot_pos = [0]*n_motors
pot_home = [1.7*150/3.3]*n_motors
volt_max = 2.9
volt_min = 0.7
# Q_max = 90
# Q_min = -40
rate_HZ = 200
l = 18
r = 21
n = 8

# pitch = 8.382 # 1/4-12 4x thread pitch in mm/rev
pitch = 1.270 # 1/4-20 thread pitch in mm/rev
# period = 50
max_vel = 300 # rpm
gain = 60
gain_home = 30
phi = 0;
theta = 0;

motor_cmd_pub = rospy.Publisher('quad_motor_command', QuadMotorData, queue_size=1)
motor_cmd_data = QuadMotorData()
cmd_reached_pub = rospy.Publisher('cmd_reached', Bool, queue_size=1)
cmd_reached_data = False
error_pub = rospy.Publisher('motor_error', Float32MultiArray, queue_size=1)

error_pub_data = Float32MultiArray()
error_pub_data.layout.dim.append(MultiArrayDimension())
error_pub_data.data = [0]*n_motors


def shutdownhook():
	global motor_cmd
	global motor_cmd_data
	global motor_cmd_pub
	print("shutting down...")

	for idx, value in enumerate(motor_cmd):
		motor_cmd[idx] = 0
		motor_cmd_data.motors[idx].velocity = motor_cmd[idx]
		
	motor_cmd_pub.publish(motor_cmd_data)
	print("velocity command reset!")


def motor_pos_cb(data):
	global motor_pos

	for idx, value in enumerate(motor_pos):
		motor_pos[idx] = data.motors[idx].position

def traj_cb(data):
	global phi
	global theta

	phi = data.Phi
	theta = data.Theta


def pot_cb(data):
	global pot_volt
	global pot_pos
	pot_volt = [data.Pot1, data.Pot2, data.Pot3, data.Pot4]
	pot_pos = [x*150/3.3 for x in pot_volt]


def check_pot_pos_error(volt, thresh):
	pos_error = [0]*n_motors
	for idx, value in enumerate(volt):
		pos_error[idx] = abs(volt[idx] - 1.7)

	return all(x < thresh for x in pos_error)


def check_position_error(error, thresh):
	return all(x < thresh for x in error)


def check_position_limits(volt):
	global motor_cmd
	for idx, value in enumerate(motor_cmd):
		if(volt[idx] > volt_max) and (motor_cmd[idx] < 0):
			print("Motor " + str(idx) + " out of range, overriding velocity...")
			motor_cmd[idx] = 0
		if(volt[idx] < volt_min) and (motor_cmd[idx] > 0):
			print("Motor " + str(idx) + " out of range, overriding velocity...")
			motor_cmd[idx] = 0



# def check_position_error(motor_cmd, motor_pos):
# 	pos_error = [0,0,0,0]
# 	for idx, value in enumerate(motor_cmd):
# 		pos_error[idx] = np.abs(motor_cmd[idx] - motor_pos[idx])

# 	return all(x < 45 for x in pos_error)

# def pose_message(message, position, orientation):
# 	message.position.x = position[0]
# 	message.position.y = position[1]
# 	message.position.z = position[2]

# 	message.orientation.x = orientation[0]
# 	message.orientation.y = orientation[1]
# 	message.orientation.z = orientation[2]
# 	message.orientation.w = orientation[3]


# def joint_message(message, jointz, jointx):
# 	message.name = []
# 	message.position = []

# 	for i in range(n):
# 		message.name.append("joint" + str(i+1) + "z")
# 		message.name.append("joint" + str(i+1) + "x")
# 		message.position.append(jointz)
# 		message.position.append(jointx)


def follow_trajectory():
	global phi
	global theta
	global motor_pos_first_read
	global pot_pos_first_read
	global pot_pos
	global pot_home
	global motor_cmd
	global motor_pos
	global motor_home
	global motor_cmd_data
	global motor_cmd_pub
	global pos_des
	global pot_volt
	global cmd_reached_data
	global error_pub_data

	error = [0,0,0,0]
	error_int = [0,0,0,0]
	error_der = [0,0,0,0]

	rate = rospy.Rate(rate_HZ) # 10hz

	while not rospy.is_shutdown():


		for idx, value in enumerate(motor_cmd):
			# error_der[idx] = (pot_pos[idx] - pot_home[idx]) - error[idx]
			error[idx] = (pot_pos[idx] - pot_home[idx])
			# error_int[idx] = error_int[idx] + error[idx]
			motor_cmd[idx] = gain_home*error[idx]
			if motor_cmd[idx] > max_vel:
				motor_cmd[idx] = max_vel
			elif motor_cmd[idx] < -max_vel:
				motor_cmd[idx] = -max_vel

			error_pub_data.data[idx] = float(error[idx])
			motor_cmd_data.motors[idx].velocity = motor_cmd[idx]	
			# motor_cmd_data.motors[idx].velocity = max_vel	

		check_position_limits(pot_volt)
		motor_cmd_pub.publish(motor_cmd_data)
		error_pub.publish(error_pub_data)

		if check_pot_pos_error(pot_volt, 0.01):
			break
		rate.sleep()


	motor_home = motor_pos[:]
	print("home_reached")
	print(motor_home)




	while not rospy.is_shutdown():

		pos_des = inverse(l, r, n, theta, phi) # returns the motor positions to achieve the desired theta/phi, in mm

		for idx, value in enumerate(motor_cmd):
			error[idx] = (pos_des[idx] - (motor_pos[idx]- motor_home[idx])*pitch/360)
			motor_cmd[idx] = error[idx]*gain

			if motor_cmd[idx] > max_vel:
				motor_cmd[idx] = max_vel
			elif motor_cmd[idx] < -max_vel:
				motor_cmd[idx] = -max_vel

			error_pub_data.data[idx] = float(error[idx])
			motor_cmd_data.motors[idx].velocity = motor_cmd[idx]

		check_position_limits(pot_volt)

		if(check_position_error(error, 0.5)):
			cmd_reached_data = True
		else:
			cmd_reached_data = False

		motor_cmd_pub.publish(motor_cmd_data)
		cmd_reached_pub.publish(cmd_reached_data)
		error_pub.publish(error_pub_data)
		rate.sleep()



if __name__ == '__main__':
	try:
		rospy.init_node('motor_command_test', anonymous=True)
		rospy.Subscriber('motor_positions', QuadMotorData, motor_pos_cb)
		rospy.Subscriber('trajectory_cmd', TrajMsg, traj_cb)
		rospy.Subscriber('pubpot', potData, pot_cb)
		rospy.on_shutdown(shutdownhook)
		follow_trajectory()
		
	except rospy.ROSInterruptException:
		pass

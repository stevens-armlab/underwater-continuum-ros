#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from teensy_quad_motor_msgs.msg import MotorData, QuadMotorData
# from sensor_msgs.msg import Imu
# from geometry_msgs.msg import Pose
# from inverse_function import inverse, pose
# from qdot_function import qdot

motor_pos = [0,0,0,0]
motor_cmd = [0,0,0,0]
home = [-784.6080, -498.6960, 1001.5280, 571.2960]


def motor_pos_cb(data):
    for i in range(4):
        motor_pos[i] = data.motors[i].position

def check_position_error(motor_cmd):
    pos_error = [0,0,0,0]
    for idx, value in enumerate(motor_cmd):
        pos_error[idx] = np.abs(motor_cmd[idx] - motor_pos[idx])

    if max(pos_error) > 20:
        return False
    else:
        return True

# def pose_message(message, position, orientation):
#     message.position.x = position[0]
#     message.position.y = position[1]
#     message.position.z = position[2]

#     message.orientation.x = orientation[0]
#     message.orientation.y = orientation[1]
#     message.orientation.z = orientation[2]
#     message.orientation.w = orientation[3]

def define_trajectory():
    l = 18
    r = 21
    n = 8
    pitch = 4 # mm/rev
    period = 50
    max_vel = 30 # rpm

    motor_cmd_pub = rospy.Publisher('quad_motor_command', QuadMotorData, queue_size=1)

    # pose_pub = []
    # for i in range(n):
    #     topic_name = 'pose_arm/link' + str(i+1)
    #     pose_pub.append(rospy.Publisher(topic_name, Pose, queue_size=1))

    rospy.init_node('motor_command_test', anonymous=True)
    rospy.Subscriber('motor_positions', QuadMotorData, motor_pos_cb)
    
    rate_HZ = 100
    rate = rospy.Rate(rate_HZ) # 10hz
    start_time = rospy.get_time()
    current_step = 0
    dt = 1.0/rate_HZ

    print("Enter desired motor positions relative to home, in mm. Positive = towards the motors.")
    motor1 = raw_input("Motor 1 adjust (mm):")
    motor2 = raw_input("Motor 2 adjust (mm):")
    motor3 = raw_input("Motor 3 adjust (mm):")
    motor4 = raw_input("Motor 4 adjust (mm):")

    motor_cmd = [float(motor1)/pitch*360 + home[0], float(motor2)/pitch*360 + home[1],\
    float(motor3)/pitch*360 + home[2], float(motor4)/pitch*360 + home[3]]
    count = 0
    while not rospy.is_shutdown():

    	# theta = np.pi/6
    	# phi = np.pi*2/period*current_step
     #    thetadot = 0
     #    phidot = np.pi*2/period

    	# motor_cmd = inverse(l, r, n, theta, phi) # mm
     #    motor_cmd = [x/pitch*360 for x in motor_cmd] # deg

        # cmd_velocity = qdot(l,r,n,theta,phi,thetadot,phidot)

        motor_cmd_data = QuadMotorData()
        if check_position_error(motor_cmd) and count == 0:
        	print("Home: " + str(motor_cmd))
        	count = 1

        # pose_test = pose(l, r, n, theta, phi)
        # pose_data = []

        # for i in range(n):
        #     pose_data.append(Pose())          
        #     pose_message(pose_data[i], pose_test[0][:,i], pose_test[1][:,i])

        # we need to check if the current motor_cmd would be too far to give
        # if not check_position_error(motor_cmd):
        #     current_step = current_step

        # else:
        #     current_step = current_step + dt

        # Since velocity is unused, we can use it to publish measured errors for debugging
        # motor_cmd_data.motors[0].velocity = motor_cmd[0] - motor_pos[0]
        # motor_cmd_data.motors[1].velocity = motor_cmd[1] - motor_pos[1]
        # motor_cmd_data.motors[2].velocity = motor_cmd[2] - motor_pos[2]
        # motor_cmd_data.motors[3].velocity = motor_cmd[3] - motor_pos[3]

        for idx, value in enumerate(motor_cmd):
            motor_cmd_data.motors[idx].position = motor_cmd[idx]
        
        motor_cmd_pub.publish(motor_cmd_data)

        # for i in range(n):
        #     pose_pub[i].publish(pose_data[i])

        rate.sleep()

if __name__ == '__main__':
    try:
        define_trajectory()
    except rospy.ROSInterruptException:
        pass

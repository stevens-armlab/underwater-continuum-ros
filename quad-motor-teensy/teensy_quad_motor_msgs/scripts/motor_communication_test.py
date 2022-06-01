#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from teensy_quad_motor_msgs.msg import MotorData, QuadMotorData

def motor_comm():
    motor_cmd_pub = rospy.Publisher('quad_motor_command', QuadMotorData, queue_size=1)
    rospy.init_node('motor_command_test', anonymous=True)
    rate = rospy.Rate(250) # 10hz
    cmd_position = [0.0, 0.1, 0.2, 0.3]
    while not rospy.is_shutdown():
        motor_command_data = QuadMotorData()
        for idx, motor_position in enumerate(cmd_position):
            motor_command_data.motors[idx].position = cmd_position[idx]
            cmd_position[idx] = cmd_position[idx] + 0.000000001
        motor_cmd_pub.publish(motor_command_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        motor_comm()
    except rospy.ROSInterruptException:
        pass

# Continuum Manipulator Kinematic Control 

This project provides the motor drive method utilizing Teensy and ROS serial communication. There are two parts: the Teensy code and the ROS code. 

In the ROS code, kinematic control is implemented to convert joystick commands into motor commands. IMU readings are converted into joint angles for the RVIZ visualization using the URDF file.

## Setup development environment for ROS messages (Teensy 4.0)
* Create a catkin workspace (see ROS beginner tutorials)
* Copy continuum_kin_control into workspace

## Running Instructions
* Connect both Teensy boards and the joystick controller.
* Run the following in separate tabs. Make sure you source devel/setup.bash each time.
`roscore`  
`roslaunch continuum_kin_control motors.launch`  
`roslaunch continuum_kin_control sensors.launch`  
`rosrun joy joy_node`  
`rosrun continuum_kin_control trajectory_planner.py`  
`rosrun continuum_kin_control velocity_command.py`  
`rosrun continuum_kin_control imu_reader.py`  
`roslaunch continuum_urdf5 display.launch`

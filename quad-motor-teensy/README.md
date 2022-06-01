# Quad-axes motor drive shield with Teensy

This project provides the motor drive method utilizing Teensy and ROS serial communication. There are two parts: the Teensy code and the ROS code. 

In the Teensy code, the motor communication takes care of the current and desired position. ROS nodes are estabilished on both Teensy and linux.

## Install & Software Environment
* Install ROS (Kinetic tested), see [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Install Arduino, see [Arduino Software IDE on linux](https://www.arduino.cc/en/Guide/Linux)
* Install Teensyduino, see [Teensyduino plugin](https://www.pjrc.com/teensy/td_download.html)  
Note 0: always check on Teensyduino for the supported version of Arduino  
Note 1: you need to run the following before you can run the downloaded file  
`chmod +x TeensyduinoInstall.linux64`  
Note 2: when asked to select Arduino folder, it is where you extracted the Arduino, which is likely to be under your Download  
Note 3: you need to also download the rules to run Teensy, see [https://www.pjrc.com/teensy/49-teensy.rules](https://www.pjrc.com/teensy/49-teensy.rules).  
    * Right click link and select "Save Link As".  
    * Run in terminal (must be in folder where you saved the rules): `sudo cp 49-teensy.rules /etc/udev/rules.d/`  
* Verify & Upload code to Teensy   
Tool -> Board -> Teensy 4.0  
Tool -> Port -> `/dev/ttyACM0` or `/dev/ttyS4`  
Verify -> Upload

## A quick motor spin test
 * Download the Dynamixel official repo at [here](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino) and git clone it under Arduino/libraries  
   `cd ~/Arduino/libraries/`  
    `git clone git@github.com:ROBOTIS-GIT/Dynamixel2Arduino.git`
 * Connect a motor for testing on J1, and run the default example `scan_dynamixel`
 * Write down all the information about this motor including: `protocol`, `id`, `baud rate`, and `model number`. 
 * [optional] Use `set_baudrate` to change the baud rate of the motor
 * [optional] Use `set_id` to change the id of the motor
 * Replace all of the information in the position_control test code, and run it

## Setup development environment for ROS messages (Teensy 4.0)
* Install rosserial_arduino. NOTE that the default installation method using command line `sudo apt-get` would fail Teensy 4.0, and the issue is solved by shakeebbb in [issue 456](https://github.com/ros-drivers/rosserial/pull/456)
* Following this thread, we have to clone shakeebbb's teensy_4_0_support branch:  
`cd <ws>/src`  
`git clone https://github.com/shakeebbb/rosserial.git`  
`cd rosserial`  
`git checkout teensy_4_0_support`  
`cd <ws>`  
`catkin_make`  
`catkin_make install`  
* Make sure that the modified rosserial package reside in your workspace/src folder
* see [rosserial Arduino IDE setup](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
* Set up Arduino ros_lib with the custom ros messages generated  
`cd <sketchbook>/libraries`  
`rm -rf ros_lib`  
`source ~/catkin_ws/devel/setup.bash`  
`rosrun rosserial_arduino make_libraries.py .`  
you may double check by `ls ros_lib | grep teensy_quad`, you should see `teensy_quad_motor_msgs`
Run Rosserial test following [rosserial_python](http://wiki.ros.org/rosserial_python):
`roscore`  
`source source devel/setup.bash`  
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`  

## Known Issues
If you cannot use serial monitor in your Arduino IDE, and getting an error of relating to permissions for serial port, try the folloinwg command:
 
 * `sudo chmod a+rw /dev/ttyACM0`  

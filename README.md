# Underwater Continuum Robot Source Files

## Installation Instructions
  
- ***THIS CODE IS FOR ROS MELODIC UBUNTU 18.04 ONLY***  
- Create new catkin workspace
- Run the following to download source files into catkin workspace: `git clone git@bitbucket.org:stevens-robotics/src.git`
- Go to the top level of workspace and run the following to install required packages: `rosdep install --from-paths src --ignore-src -r -y`
- Run the following to build package: `catkin_make install`

## Robot Control
Remember to source your catkin workspace for each unique terminal.  
Note: `motors.launch` and `sensor.launch` correspond to launching a node for the Teensy on port `/dev/ttyACM0` and `/dev/ttyACM1` respectively. 
This may not correspond with the actual motor and sensor boards depending on the order in which they were connected. 
  
  
To run simple text-based trajectory commands:  

- `roscore`  
- `roslaunch continuum_kin_control motors.launch`  
- `roslaunch continuum_kin_control sensor.launch`  
- `rosrun continuum_kin_control velocity_command.py`  
- `rosrun continuum_kin_control trajectory_input.py`  

To get IMU visualization:  
  
- `roscore`  
- `roslaunch continuum_kin_control sensor.launch`  
- `rosrun continuum_kin_control imu_reader.py` (must be in top level of catkin workspace)  
- `roslaunch continuum_urdf5 display.launch` 

To run joystick control:  

- `roscore`  
- `roslaunch continuum_kin_control motors.launch`  
- `roslaunch continuum_kin_control sensor.launch`  
- `rosrun continuum_kin_control velocity_command.py`  
- `rosrun continuum_kin_control trajectory_planner.py`  
- `rosrun joy joy_node` 


## Communicating Remotely via Tether  

- The Jetson Nano on the robot should have this package installed.  
- Connect Jetson to Fathom-X board in robot via short ethernet cable.  
- Connect a TSP cable from the tether to Fathom-X.  
- Plug tether into Fathom-X Topside Interface.  
- Plug USB of interface into topside computer.  
- Set Static IP address to be 192.168.1.XX (anything except .54), and the netmask to 255.255.255.0  
- Test connection by running `ping 192.168.1.54`  
- Use SSH to remote log into Jetson via custom IP address. For us, that is `ssh justin@192.168.1.54`  
- Launch roscore and Teensy nodes on Jetson to prevent excessive latency.  
- IMU visualization and joystick nodes can be run on topside computer, but requires exporting ROS IP and URI data:  
- `export ROS_IP=192.168.1.54`  
- `export ROS_MASTER_URI=http://192.168.1.54:11311/`
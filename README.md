# Texas Aerial Robotics ROS package 

In this repository are the files nessary to build the ROS (Robot Operating System) package. 

We use Ubuntu 16.04 and ROS Kinetic 

ROS can be installed following these instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu 

This package is loaded and run on our Nvidia Jetson TX1 onboard the quadcopter. 
It then feeds commands using MAVROS. 

# Clone instructions 

Clone repository into a catkin workspace (typically a folder src called catkin_ws) 
* Becomes `catkin_ws/src/Controls-ROS/`

# Dependancies installation 

Install `mavros` from source using https://dev.px4.io/en/ros/mavros_installation.html#source-installation 
* Make sure to change commands that say `indigo` to `kinetic`

# Build instructions 

Inside catkin_ws, run `catkin_make`


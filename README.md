# Texas Aerial Robotics ROS package 

In this repository are the files nessary to build the ROS (Robot Operating System) package. 

We use Ubuntu 16.04 and ROS Kinetic 

ROS can be installed following these instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu 

This package is loaded and run on our Nvidia Jetson TX1 onboard the quadcopter. 

It then feeds commands to the Pixhawk 2 using MAVROS. 

# Clone instructions 

Clone repository into a catkin workspace (typically a folder `src` called `catkin_ws`). To set up catkin follow this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
1. In `catkin_ws/src`, run `git clone https://github.com/Texas-Aerial-Robotics/Controls-ROS.git`
2. Becomes `catkin_ws/src/Controls-ROS/`

# Dependencies installation 

Install `mavros` from source using https://dev.px4.io/en/ros/mavros_installation.html#source-installation 
* Make sure to change commands that say `indigo` to `kinetic`

# Build instructions 

Inside `catkin_ws`, run `catkin build`

# Gazebo installation with ardupilot 

First clone ardupilot:

Follow this [GitHub post](https://github.com/ArduPilot/ardupilot_wiki/issues/1001) to install gazebo with ardupilot support

Follow this [tutorial](https://github.com/AS4SR/general_info/wiki/ArduPilot:-Instructions-to-set-up-and-run-an-autopilot-using-SITL-and-Gazebo-simulator) to clone ardupilot and get the drone running

# Better Gazebo Installation Instructions

## Download ArduPilot

In home directory:  

~~~
git clone git://github.com/ArduPilot/ardupilot.git
cd ardupilot  
git submodule update --init --recursive
~~~

Install some packages

~~~
sudo apt-get install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml  
sudo apt-get install python-scipy python-opencv ccache gawk git python-pip python-pexpect  
sudo pip2 install future pymavlink MAVProxy 
~~~

Open ~/.bashrc:  

`gedit ~/.bashrc`  

Add these lines to .bashrc:  
~~~
export PATH=$PATH:$HOME/ardupilot/Tools/autotest  
export PATH=/usr/lib/ccache:$PATH
~~~ 

Reload ~/.bashrc:  
`. ~/.bashrc`  

Run SITL once to set params:
~~~
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
~~~

## Install Gazebo

Install Gazebo:  

`curl -ssL http://get.gazebosim.org | sh`  

Get plugin for APM:

~~~
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
~~~

Set paths for Models
~~~
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/gazebo_models' >> ~/.bashrc
. .bashrc
~~~

Add Gazebo Models

~~~

hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
cd ~/gazebo_ws/gazebo_models
hg checkout zephyr_demos
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models' >> ~/.bashrc
source ~/.bashrc
~~~

# To Run Sim

Run SITL:
`sim_vehicle.py -j4 -f Gazebo`  

Run Gazebo:
`gazebo --verbose ~/ardupilot_gazebo/gazebo_models/iris_irlock_demo.world`  

ROS connection string is `udp://127.0.0.1:14551@14555`

Set parameters for sim in the same window after you run the sim_vehicle.py script. Do this my using command `param load <filename>`  
Example params can be found in the Controls-Other repo.

# Texas Aerial Robotics ROS package 

In this repository are the files nessary to build our ROS (Robot Operating System) package. 

We use **Ubuntu 16.04** and **ROS Kinetic** 

This package is loaded and run on our Nvidia Jetson TX1 onboard the quadcopter. 

It then feeds commands to the Pixhawk 2 using MAVROS. 

## 1. Install ROS 

   First, install **ROS Kinetic** using the following instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu 

   Do _Desktop Install_  
   Make sure to follow all the way through the installation (until _Step 1.7_ at the end of the page)

## 2. Set Up Catkin workspace 

We use catkin build instead of catkin_make. Please install the following: 
```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace: 
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

## 3. Dependencies installation 

Install `mavros` and `mavlink` from source: 
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```

## 4. Clone our ROS package repository 

```
cd ~/catkin_ws/src 
git clone https://github.com/Texas-Aerial-Robotics/Controls-ROS.git
```
Our repository should now be copied to `~/catkin_ws/src/Controls-ROS/`

## 5. Build instructions 
   Inside `catkin_ws`, run `catkin build`

```
cd ~/catkin_ws 
catkin build 
```

## 6. Install Ardupilot

In home directory: 
```
cd ~
git clone git@github.com:Texas-Aerial-Robotics/ardupilot.git
cd ardupilot  
git submodule update --init --recursive
```

Install some packages: 
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml  
sudo apt install python-scipy python-opencv ccache gawk git python-pip python-pexpect  
sudo pip2 install future pymavlink MAVProxy 
```

Open `~/.bashrc`:  
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc`:  
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest  
export PATH=/usr/lib/ccache:$PATH
``` 

Reload `~/.bashrc`:  
```
. ~/.bashrc
```  

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

## 7. Install Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
```

Reload software list: 
```
sudo apt-get update
```

Install Gazebo: 
```
sudo apt-get install gazebo7 libgazebo7-dev
```

Install ROS plugins: 
```
sudo apt install ros-kinetic-gazebo-ros ros-kinetic-gazebo-plugins
```


Get Gazebo plugin for APM (ArduPilot Master):
```
cd ~
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```

Set paths for models: 
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/gazebo_models' >> ~/.bashrc
. ~/.bashrc
```

Add Gazebo Models: 
```
hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
cd ~/gazebo_ws/gazebo_models
hg checkout zephyr_demos
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

## 8. Run Simulator 

In one terminal, run SITL:
```
sim_vehicle.py -j4 -f Gazebo --console 
```  

In another terminal, run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/gazebo_worlds/iris_irlock_demo.world
```  
ROS connection string is `udp://127.0.0.1:14551@14555`

Set parameters for sim in the same window after you run the `sim_vehicle.py script`. 
Do this by using command `param load <filename>`  
Example params can be found in the `Controls-Other` repo: https://github.com/Texas-Aerial-Robotics/Controls-Other 


## 9. Install Roomba Drone Simulator 

In order to use the ardupilot simulator with the roombas as well as other plugins such as cameras you need to run gazebo with ros. Install the following dependencies to get the simulator to run

```
 sudo apt install ros-kinetic-gazebo-ros ros-kinetic-gazebo-plugins
```
clone in Gazebo-Ros in your catkin_ws and the Computations repo in your home directory 

```
cd ~/catkin_ws/src
git clone https://github.com/Texas-Aerial-Robotics/Gazebo-Ros.git
cd ~
git clone https://github.com/Texas-Aerial-Robotics/Computations.git
```

Add the following to your bashrc to use the roomba plugins

```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/Computations/roomba_host/models
```

```
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Computations/roomba_host/build
```
### Run a sim with roslaunch 
```
roslaunch gazebo_ros_sim gazebo.launch
```
in another terminal run the following command to start and connect the ardupilot sim to gazebo

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris --console -I0
```

### To view camera plugin 
install image view 

```
sudo apt install ros-kinetic-image-view
```

To view camera footage 
```
rosrun image_view image_view image:=/webcam/image_raw
```


## 10. Run Mavros Scripts from Controls-Ros package 

Clone the Controls-Other repo to access our params and apm launch file
```
git clone https://github.com/Texas-Aerial-Robotics/Controls-Other
```
Make a launch directory in your Ardupilot directory and copy the apm.launch file there

```
cd ardupilot/
mkdir launch
cp ~/Controls-Other/Launch/apm.launch ~/ardupilot/launch
```
launch the sim via roslaunch 

```
roslaunch gazebo_ros_sim gazebo.launch
```
in another terminal run the following command to start and connect the ardupilot sim to gazebo

```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris --console -I0
```
in another terminal run
```
cd ~/ardupilot/launch
roslaunch apm.launch
```
Be sure to wait for the drone to go throught the full initialization process. The drone will be ready to fly once the console reads Optical Flow now aiding and origin set to GPS. For the competition we will use mission planner to home the drone, but for introdution sake let the drone use GPS

in another terminal run
```
rosrun flight_pkg staple
```


--- 

## Troubleshooting and Common Problems: 
### During Catkin Build: 
#### Complains about "GeographicLib" 
```
cd ~/catkin_ws/src/mavros/mavros/scripts 
sudo ./install_geographiclib_datasets.sh 
``` 

#### Complains about "Geometry Msgs" 
```
sudo apt install ros-kinetic-geometry-msgs 
``` 

--- 

References: 
- https://dev.px4.io/en/ros/mavros_installation.html#source-installation 
- http://wiki.ros.org/catkin/Tutorials/create_a_workspace  
- https://github.com/ArduPilot/ardupilot_wiki/issues/1001 
- https://github.com/AS4SR/general_info/wiki/ArduPilot:-Instructions-to-set-up-and-run-an-autopilot-using-SITL-and-Gazebo-simulator

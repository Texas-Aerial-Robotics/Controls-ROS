# Texas Aerial Robotics ROS package

In this repository are the files nessary to build our ROS (Robot Operating System) package.

We use **Ubuntu 18.04** and **ROS Melodic**

This package is loaded and runs on our Nvidia Jetson TX2 onboard the quadcopter.

It then feeds commands to the Pixhawk 2 using MAVROS.

Note that there is a **Troubleshooting** section at the bottom of this page.

Code blocks are meant to be typed in Terminal windows. "Control+Alt+T" opens a new Terminal window.

## 1. Install ROS

   - Do _Desktop Install_
   - Follow until _Step 1.7_ at the end of the page

   First, install **ROS Melodic** using the following instructions: http://wiki.ros.org/melodic/Installation/Ubuntu


## 2. Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:
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

Open your `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add this line to end of `~/.bashrc`:
```
source ~/catkin_ws/devel/setup.bash
```
Save and exit. Then run:
```
source ~/.bashrc
```

## 4. Clone our ROS package repository

```
cd ~/catkin_ws/src
git clone https://github.com/Texas-Aerial-Robotics/Controls-ROS.git
```
Our repository should now be copied to `~/catkin_ws/src/Controls-ROS/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

## 5. Build instructions
   Inside `catkin_ws`, run `catkin build`:

```
cd ~/catkin_ws
catkin build
```

## 6. Install Ardupilot

In home directory:
```
cd ~
git clone https://github.com/Texas-Aerial-Robotics/ardupilot.git
cd ardupilot
git checkout Copter-3.5
git submodule update --init --recursive
```

Install some packages:
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk git python-pip python-pexpect
sudo pip2 install future pymavlink MAVProxy
```

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

Once the new Terminal window pops up and you see the ArduPilot software say "READY TO FLY," you can kill the process by hitting "Control+C" in the Terminal it is running out of.

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
sudo apt update
```

Install Gazebo:
```
sudo apt install gazebo9 libgazebo9-dev
```

Install ROS plugins:
```
sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
```

Get Gazebo plugin for APM (ArduPilot Master):
```
cd ~
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
git checkout gazebo9
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
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

## 8. Run Simulator

In one Terminal (Terminal 1), run SITL:
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -j4 -f Gazebo --console
```

In another Terminal (Terminal 2), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/gazebo_worlds/iris_irlock_demo.world
```

Set parameters for sim in the same window after you run the `sim_vehicle.py script`.
Do this by using command `param load <filename>`
Example params can be found in the `Controls-Other` repo: https://github.com/Texas-Aerial-Robotics/Controls-Other

To load our parameters, clone our `Controls-Other` repository (Terminal 3):
```
cd ~
git clone https://github.com/Texas-Aerial-Robotics/Controls-Other
```
In the Ardupilot simulator Terminal (Terminal 1) run:
```
param load ../../Controls-Other/Params/Sim_master.param
```
Play around, then close by "Control+C" in the Terminal windows.

## 9. Install Roomba Drone Simulator

In order to use the Ardupilot simulator with the Roombas - as well as other plugins such as cameras - you need to add ROS packages to Gazebo. 
Install the following dependencies:
```
 sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins
```

Clone in Gazebo-Ros in your catkin_ws and the Computations repo in your home directory:
```
cd ~/catkin_ws/src
git clone https://github.com/Texas-Aerial-Robotics/Gazebo-Ros.git
cd ~
git clone https://github.com/Texas-Aerial-Robotics/Computations.git
```

Add the following to your `~/.bashrc` to use the Roomba plugins:
```
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/Computations/roomba_host/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Computations/roomba_host/build
```

Build the ROS packages you just downloaded:
```
cd ~/catkin_ws/
catkin build
```

Reload `~/.bashrc` so the Terminal knows where the packages are located:
```
source ~/.bashrc
```

### Run a sim with roslaunch
In one Terminal (Terminal 1):
```
roslaunch gazebo_ros_sim droneOnly.launch
```
In another Terminal (Terminal 2), run the following command to start and connect the Ardupilot simulator to Gazebo:
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris --console -I0
```
Play around, then close by "Control+C" in the Terminal windows.

### To view camera plugin
Install image view:
```
sudo apt install ros-melodic-image-view
```

To see camera view:
```
rosrun image_view image_view image:=/webcam/image_raw
```

## 10. Run Mavros Scripts from Controls-Ros package

Launch the sim via roslaunch (Terminal 1):
```
roslaunch gazebo_ros_sim droneOnly.launch
```
In another Terminal (Terminal 2) run the following command to start and connect the Ardupilot sim to Gazebo:
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris --console -I0
```
In another Terminal (Terminal 3) run:
```
cd ~/Controls-Other/Launch/
roslaunch apm.launch
```
Be sure to wait for the drone to go throught the full initialization process. The drone will be ready to fly once the console reads Optical Flow now aiding and origin set to GPS. For the competition we will not use GPS to home the drone, but for introduction sake let the drone use GPS

In another Terminal (Terminal 4) run:
```
rosrun flight_pkg staple
```

Then in MavProxy (Terminal 2) switch the mode into `guided`:
```
mode GUIDED
```

you did it! 🎉

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
sudo apt install ros-melodic-geometry-msgs
```

---

References:
- https://dev.px4.io/en/ros/mavros_installation.html#source-installation
- http://wiki.ros.org/catkin/Tutorials/create_a_workspace
- https://github.com/ArduPilot/ardupilot_wiki/issues/1001
- https://github.com/AS4SR/general_info/wiki/ArduPilot:-Instructions-to-set-up-and-run-an-autopilot-using-SITL-and-Gazebo-simulator

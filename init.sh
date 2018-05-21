sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update
sudo apt install ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-wstool python-rosinstall-generator python-catkin-tools
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
cd ~/catkin_ws
wstool init ~/catkin_ws/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y
catkin build
echo "source ~/catkin_ws/devel/setup.bash " >> ~/.bashrc
source ~/.bashrc
cd ~/catkin_ws/src 
git clone https://github.com/Texas-Aerial-Robotics/Controls-ROS.git
cd ~/catkin_ws
catkin build
cd ~
git clone https://github.com/Texas-Aerial-Robotics/ardupilot.git
cd ardupilot  
git submodule update --init --recursive
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml  
sudo apt install python-scipy python-opencv ccache gawk git python-pip python-pexpect  
sudo pip2 install future pymavlink MAVProxy 
echo "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.bashrc
echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.bashrc
source ~/.bashrc
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w &
sleep 20 
kill %1
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
sudo apt-get update
sudo apt-get install gazebo7 libgazebo7-dev
sudo apt install ros-kinetic-gazebo-ros ros-kinetic-gazebo-plugins
cd ~
git clone https://github.com/SwiftGust/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/gazebo_models' >> ~/.bashrc
source ~/.bashrc
hg clone https://bitbucket.org/osrf/gazebo_models ~/gazebo_ws/gazebo_models
cd ~/gazebo_ws/gazebo_models
hg checkout zephyr_demos
echo 'export GAZEBO_MODEL_PATH=~/gazebo_ws/gazebo_models' >> ~/.bashrc
source ~/.bashrc
cd ~
git clone https://github.com/Texas-Aerial-Robotics/Controls-Other 

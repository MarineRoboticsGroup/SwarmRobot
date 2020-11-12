#!/bin/bash

# Shell script to install all required packages for RPLiDAR NUC PC
# Make script executable (chmod +x nuc_install.sh) then run ./nuc_install.sh


# Enable repositories
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse

# Set up ROS Melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool python3-vcstool build-essential
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update

# Downloading all ROS packages via apt package manager
PKG_PREFIX="ros-melodic"
PKGS="navigation nav-msgs trajectory-msgs geometry-msgs sensor-msgs rqt-tf-tree rosserial-python std-msgs joy"
sudo apt update -y
for pkg in $PKGS
do
  if sudo apt install -y $PKG_PREFIX-$pkg
  then
    echo "Retrieved packages successfully."
  else
     echo "Unable to retrieve packages. Check that repository is reachable."
     echo "Exiting..."
     return
  fi
done

source ~/.bashrc
# Setting up catkin workspace
CATKIN_HOME=$HOME/catkin_ws
CATKIN_SRC_DIR=$CATKIN_HOME/src/
if [ ! -d $CATKIN_SRC_DIR ]; then
    mkdir -p $CATKIN_SRC_DIR
fi
cd $CATKIN_SRC_DIR

# clone rplidar repo into catkin workspace
RPLIDAR_REPO="https://github.com/robopeak/rplidar_ros.git"
git clone $RPLIDAR_REPO

# set up all ROS dynamixel repos
DYNAMIXEL_REPO="https://github.com/ROBOTIS-GIT/dynamixel-workbench.git"
git clone $DYNAMIXEL_REPO
cd $CATKIN_HOME
sudo apt install -y python3-vcstool
vcs import src < $CATKIN_SRC_DIR/dynamixel-workbench/.dynamixel_workbench.rosinstall
vcs pull src

# Set symbolic link to U2D2 for dynamixels so can always be found
# (https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
sudo cp ~/SwarmRobot/install/99-dynamixel-workbench-cdc.rules /etc/udev/rules.d/
sudo cp ~/SwarmRobot/install/88-rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm control --reload
sudo udevadm trigger

# Sourcing and building workspace
sudo adduser $USER dialout
sudo chmod 666 /dev/ttyUSB0

# build catkin workspace
sudo apt install -y python-catkin-tools
cp -r ~/SwarmRobot $CATKIN_SRC_DIR/swarm-robot
source ~/.bashrc
catkin build
source ~/.bashrc

# Set current catkin workspace to default (means should only have one workspace on computer)
echo "source $CATKIN_HOME/devel/setup.bash" >> ~/.bashrc
echo "export ROS_IP=<fill in with wlan IP address>" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://128.30.10.130:11311" >> ~/.bashrc
source ~/.bashrc

# so we can ssh into this machine
sudo apt install -y openssh-server

# install terminator
sudo add-apt-repository ppa:gnome-terminator
sudo apt update -y
sudo apt install -y terminator

# Test downloads through RPLiDAR with roslaunch rplidar_ros view_rplidar.launch
# If launch file isn't working, confirm that packages/programs are sourced and
# in the src folder


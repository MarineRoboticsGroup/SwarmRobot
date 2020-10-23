#!/bin/bash

# Shell script to install all required packages for RPLiDAR NUC PC
# Make script executable (chmod +x nuc_install.sh) then run ./nuc_install.sh

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

# Sourcing and building workspace
sudo adduser $USER dialout
sudo chmod 666 /dev/ttyUSB0

sudo apt install -y python-catkin-tools
cp -r ~/SwarmRobot $CATKIN_SRC_DIR/swarm-robot
catkin build

# Set current catkin workspace to default (means should only have one workspace on computer)
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source $CATKIN_HOME/devel/setup.bash" >> ~/.bashrc
echo "export ROS_IP=10.42.0.1" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://10.42.0.1:11311" >> ~/.bashrc
source ~/.bashrc

# Test downloads through RPLiDAR with roslaunch rplidar_ros view_rplidar.launch
# If launch file isn't working, confirm that packages/programs are sourced and
# in the src folder

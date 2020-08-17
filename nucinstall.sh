#!/bin/bash

# Shell script to install all required packages for RPLiDAR NUC PC
# Make script executable (chmod +x nuc_install.sh) then run ./nuc_install.sh

INSTALL_ROOT=$HOME/catkin_ws
INSTALL_SRC=$INSTALL_ROOT/src/
PKG_PREFIX="ros-melodic"

PKGS="dynamixel-workbench navigation nav-msgs geometry-msgs sensor-msgs rqt-tf-tree rosserial-python std-msgs"

GITS="https://github.com/robopeak/rplidar_ros.git"

# Configure catkin workspace 
if [ ! -d $INSTALL_SRC ]; then
    mkdir -p $INSTALL_SRC
fi
cd $INSTALL_SRC
# Downloading all listed packages; can update list 
sudo apt-get update -y
for pkg in $PKGS
do
  if sudo apt-get install -y $PKG_PREFIX-$pkg
  then
    echo "Retrieved packages successfully."
  else
     echo "Unable to retrieve packages. Check that repository is reachable."
     echo "Exiting..."
     return
  fi
done

# Cloning and downloading repos
for git in $GITS
do
  git clone $git
done

# Sourcing and building workspace
sudo adduser $USER dialout
sudo chmod 666 /dev/ttyUSB0
cd $INSTALL_ROOT
catkin_make
echo "source $INSTALL_ROOT/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
export ROS_IP=10.42.0.1

# Test downloads through RPLiDAR with roslaunch rplidar_ros view_rplidar.launch
# If launch file isn't working, confirm that packages/programs are sourced and
# in the src folder

#!/bin/bash

# Shell script to install ROS on the NUC
# Taken from ROS wiki instructions on downloading Melodic

# Enable repositories
sudo add-apt-repository restricted
sudo add-apt-repository universe
sudo add-apt-repository multiverse

# Set-up sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" $

# Set-up keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6$

# Update and install ROS (desktop-full)
sudo apt update
sudo apt install ros-melodic-desktop-full

# Dependencies
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator pytho$
sudo apt install python-rosdep
sudo rosdep init
rosdep update

# Config environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

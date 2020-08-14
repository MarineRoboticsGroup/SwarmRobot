#!/bin/bash

# Shell script to install ROS on Jetson Nano
# Taken from JetsonHacks repo at https://github.com/JetsonHacksNano/installROS

# Clone repository
git clone https://github.com/JetsonHacksNano/installROS.git
cd installROS
./installROS.sh -p ros-melodic-desktop-full

# Set-up workspace
./setupCatkinWorkspace.sh catkin_ws

# Configure network
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.25

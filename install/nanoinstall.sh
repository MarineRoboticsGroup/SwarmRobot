#!/bin/bash

# Shell script to install all required packages for RealSense Nano pc
# Make script executable (chmod +x nano_install.sh) then run ./nano_install.sh
# Credit to JetsonHacks for their Nano-specific downloads!

cd ~
git clone https://github.com/JetsonHacksNano/installROS.git
cd installROS
./installROS.sh -p ros-melodic-desktop-full
./setupCatkinWorkspace.sh catkin_ws

CATKIN_HOME=$HOME/catkin_ws
CATKIN_SRC_DIR=$CATKIN_HOME/src/
ROS_VERSION="melodic"

# Check that ROS is installed
if [ rosversion -d is false ] || [ rosversion -d != "$ROS_VERSION" ] ; then
   echo "ROS ($ROS_VERSION) not found. Make sure it's installed on the server."
fi

# Software can be installed already built, configured for the Nano from JetsonHacks
git clone https://github.com/JetsonHacksNano/installLibrealsense
cd installLibrealsense
./installLibrealsense.sh
sudo apt-get install -y ros-melodic-realsense2-camera

# Test downloads through RealSense with roslaunch realsense2_camera rs_camera.launch
# If launch file isn't working, check source of launch file with rospack
# and source again if necessary

echo "source $CATKIN_HOME/devel/setup.bash" >> ~/.bashrc
echo "export ROS_IP=10.42.0.25" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://128.30.10.130:11311" >> ~/.bashrc
source ~/.bashrc

#so we can ssh into this machine
sudo apt install -y openssh-server

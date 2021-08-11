# Setting Up Complete RTABMap System on the SwarmRobots
---
### Our current system operates with a minimal RTABMap set-up, as we found the binary to be most robust for our data collection and analysis needs. While RTABMap can be built from source, we encountered several dependency conflicts that didn’t successfully lead to data collection and visualization output. 

### Through the rtabmap-ros binary, we established a working odometry collection pipeline and data collection/visualization set-up that can be reproduced reliably through the steps below. These steps include methods from several sources and write-ups, which are linked below for additional information.
---
1. Delete previous software for the Intel RealSense d435i, including the ROS binary for the realsense and its corresponding SDK.
    1. sudo apt-get remove ros-melodic-realsense2-camera
    2. Go through the operating ROS workspace and rm -rf any realsense-related directories eg. realsense-ros, librealsense, librealsense2, to minimize version conflicts and repeats (these will be rebuilt at a later point)
2. For a clean, dependable build, it was found necessary to purge the current ROS set-up to ensure fully-maintained dependencies and software versions. This was done by purging all related ROS software and rebuilding completely.
    1. sudo apt-get purge ros-*
    2. sudo apt-get purge python-ros*
    3. sudo apt-get autoremove
3. Now that all previous ROS software has been removed, it can be reinstalled as usual
    1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    2. curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    3. sudo apt update 
    4. sudo apt upgrade
    5. sudo apt install ros-melodic-desktop-full
    6. echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    7. source ~/.bashrc
    8. sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    9. sudo apt install python-rosdep
    10. sudo rosdep init
    11. rosdep update
4. Following the minimal install and set-up required to run SLAM via RTABMap on an Intel realsense D435i, there are several binaries to install before getting the system running
    1. sudo apt-get install ros-melodic-realsense2-camera
    2. sudo apt-get install ros-melodic-imu-filter-madgwick
    3. sudo apt-get install ros-melodic-rtabmap-ros
    4. sudo apt-get install ros-melodic-robot-localization
    5. sudo apt-get install ros-melodic-octomap-rviz-plugins
5. Realsense-ros also requires the librealsense SDK for camera operation and streaming settings
    1. cd ~/catkin_ws/src
    2. sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
    3. git clone https://github.com/IntelRealSense/librealsense.git
    4. cmake .
    5. cd ../..
    6. catkin build
6. Finally, the main RealSense launch file must be configured for odometry data, as it combines accelerometer and gyroscope readings to output odometry
    1. cd /opt/ros/melodic/share/realsense2_camera/launch
    2. code . (call for VSCode)
    3. Within rs_camera.launch, change enable_gyro, enable_accel, enable_infra, enable_infra1, and enable_infra2 to “true”
    4. cd 
    5. source ~/.bashrc
7. RTABMap is now fully set-up with the minimal binaries required for odometry output and RGB-depth mapping
    1. The collection and analysis to follow have been documented in Odom Data Collection/Conversion/Analysis Documentation
    2. It’s worth noting that these steps should lead to a working data collection and analysis setup from the host pc (ie. the robot’s onboard computer). However, several issues were encountered when implementing a multi-robot program manager such as Procman-ROS.	
        1. These issues were resolved by ssh-ing into the robot’s computer to run RTABMap remotely, which could still be picked up when recording a ROSBAG on the master computer.

Additional Documentation

Debugging: https://answers.ros.org/question/361445/rtabmap-tutorial-odometry-died-on-startup-noetic-xtion-u2004-beginner/ 

RTABMap set-up: https://github.com/IntelRealSense/realsense-ros/wiki/SLAM-with-D435i

# Common Bugs and Solutions for the SwarmBots

## UWB or LiDAR USB Conflicts:
---
### Error Message: 
ERROR *** UART: Unable to open UART
Ensure it is not in use by another application
Not initialized. 
Received 0 bytes, expected 7 bytes, timed out in 1000 ms

### Solution:
The symlinks created for the UWB and LiDAR aren’t initialized, so the initial problem of dynamic USB-ID naming has incorrectly assigned the two devices
Refer to the **Guide to Configuring Indistinct USB/UART Conflicts** documentation

## RealSense Setting Configurations:
---
### Error Message:
No odom data found
No RGB data found

### Solution:
The main RealSense launch file, rs_camera.launch, hasn’t been configured with correct gyro/accel and infra1/infra2 settings
Refer to the **Setting Up Complete RTABMap System on the SwarmRobots** documentation

## Robot Roslaunch Frozen Commands:
---
### Error Message:
Terminal freezing upon roslaunch command
Output only when quitting via “Ctrl-C”

### Solution:
The ROS_IP and ROS_MASTER_URI likely haven’t been set to the IP address of the current wifi network
Using ifconfig and sudo nano ~/.bashrc, configure the respective IP settings and source the changes using source ~/.bashrc or simply closing and reopening the terminal

## Running Roslaunch and Python Library Not Found:
---
### Error Message:
<Python library name> not found

### Solution:
Anaconda and its respective python versions and shells conflict with certain programs and roslaunch commands
Quit the terminal Anaconda session with conda deactivate

## No Odometry Data Available:
---
### Error Message:
E: Did not receive data since 5 seconds! 

### Solution:
When analyzing a rosbag of recorded RTABMap data, there may appear to be zero odometry readings collected
RTABMap occasionally loses odometry in the data collection process, as its RGB-depth pointcloud projection can be broken if the camera is moved too quickly or sharply
This situation can be confirmed by playing the rosbag back and showing the collected odometry measurements through rostopic echo /rtabmap/odom
If the messages are all zero values, then redo the rosbag recording with more attention to the recording methods

## Unable to Download Certain Packages (Ubuntu):
---
### Error Message:
E: ubuntu apt-get unable to fetch some archives 404
E: Unable to fetch some archives, maybe run apt-get update or try with --fix-missing?

### Solution:
Some packages from certain repositories are blocked from installing on the computers, depending on the initial settings of the ubuntu instance
This can often be solved by disabling Secure Boot within BIOS Boot Configuration settings
If the above step doesn’t solve it, make sure the sources.list file in /etc/apt directory is updated to a current file version

## Zero-Value Odometry to Plaza Format Data:
---
### Error Message:
ValueError: Found zero norm quaternions in `quat`.

### Solution:
This error comes up in the post-processing stage, where when converting from rtabmap/odometry format to plaza format, the quaternion transform is unable to work with zero-valued data
There’s a method written into the format conversion program that should skip these values, which still maintains the majority of the data due to minimal zero values (eg. 1-2 zeros in a medium dataset)

## Unable to Install pyGTK:
---
### Error Message:
E: Unable to locate package pygtk
pyGTK stuck on dependencies that are unable to be resolved
	
### Solution:
sudo apt install python-gtk2 python-gtk2-dev
Reference (https://itsfoss.community/t/solved-pygtk-installation/2781)

## Unable to Install LCM:
---
### Error Message:
Unable to build LCM from source

### Solution:
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j2
sudo make install
Make sure all required dependencies are installed and built (build-essential, libglib2.0-dev, cmake)
echo $LCM_LIBRARY_DIR > /etc/ld.so.conf.d/lcm.conf
Reference (https://lcm-proj.github.io/build_instructions.html)

## Realsense Data Overflow:
---
### Error Message:
Hardware Notification: 1.55068e+12, Error, Unknown Error

### Solution:
This error usually occurs when the USB-C to USB-A cable is rated 2.0 as opposed to the required 3.0. 
By switching out the cable for a 3.0 lightning cable, the overflow is contained 
Verify that the USB port is also powered, and that the RealSense is detected with rs-enumerate-devices
If the camera is not found, unplug and replug the camera. realsense-viewer is also built into the SDK for debugging and verification purposes

## Python Version Conflicts:
---
### Error Message:
ImportError: dynamic module does not define init function (PyInit__tf2)

### Solution:
If there are multiple versions of python running in the same environment, conflicts between version-based packages arise when running certain programs
Python3 shouldn’t be running through ROS as ROS2 goes along with Python3
Anaconda is also built with Python3, so it’s important to be aware of the current version as well as build of involved software


## RealSense Software Missing:
---
### Error Message: 
(synthetic-stream.cpp:48) Exception was thrown during user processing callback: Out of frame resources!

### Solution:
This error can come up if RealSense software hasn’t been properly installed. Whether the binary or SDK haven’t been installed or built correctly, or certain packages and dependencies are broken, this situation is often fixed by complete reinstall of all RealSense software
Refer to the **RTABMap Set-up** documentation on setting up all required RealSense software successfully

## RTABMap Process Dies After Start-Up:
---
### Error Message:
Process has died [pid 6785, exit code -6, cmd /opt/ros/noetic/lib/rtabmap_ros/rgbd_odometry rgb/image:=/camera/rgb/image_rect_color depth/image:=/camera/depth_registered/image_raw rgb/camera_info:=/camera/rgb/camera_info rgbd_image:=rgbd_image_relay odom:=/odom imu:=/imu/data __name:=rgbd_odometry __log:=/home/aug/.ros/log/d90a0fe6-f490-11ea-a0cb-6b5fc5d13082/rtabmap-rgbd_odometry-1.log]. log file: /home/aug/.ros/log/d90a0fe6-f490-11ea-a0cb-6b5fc5d13082/rtabmap-rgbd_odometry-1*.log

### Solution:
While this error can come up for several different reasons, one of the Swarm Robots encountered exit code -6 because of a set-up error resulting from multiple versions of OpenCV
Both OpenCV 4.5 and OpenCV 3.2 were running in the same Ubuntu environment, causing RTABMap to die after the launch file was called through roslaunch
Running the command “find / 2>/dev/null | grep libopencv | egrep "4.5|3.2" | grep ".so" | more” can clarify if two versions of OpenCV are floating around 
If this is the case, remove the unnecessary system (which can be determined depending on the build versions of reliant software)

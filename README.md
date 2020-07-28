# RoboSwarm

Background
--------------

Testing and implementing developed SLAM algorithms is difficult due to the accessibility of fully-formed robotic swarms, and the available swarms are often expensive or ineffective. The goal of our project is to design and build a robot that can communicate via ultra wideband and scale up to a larger swarm, with the capability of assessing SLAM algorithms and swarm experimentation. 

Table of Contents
--------------------

- [Resources](#resources)
  * [Dynamixel Instructions](#dynamixel-workbench)
  * [Intel RealSense Instructions](#ros-wrapper-for-intel-realsense-devices)
  * [RPLiDAR Instructions](#rplidar-ros-node-and-test-application)
- [Hardware Setup](#hardware-setup)
  * [Connection Diagram](#connection-diagram)
  * [Multi-Robot Swarm Connections Diagram](#multi-robot-swarm-connections-diagram)
- [Software Setup](#software-setup)
  * [ROS System Architecture Diagram](#ros-system-architecture-diagram)
- [Design Principles](#design-principles)
  * [CAD Design on SolidWorks 2018](#cad-design-on-solidworks-2018)
- [Getting Started](#getting-started)
  * [Setting Up](#setting-up)
  * [Running](#setting-up)
  * [Troubleshooting](#setting-up)
- [More Information](#more-information)
- [Authors](#authors)
- [Credits](#credits)
- [License](#license)


## Resources

### Dynamixel Workbench
----------------------

Control and communication between Dynamixel motors, which can be controlled as one or daisy-chained together for shared power and movement. Deciding on software platforms largely depends on three things: type of Dynamixel motor, other components, and software configuration. Our software was built using the Dynamixel Workbench due to larger control over the adjustments and fine-tuning. 
(Full workbench can be found at https://github.com/MarineRoboticsGroup/dynamixel-workbench) 

#### ROBOTIS e-Manual for Dynamixel Workbench
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)

#### Wiki for dynamixel_workbench Packages
- http://wiki.ros.org/dynamixel_workbench (metapackage)
- http://wiki.ros.org/dynamixel_workbench_controllers
- http://wiki.ros.org/dynamixel_workbench_operators
- http://wiki.ros.org/dynamixel_workbench_single_manager
- http://wiki.ros.org/dynamixel_workbench_single_manager_gui
- http://wiki.ros.org/dynamixel_workbench_toolbox

#### Open Source related to Dynamixel Workbench
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [dynamixel_workbench_msgs](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

#### Documents and Videos related to Dynamixel Workbench
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for OpenCR](http://emanual.robotis.com/docs/en/parts/controller/opencr10/)

Following the steps outlined in the Dynamixel Workbench e-manual worked successfully for us. Configuring the environment and packages for ROS was relatively simple, and it was possible to test the Dynamixels from the computer after a few tweaks outlined in debugging. These are important debugging steps that utilize Dynamixel Wizard 2.0 (available on Windows, Linux, and Mac) for configuring the motors before they can communicate with the NUC remotely. 

### ROS Wrapper for Intel RealSense Devices
-------------------------------------------------------

The Intel RealSense is a great camera for high quality RGB and depth streams for 3D scanning and video capture. The integrated IMU sensor allows for improved navigation capabilities. We went with the RealSense 435i for its global shuttering and larger field of view.  
(Full workbench can be found at https://github.com/IntelRealSense/realsense-ros) 

LibRealSense supported version: v2.29.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

#### Installation Instructions

The simplest way to install on a clean machine is to follow the instructions on the [.travis.yml](https://github.com/intel-ros/realsense/blob/development/.travis.yml) file. It basically summerize the elaborate instructions in the following 3 steps:

Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0
- Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

OR
- Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.29.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

Step 2: Install the ROS distribution
- Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), on Ubuntu 16.04

Step 3: Install Intel&reg; RealSense&trade; ROS from Sources
- Create a [catkin](http://wiki.ros.org/catkin#Installing_catkin) workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
```
- Clone the latest Intel&reg; RealSense&trade; ROS from [here](https://github.com/intel-ros/realsense/releases) into 'catkin_ws/src/'
```bashrc
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..
```
- Make sure all dependent packages are installed. You can check .travis.yml file for reference.
- Specifically, make sure that the ros package *ddynamic_reconfigure* is installed. If *ddynamic_reconfigure* cannot be installed using APT, you may clone it into your workspace 'catkin_ws/src/' from [here](https://github.com/pal-robotics/ddynamic_reconfigure/tree/kinetic-devel) (Version 0.2.0)

```bash
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Usage Instructions

##### Start the camera node
To start the camera node in ROS:

```bash
roslaunch realsense2_camera rs_camera.launch
```

This will stream all camera sensors and publish on the appropriate ROS topics. Other stream resolutions and frame rates can optionally be provided as parameters to the 'rs_camera.launch' file.

#### Published Topics

The published topics differ according to the device and parameters.
After running the above command with D435i attached, the following list of topics will be available (This is a partial list. For full one type `rostopic list`):
- /camera/color/camera_info
- /camera/color/image_raw
- /camera/depth/camera_info
- /camera/depth/image_rect_raw
- /camera/extrinsics/depth_to_color
- /camera/extrinsics/depth_to_infra1
- /camera/extrinsics/depth_to_infra2
- /camera/infra1/camera_info
- /camera/infra1/image_rect_raw
- /camera/infra2/camera_info
- /camera/infra2/image_rect_raw
- /camera/gyro/imu_info
- /camera/gyro/sample
- /camera/accel/imu_info
- /camera/accel/sample
- /diagnostics


### RPLiDAR ROS Node and Test Application
----------------------------------------------

The RPLiDAR A1M8 from Roboshop is a great, inexpensive LiDAR with 360 degree FOV and a large ranging distance that integrates with SLAMAlg. It is also used on the TurtleBot3 Burger. 
Full workbench can be found at https://github.com/robopeak/rplidar_ros) 

Visit following Website for more details about RPLIDAR:

- [rplidar roswiki](http://wiki.ros.org/rplidar)
- [rplidar HomePage](http://www.slamtec.com/en/Lidar)
- [rplidar SDK](https://github.com/Slamtec/rplidar_sdk)
- [rplidar Tutorial](https://github.com/robopeak/rplidar_ros/wiki)

#### How to build rplidar ros package
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build rplidarNode and rplidarNodeClient

#### How to run rplidar ros package

There're two ways to run rplidar ros package

#### 1. Run rplidar node and view in the rviz
roslaunch rplidar_ros view_rplidar.launch (for RPLIDAR A1/A2)
or
roslaunch rplidar_ros view_rplidar_a3.launch (for RPLIDAR A3)

You should see rplidar's scan result in the rviz.

#### 2. Run rplidar node and view using test application

roslaunch rplidar_ros rplidar.launch (for RPLIDAR A1/A2)
or
roslaunch rplidar_ros rplidar_a3.launch (for RPLIDAR A3)

rosrun rplidar_ros rplidarNodeClient

You should see rplidar's scan result in the console

Notice: the different is serial_baudrate between A1/A2 and A3

#### RPLidar frame

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png

## Network setup
------------------

1. Set up NUC and Nano with software (Ubuntu 18.04 and ROS). For the Nano, downloading ROS is a little trickier due to its arm-based architecture but JetsonHacks has nice, full software downloads. It's worth noting that the Jetson software can only be booted from a micro-sd card whereas the NUC could boot off of a flash drive. 
2. Connect the NUC to local wifi and the Nano to NUC via ethernet (Nano doesn't have wifi access).
3. Set up chrome remote desktop on NUC so you can visualize its GUI. Set up ssh for Nano (chrome remote doesn't support arm-based architecture) with openssh.
4. Connect to the NUC via chrome remote and ssh into Nano to begin. While the above section on installing all necessary RealSense software is fine, sudo apt-get install ros-melodic-realsense2-camera downloads all necessary software to the Nano.
5. Set EXPORT ROS_MASTER_URI and EXPORT ROS_IP to Nano's IP address so you can run Rviz from NUC while running the RealSense through the Nano.

## Hardware setup
-----------------

We based our design around similar principles as the Turtlebot3, a popular industry swarm robot. The major differences between the two are from additional sensing capabilities and different components, e.g. swapping the Jetson Nano for the Raspberry Pi computer. 

- (ROS Control Module) Intel NUC7i3DNHNC Mini PC
- (Computer) Jetson Nano Developer Kit
- (UWB/Communications) DWM1001 Development Board
- (Camera) Intel RealSense Depth Camera D435i
- (LiDAR) DFR0315 RPLIDAR A1 Development Kit
- (Actuators) Dynamixel XL430-W250-T
- U2D2*
- U2D2 Power Hub**

\* U2D2 is needed for Dynamixel-to-computer communication  
\** U2D2 Power Hub supplies constant power to Dynamixels, necessary for required voltage and current

#### Connection Diagram
![screenshot](https://user-images.githubusercontent.com/66733789/86042497-3e44d780-ba15-11ea-9029-2bfb11db3b1c.png)

#### Multi-Robot Swarm Connections Diagram
![Screenshot 2020-06-29 at 5 33 57 PM](https://user-images.githubusercontent.com/66733789/86058475-f632ae80-ba2e-11ea-8996-7b5bd60f3f86.png)


## Software setup
--------------

The ROS nodes and communications will be running on Ubuntu 18.04 with ROS Melodic. This can also be done using previous versions of Ubuntu and ROS with most available software. Packages available for reference from ROS Wiki are listed below:

- Navigation Stack = http://wiki.ros.org/navigation
- Dynamixel Workbench Metapackage = http://wiki.ros.org/dynamixel_workbench
- RPLiDAR = http://wiki.ros.org/rplidar
- RealSense = http://wiki.ros.org/RealSense

#### ROS System Architecture Diagram
![Screenshot 2020-06-29 at 2 37 17 PM](https://user-images.githubusercontent.com/66733789/86043502-d5f6f580-ba16-11ea-907d-89f3ed522685.png)


## Design principles
-----------------------

1. Indoor SLAM testing
2. Inexpensive scalability
3. Robust design
4. UWB communications
5. Multiple sensing capabilities and room for more
6. Modular software programming
7. Multi-purpose design

Our design is centered around several principles, which are loosely ordered by importance above. Testing capabilities for SLAM were primarily important for research purposes, and scalability for testing swarms. A robust design is critical for repeated testing and multi-purpose experiments. Since our robot design's function is for indoors SLAM, the cheap yet effective ultra wideband was a solution for efficient indoor communications. Working in ROS, keeping the robot's software modular helps debugging while also keeping the system dynamic. 

#### CAD Design on SolidWorks 2018
![Screenshot 2020-06-29 at 5 30 12 PM](https://user-images.githubusercontent.com/66733789/86058123-5d039800-ba2e-11ea-9ee7-4894279078cf.png)


## Getting Started
----------------

### Setting up

All listed components are the main parts of the robot, but other parts are needed for power distribution and connection. These include: one or more power banks with the required ports, barrel jack cables, TTL adaptors, wheels, ball caster (for stability), and plates (we custom designed and laser-cut the plates). 
(Currently incomplete)

### Running

(Currently incomplete)

### Troubleshooting

- Be sure to use USB3.x for the RealSense camera, otherwise you will run into limitations on depth+rgb throughput when testing in ROS. 
[Intel's website](https://www.intelrealsense.com/wp-content/uploads/2019/03/Depth_Camera_powered_by_Ethernet_WhitePaper.pdf)  
(Currently incomplete)
- The Dynamixels require direct configuration from the Dynamixel Wizard 2.0 application, which can be downloaded onto any platform. For the computer to be able to communicate with each Dynamixel, we found that registering them on Wizard was the most simple. 
    - After connecting the Dynamixels to the NUC and a power source, scan for the connected motors on the Wizard. The XL430-W250 motors that we used were registered with 57600 baud rate and Protocol 2.0. 
    - Once they were registered via scanning, it was necessary to configure them to 'wheel mode' from the automatic 'joint mode'. This could be done through the control panel with 'Operating Mode', changing the mode to '1' or velocity-based. Editing the system controls is only possible when torque mode is turned off, but it should be turned on for running tests. 
    - Since the motors are mirrored, it's necessary to configure them to run in the same direction. This can also be done through Dynamixel Wizard 2.0, by checking 'Reverse-drive mode'. 

## More information

(Currently incomplete)

## Authors

* **Sophia Franklin** 

## Credits
* **ROBOTIS-GIT/dynamixel-workbench**
* **kintzhao and the rest of the contributors for https://github.com/robopeak/rplidar_ros**
* **Intel and the contributors at https://github.com/IntelRealSense/realsense-ros**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

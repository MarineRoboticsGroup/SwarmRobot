# RoboSwarm

Background
--------------

Testing and implementing developed SLAM algorithms is difficult due to the accessibility of fully-formed robotic swarms, and the available swarms are often expensive or ineffective. The goal of our project is to design and build a robot that can communicate via ultra wideband and scale up to a larger swarm, with the capability of assessing SLAM algorithms and swarm experimentation. 

![IMG_20200811_091944 (1)](https://user-images.githubusercontent.com/66733789/90140512-da1d6f00-dd47-11ea-997a-4290ef179129.jpg)

Table of Contents
--------------------

- [Resources](#resources)
  * [Dynamixel Instructions](#dynamixel-workbench)
  * [Intel RealSense Instructions](#ros-wrapper-for-intel-realsense-devices)
  * [RPLiDAR Instructions](#rplidar-ros-node-and-test-application)
- [Network Setup](#network-setup)
  * [Setting Static IP Addresses for the NUC and Nano](#setting-static-ip-addresses-for-the-nuc-and-nano)
  * [Registering Dynamixel and RPLiDAR USB Ports](#registering-dynamixel-and-rplidar-usb-ports)
  * [Environment Variables and Host Key Setup](environment-variables-and-host-key-setup]
- [Hardware Setup](#hardware-setup)
  * [Connection Diagram](#connection-diagram)
  * [Multi-Robot Swarm Connections Diagram](#multi-robot-swarm-connections-diagram)
- [Software Setup](#software-setup)
  * [ROS System Architecture Diagram](#ros-system-architecture-diagram)
  * [Configuring Master Launch File](#configuring-master-launch-file)
- [Design Principles](#design-principles)
  * [CAD Design on SolidWorks 2018](#cad-design-on-solidworks-2018)
- [Getting Started](#getting-started)
  * [Setting Up](#setting-up)
  * [Running](#setting-up)
   * [Hello World for Components](#hello-world-for-components)
    * [RealSense](#realsense)
    * [Dynamixels](#dynamixels)
    * [RPLiDAR](#rplidar)
  * [Troubleshooting](#setting-up)
   * [Registering Dynamixels](#registering-dynamixels)
   * [Printing SolidWorks Drawings To Scale](#printing-solidworks-drawings-to-scale)
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

Once properly set up (either with test computer or NUC), the Dynamixels can be controlled from the keyboard with a few Linux commands. 
```bash
$ cd ~/catkin_ws && catkin_make
$ roslaunch dynamixel_workbench_operators wheel_operator.launch
$ roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch use_cmd_vel:=true
```

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

If you're looking to test with more specific data-gathering modes, you can generate a pointcloud with the RealSense. Visualizing the camera output can be easily set up through Rviz, by launching Rviz in another terminal. There are a few steps to configuring the camera settings, shown below.

To start the pointcloud camera node in ROS:
```bash
roslaunch realsense2_camera rs-camera.launch filters:=pointcloud 
``` 
1. Launch Rviz in separate terminal
2. Switch Fixed Frame to camera_link
3. Add in Image and PointCloud2
4. Designate Image Topic and Topic to the visual input that you want to generate the pointcloud from (it doesn’t automatically select those options, which may result in ‘no image found’)

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
3. Set static IP addresses with nm-connection-editor for the NUC and Nano on both wifi and ethernet sides, as IP addresses are dynamically assigned
4. Set up chrome remote desktop on NUC so you can visualize its GUI. Set up ssh for Nano (chrome remote doesn't support arm-based architecture) with openssh.
5. Dynamixel and RPLiDAR usb ports are also dynamically assigned on NUC but specific to each component, set symlinks in respective launch files via /etc/udev/rules
6. Connect to the NUC via chrome remote and ssh into Nano to begin. While the above section on installing all necessary RealSense software is fine, sudo apt-get install ros-melodic-realsense2-camera downloads all necessary software to the Nano.
7. Set EXPORT ROS_MASTER_URI and EXPORT ROS_IP to Nano's IP address so you can run Rviz from NUC while running the RealSense through the Nano. To launching Nano launch files from NUC requires specific env. variables and a path set with a host key, which is explained in more depth below. 


### Setting Static IP Addresses for the NUC and Nano
--------------------------------------------------

Setting the Nano’s IP address permanently was tricky due to the wandering range of IP addresses that the DCHP would accept/look for. Most of the time, the Nano connected to the NUC using 10.42.0.25, but every so often, 10.42.0.26 might be the correct address.

The solution to this is to go into the nm-manager-editor settings on the Nano (through a hard connection to the monitor + keyboard/mouse) and ‘MANUALLY’ setting the IP configuration to 10.42.0.25 through netmask 255.255.255.255 (IPv4 settings).

### Registering Dynamixel and RPLiDAR USB Ports
--------------------------------------------------

As mentioned above, you must register the USB devices (for the Dynamixels and RPLiDAR) robustly so that they have a consistent USB id. This is because the USB devices are assigned dynamically as each device is plugged in e.g. /dev/ttyUSB0, /dev/ttyUSB1 etc…

To solve this, you can update /etc/udev/rules.d/* to match specific devices and create symbolic links:
/dev/rplidar
/dev/dynamixel

The two rules used:

99-dynamixel-workbench-cdc.rules
```bash
#http://linux-tips.org/t/prevent-modem-manager-to-capture-usb-serial-devices/284/2.

#cp rules /etc/udev/rules.d/
#sudo udevadm control --reload-rules
#sudo udevadm trigger

KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1", SYMLINK+="dynamixel"
``` 

88-rplidar.rules
```bash
# set the udev rule , make the device_port be fixed by rplidar
# ref: https://askubuntu.com/questions/1039814/how-to-create-a-udev-rule-for-two-devices-with-the-same-manufacturer-id-product
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
``` 

Then to reload:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
``` 

Then you can specify a port with each command:
```bash
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch use_cmd_vel:=true usb_port:=/dev/dynamixel
roslaunch rplidar_ros view_rplidar.launch  serial_port:=/dev/rplidar
``` 

### Environment Variables and Host Key Setup
------------------------------------------------

/opt/ros/melodic/env.sh needs to be edited to include specific environment variables on ~/.bashrc.

export ROS_IP=10.42.0.25
export ROS_MASTER_URI=http://10.42.0.1:11311

Set up a group with additional machine info then include the file (which is included in the NUC launch file for conveniance). 

```bash
  <group>
    <machine name="jetset_nano" address="10.42.0.25" user="sophia" password="sop
hia" env-loader="/opt/ros/melodic/env.sh" default="true" />
    <include file="/home/sophia/nanolaunch.launch" />
  </group>
``` 

All of the necessary code and configs must be set up on the master node that is invoking things remotely - the NUC in our case. 

More good info here:
http://wiki.ros.org/roslaunch/XML/machine

SSH keys won't work by default, e.g. if you already connected to the Jetson through SSH. ROS needs a very specific kind of key in ~/.ssh/known_hosts. If you get a weird error then you will need to remove this file (or at least the entry for the Jetson if you can find it) and then reconnect per these instructions:

```bash
ssh -oHostKeyAlgorithms='ssh-rsa' 10.42.0.25

Install realsense camera on NUC so that all of the files are there:
sudo apt-get install ros-melodic-realsense2-camera
``` 

## Hardware Setup
-----------------

We based our design around similar principles as the Turtlebot3, a popular industry swarm robot. The major differences between the two are from additional sensing capabilities and different components, e.g. swapping the Jetson Nano for the Raspberry Pi computer. 

![IMG_20200811_091244 (1)](https://user-images.githubusercontent.com/66733789/90140625-0507c300-dd48-11ea-98be-9b012315e0c9.jpg)
![IMG_20200810_095200](https://user-images.githubusercontent.com/66733789/90140713-1ea90a80-dd48-11ea-95b0-4f4a6d481a8c.jpg)

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
![image](https://user-images.githubusercontent.com/66733789/90141328-02599d80-dd49-11ea-8848-1765b3c7a7d8.png)

#### Multi-Robot Swarm Connections Diagram
![Screenshot 2020-06-29 at 5 33 57 PM](https://user-images.githubusercontent.com/66733789/86058475-f632ae80-ba2e-11ea-8996-7b5bd60f3f86.png)


## Software Setup
--------------

The ROS nodes and communications will be running on Ubuntu 18.04 with ROS Melodic. This can also be done using previous versions of Ubuntu and ROS with most available software. Packages available for reference from ROS Wiki are listed below:

- Navigation Stack = http://wiki.ros.org/navigation
- Dynamixel Workbench Metapackage = http://wiki.ros.org/dynamixel_workbench
- RPLiDAR = http://wiki.ros.org/rplidar
- RealSense = http://wiki.ros.org/RealSense

### Configuring Master Launch File
-----------------------------------

Running multiple launch files with one or more arguments is a little more tricky. We found that you needed to pass in the specific launch file arguments through the component launch files. As an example, setting the USB port for the RPLiDAR isn’t passed in through NUC.launch file, rather you have to go into the test_rplidar.launch file and manually set it through nano test_rplidar.launch.

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
![image](https://user-images.githubusercontent.com/66733789/90139314-31badb00-dd46-11ea-90d8-fe72a71b2b9c.png)
![image](https://user-images.githubusercontent.com/66733789/90139395-4e571300-dd46-11ea-8965-1a4f023bb480.png)

## Getting Started
----------------

### Setting up

All listed components are the main parts of the robot, but other parts are needed for power distribution and connection. These include: one or more power banks with the required ports, barrel jack cables, USB micro cables, USB-C cablesTTL adaptors, sticky velcro tape, wheels, ball caster (for stability), and plates (we custom designed and laser-cut the plates). 
(Currently incomplete)

![image](https://user-images.githubusercontent.com/66733789/90140916-69c31d80-dd48-11ea-9a26-2d006f112616.png)

### Running

Once the wifi and networking is configured, you can Chrome Remote or SSH into the NUC (and then into the Nano from the NUC). You can now communicate and control the NUC/Nano. Whether you're looking to test SLAM algorithms on the NUC's software platform or teleoperate the robot(s), it's all possible through the robot's software platform.

#### Hello World for Components

Hello World for Components: Basic control with individual components

##### RealSense 
1. Run roslaunch realsense2_camera rs_camera.launch to start the realsense node
2. Run realsense-viewer to open the graphical display (best method for testing because user can see camera output)
3. Run roslaunch realsense2_camera rs-camera.launch filters:=pointcloud and keep open
4. In another terminal, run rviz to generate 3D pointcloud 
- Switch Fixed Frame to camera_link
- Add in Image and PointCloud2

Debugging: designate Image Topic and Topic to the visual input that you want to generate the pointcloud from (it doesn’t automatically select those options, which may result in ‘no image found’)

##### Dynamixels 

Once the Dynamixels are configured with Dynamixel Wizard 2.0 (explained in greater detail under 'Troubleshooting', you can control the motors through the terminal. Dynamixel Workbench allow for python and cpp, and we recommend following the manual procedures for a ROS-based platform from the Dynamixel-Workbench GitHub repository. 

Summing up the process:
- There are three gits to download (workbench, msgs, and SDK)
- Setup SDK and Workbench libraries with 64bit arch (skip sample code from SDK)
- Copy rules files, check USB port
- Finding Dynamixels is waste of time as it only registers one (don’t be alarmed by that, only one is connected)
- Test connection with Workbench by 
```bash
cd  ~/dynamixel-workbench/dynamixel_workbench_toolbox/examples/build
./position /dev/ttyUSB0 57600 1 (will rotate pi degrees 6 times)
``` 

##### RPLiDAR

The LiDAR starts spinning as soon as micro-USB is plugged into pc, and you can test the node control with roslaunch rplidar_ros view_rplidar.launch. This program generates a rviz graphical readout where pointcloud is continuously made from LiDAR data.

Debugging: 

An error might occur where the RPLiDAR couldn’t bind to the specified serial port /dev/tty/USB0. This seems to be common LiDAR problem, remedied through the command:
```bash
sudo chmod 666 /dev/ttyUSB0"
``` 
This changes the permissions of RPLiDAR rules file from 0777 to 0666, as it automatically reverts to 0777 when the LiDAR starts running or is plugged in. Once the command is in, everything runs smoothly. (This is separate from long-term set-up with setting sym-links to the RPLiDAR USB port, aimed towards quick initial testing.)

### Troubleshooting

- Be sure to use USB3.x for the RealSense camera, otherwise you will run into limitations on depth+rgb throughput when testing in ROS. 
[Intel's website](https://www.intelrealsense.com/wp-content/uploads/2019/03/Depth_Camera_powered_by_Ethernet_WhitePaper.pdf)  
(Currently incomplete)
- The Dynamixels require direct configuration from the Dynamixel Wizard 2.0 application, which can be downloaded onto any platform. For the computer to be able to communicate with each Dynamixel, we found that registering them on Wizard was the most simple. 
    - After connecting the Dynamixels to the NUC and a power source, scan for the connected motors on the Wizard. The XL430-W250 motors that we used were registered with 57600 baud rate and Protocol 2.0. 
    - Once they were registered via scanning, it was necessary to configure them to 'wheel mode' from the automatic 'joint mode'. This could be done through the control panel with 'Operating Mode', changing the mode to '1' or velocity-based. Editing the system controls is only possible when torque mode is turned off, but it should be turned on for running tests. 
    - Since the motors are mirrored, it's necessary to configure them to run in the same direction. This can also be done through Dynamixel Wizard 2.0, by checking 'Reverse-drive mode'.
- Set Jetson to low power mode to run off of USB (board comes with jumper to switch to barrel-jack input, not needed for low-power mode.
- Configure Dynamixels using Wizard 2.0 (torque on, register USB ports, set direction)
- MaxOak auto-sleeps without enough current draw, keep RPLiDAR plugged in
- Print Solidworks Drawings to scale using custom scale and pdf-poster generator
- Dynamixel threaded holes are only 4mm deep, can breach inner chamber if not careful


#### Registering Dynamixels

As mentioned above, you can register and configure the Dynamixel motors using DYnamixel Wizard 2.0. This was the most simple platform for a quick set-up, and runs on Windows, Linux, and Mac. 

- Set up the Dynamixels with the U2D2, U2D2 Power Hub, and the power source.
- Select Dynamixel X and port COM4 (or the port that the Dynamixel automatically connects to via USB)
  - You can find the connected port on the pc using Device Manager
- It will appear to connect, then run a search using baud rate
- XL-430 connects with any baud rate, can go with the standard '57600'
- Once connected, test out connection with Control Table. Turn on Torque Enable and move around through Goal Position. You can also reverse direction of one or more motors to get opposite motors running in parallel. 

Debugging: MaxOak will shut down if there’s not a high enough current draw. When lights aren’t on/power button hasn’t been pressed in x amount of time, that’s because the two main outputs shut down with 150 and 200 mA. 


#### Printing SolidWorks Drawings To Scale

Printing a full-size drawing of a design can be helpful in the prototyping or manufacturing phase, depending on your set-up and available tools.

Here are some steps for successfully creating a to-scale representation.

1. Create the part in SolidWorks
2. Open as Drawing (DRWG)
3. Depending on the size of the part, make custom paper size (in multiples of 8.5”x11”) if larger than letter paper
4. Put goal view on page and select ‘scale to page’
5. Change page scale to be 1:1 (most printers aren’t accurate enough to print 1:1. Print out a test measurement ex. a six inch line, and modify the first ‘1’ to that value.)
6. Our printer’s scale was closer to 1.03225:1, which I put into the custom scale properties.
7. Save drawing as pdf
8. Open pdf in a pdf reader (make sure the app can print posters if part is larger than letter paper, I used Adobe Acrobat.)
9. Print as poster in pdf reader, spanning across pages with 100% scaling.
10. Cut pages at lines and assemble paper design with tape for a simple to-scale drawing.


## More information

Documentation is available on the MRG Drive with full instructions and explanations. If you have any questions that aren't answered from the GitHub or Google Drive, please feel free to reach out to Sophia Franklin on slack or at sophia.franklin@gmail.com. 

## Authors

* **Sophia Franklin** 

## Credits
* **ROBOTIS-GIT/dynamixel-workbench**
* **kintzhao and the rest of the contributors for https://github.com/robopeak/rplidar_ros**
* **Intel and the contributors at https://github.com/IntelRealSense/realsense-ros**
* **JetsonHacks at jetsonhacks.com**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

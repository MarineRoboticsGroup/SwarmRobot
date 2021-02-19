

# Swarm Robot

## Quick Start

This section will walk you through getting the software and networking set up for your robot to work. This assumes that your robot has been fabricated, assembled, and all components are properly connected. Much of the setup has been automated through the bash scripts provided in this repo. Start by cloning this repository into your home directory on the Intel Nuc and running the Nuc setup script.

### NUC Setup

<details>

This setup assumes that you have either Ubuntu 18.04 or 20.04 installed on the NUC. If not, I recommend following the official Ubuntu tutorials on [installing Ubuntu desktop from USB flash drive](https://ubuntu.com/tutorials/install-ubuntu-desktop#4-boot-from-usb-flash-drive) and [creating a bootable USB stick](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview).

Once this is done, the following instructions will walk you through the necessary steps to set up the NUC.

``` BASH
cd ~
git clone https://github.com/MarineRoboticsGroup/SwarmRobot.git swarm-robot
cd ~/swarm-robot/install
bash ./nucinstall.sh
```

**The bash script may sometimes fail while building the catkin workspace.** In this case: close the terminal, open a new terminal, and rerun `bash ./nucinstall.sh`

This will install all of the basic packages required, create a catkin workspace, clone all of the required repositories into this workspace, build the workspace, and remove the original repo cloned into the home directory. In addition, this performs some of the necessary network configurations of the Nuc to allow for use of this robot in a multi-robot network.

Then to allow for the robots to operate over a common network, change the namespace variables in the robot launch files such that each robot has a unique namespace. In general you can do this by changing the line `<arg name="namespace" default="mrg1" />` to `<arg name="namespace" default="<this robot's unique namespace name>" />`.

As of now the launch file(s) required to change are:

- ~/catkin_ws/src/swarm-robot/ros/launch/swarm_teleop.launch
- ~/catkin_ws/src/dynamixel-workbench/dynamixel_workbench_controllers/launch/dynamixel_controllers.launch

In addition, you will need to modify the ROS environment variables which specify the IP address of the ROS master and the NUC you are currently using. This was partially done in the install file, but the values will need to be modified to suit your desired setup and the specific IP of the NUC you are working on. This setup guide will cover the desired IP addresses you will want to set for several possible situations.

It will be valuable to know the `ifconfig` terminal command and understand which entries correspond to wireless and wired networks. Generally, if the network interface name starts with 'wl' (e.g. 'wlp0s20f3') then it is a wireless network. If the network interface name starts with 'en' (e.g. 'enp7s0f1') then it is an ethernet connection. Note that the format of the device name varies based on the operating system being used, so may not comply with the patterns shown above.

First open your `.bashrc` file in any editor and navigate to the bottom of the file.

``` BASH
nano ~/.bashrc
```

You will see two lines which look like this, where the values in <> are actually temporary values you will **need to change**:

``` BASH
echo "export ROS_IP=<Current NUC IP>" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://<Master IP>:11311" >> ~/.bashrc
```

### IP Address options (NUC Setup)

<details>

#### Running a connected network with a separate computer

This is the most recommended and expected setup, where you will run the robots in a connected network. In this arrangement there will be an additional computer (either a laptop or a desktop which is connected to the same network shared by all of the NUCs) which will be the ROS master. In this scenario you will set the following:

\<Current NUC IP> = the IP address of the NUC on the shared wireless network.
\<Master IP> = the IP address of the connected laptop or desktop on the shared wireless network.

#### Running a connected network without a separate computer

This is like the previous scenario but without the additional computer, which one would likely use for monitoring. It is not immediately clear why someone would want to do this, but it will be listed anyways.

\<Current NUC IP> = the IP address of the NUC on the shared wireless network.
\<Master IP> = the IP address of any of the NUCs on the shared wireless network. **Note**: It is important that in this configuration that all robots must have the same \<Master IP> address.

#### Running a single robot

If you intend to only run this robot independently without having it connected to any other computers over a wireless network then you can set the `Current NUC IP` and `Master IP` to the same values. This could apply to a situation where you are running multiple robots but they are not connected with eachother and are running independently of one another. Such a scenario is possible in locations where it is difficult to establish a wireless network for one reason or another.

In this configuration we're assuming that the NUC is not connected to a wireless network at all.

\<Current NUC IP> = the IP address of the NUC on the shared ethernet network.
\<Master IP> = the IP address of the NUC on the shared ethernet network.

</details>

#### Configure the connection between NUC and Nano (NUC)

Next, you will want to configure the ethernet connection between the Intel NUC and the Jetson Nano. The easiest way to do this is by first making sure the Nano is powered on and connected to the NUC via ethernet. Then open your connections as such:

``` BASH
nm-connection-editor
```

Edit the wired connection between the NUC and the Nano (commonly named 'Wired Connection 1') and navigate to the IPv4 Settings Tab. Change method to `Shared to other computers` and click in the table of addresses to add a new entry. Set the entry to the following values - Address: 10.42.0.1 / Netmask: 24 / Gateway: (leave empty). This will set it such that the IP address of the NUC on the network made over this ethernet connection will always be `10.42.0.1`.

This can be done for all of the NUCs on the network, and will not cause any issues. This is because this is only their IP address for the local network connection they have with their connected Nano, so this IP address will not be reachable for any of the other NUCs in the larger ROS network. The connections between all of the NUCs on the ROS network will be shared over some local wireless network, on which it will have a separate IP address from the local ethernet network it shares with the Nano.

Finally, for convenience it is recommended to disable automatic updates, as described in [this line](https://www.omgubuntu.co.uk/2016/02/how-to-disable-automatic-update-ubuntu).

</details>

### Nano Setup

<details>

The nano setup should be mostly uninvolved. This setup assumes that the Jetson Nano has already been setup and is bootable. If not, follow [this tutorial](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro) to get started with the Nano.

We will first run a setup script that installs the necessary libraries for the Nano to work with the RealSense Camera.

``` BASH
cd ~
git clone https://github.com/MarineRoboticsGroup/SwarmRobot.git swarm-robot
cd ~/swarm-robot/install
bash ./nanoinstall.sh
```

*If having issues try running `bash ./nanoinstall.sh` again*. Sometimes it doesn't fully install the first time.

From here we just need to set up the wired connection between the Nano and the NUC. Make sure that there is an ethernet connecting the two devices and then open the connections configurations with the following command.

``` BASH
nm-connection-editor
```

There should be a single connection profile that says something like 'Wired Connection 1'. Edit this connection. Navigate to the `IPv4 Settings` tab. Change `Method` to *Manual*. In the Addresses table set the first (and only) row item to `Address = 10.42.0.25 | Netmask = 24 | Gateway = 10.42.0.1`.

This should be all that is necessary to set up on the Nano for our base configuration. You should check that the connection between the NUC and Nano is functioning by first trying to ping the NUC from the nano: `ping 10.42.0.1`. If this works then the Nano can find the NUC on the shared wired network between them. You can try ssh-ing into the NUC from the Nano and vice-versa to be more certain that the connections are setup properly, but as our networking is fairly simple ping-ing seems to generally be a sufficient test unless you notice further networking issues when trying to operate the robots.

</details>

### Quickstart Testing

This section will walk you through tests to check that the individual robot is
fully functional (at the component level).

#### Dynamixel and Joystick Testing (NUC)

<details>

This will require 2 separate terminals to run 2 different launch files. In
addition, make sure that a joystick (we use Logitech Wireless Gamepad F710) is plugged into the NUC.

##### terminal 1

``` BASH
roslaunch swarm_robot swarm_teleop.launch
```

##### terminal 2

``` BASH
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch
```

After running these two things and making sure the joystick controller USB is in
the NUC you should be able to control the robot's wheels by holding down the
left bumper (labeled LB) and moving the left joystick around. The other buttons
are currently mapped to different trajectories and should be avoided for now.

</details>

#### UWB Testing (NUC)

Make sure that the UWB is properly connected to the NUC. This means the
light-blueish USB is plugged in and the female jumper connectors on the other end
of this cable are attached to the UWB.

From here, if the `uwb_slam` repo has been cloned into `catkin_ws/src` and
properly built you should be able to test the connection to the UWB by running
`roslaunch uwb_slam_ros uwb_node.launch`. The output should look something like
the following:

``` BASH
.....................
.....................
hal:     UART: Rx 2 bytes: 0x3200
lmh:     UART: Received all 2 bytes, within 19.89 ms     OK
 [GetStatus]: Location data is ready = 1
 [GetStatus]: Node connected to UWB = 1
 [GetStatus]: UWB scan ready ready = 1
 [GetStatus]: User data received = 1
 [GetStatus]: User data ovr UWB sent = 0
 [GetStatus]: Firmware updating? = 0
hal:     UART: Tx 2 bytes: 0x0c00
lmh: rx started, timeout period changed to 18 ms
hal:     UART: Rx 2 bytes: 0x0c00
lmh:     UART: Received all 2 bytes, within 19.70 ms     OK
.....................
.....................
```

issues would be indicated by output that looks like the following:

``` BASH
.....................
.....................
lmh:     LMH_UARTRX_Init()...
hal:     UART: Init start.
hal: *** ERROR *** UART: Unable to open UART. 
hal: *** ERROR *** UART: Ensure it is not in use by another application
hal: *** ERROR *** UART: not initialized. 
lmh: *** ERROR *** UART: Received 0 bytes, expected 7 bytes, timed out in 1000 ms
lmh:     UART: Received length=0
 [GetStatus]: Unknown command or broken TLV.
 [GetStatus]: Location data is ready = 0
 [GetStatus]: Node connected to UWB = 176
 [GetStatus]: UWB scan ready ready = 252
 [GetStatus]: User data received = 127
 [GetStatus]: User data ovr UWB sent = 0
 [GetStatus]: Firmware updating? = 0
hal: *** ERROR *** UART: not initialized. 
lmh: UART: Received 0 bytes, until timed out in 1000 ms          OK
 [GetDistances]: Unknown command or broken TLV.
hal: *** ERROR *** UART: not initialized. 
^C[uwb_node-2] killing on exit
lmh: *** ERROR *** UART: Received 0 bytes, expected 7 bytes, timed out in 1000 ms
lmh:     UART: Received length=0
 [GetStatus]: Unknown command or broken TLV.
 [GetStatus]: Location data is ready = 0
 [GetStatus]: Node connected to UWB = 176
 [GetStatus]: UWB scan ready ready = 252
 [GetStatus]: User data received = 127
 [GetStatus]: User data ovr UWB sent = 0
 [GetStatus]: Firmware updating? = 0
hal: *** ERROR *** UART: not initialized. 
.....................
.....................
```

**Important, Read if Errors!**: the UWB as of now needs to be recognized as `/dev/ttyUSB0` to
function properly according to the uwb_slam repository that is used to interface
with it. This can be first checked by running a bash script inside this
repository `bash <swarm-robot-repo>/scripts/find_dev_port.bash`. The device in
question will be recognized as
'Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001'. However, this can cause
errors as the same USB to serial connector is used on the LiDAR we use. For this
reason, the LiDAR should be disconnected for now until the error has been fixed.
Both of these problems are current issues in this and the `uwb_slam` repos, and
should be fixed in the near future. Sometimes as a quick fix to the problem of
using the correct USB port we can use `sudo setserial -g /dev/ttyUSB0` but this
is not a perfect solution and only works when the LiDAR is not connected.

## Background

<details>

<summary> Please ignore this section until it has been cleaned up and corrected to accommodate the current codebase</summary>

Testing and implementing developed SLAM algorithms is difficult due to the accessibility of fully-formed robotic swarms, and the available swarms are often expensive or ineffective. The goal of our project is to design and build a robot that can communicate via ultra wideband and scale up to a larger swarm, with the capability of assessing SLAM algorithms and swarm experimentation.

![IMG_20200811_091944 (1)](https://user-images.githubusercontent.com/66733789/90140512-da1d6f00-dd47-11ea-997a-4290ef179129.jpg)

## Table of Contents

- [Design Principles](#design-principles)
- [Getting Started](#getting-started)
  - [Setting Up](#setting-up)
  - [Complete Instructions](#complete-instructions)
- [Hardware Setup](#hardware-setup)
  - [Connection Diagram](#connection-diagram)
  - [Multi-Robot Swarm Connections Diagram](#multi-robot-swarm-connections-diagram)
- [Network Setup](#network-setup)
  - [Setting Static IP Addresses for the NUC and Nano](#setting-static-ip-addresses-for-the-nuc-and-nano)
  - [Registering Dynamixel and RPLiDAR USB Ports](#registering-dynamixel-and-rplidar-usb-ports)
  - [Environment Variables and Host Key Setup](#environment-variables-and-host-key-setup)
- [Software Setup](#software-setup)
  - [ROS System Architecture Diagram](#ros-system-architecture-diagram)
  - [Configuring Master Launch File](#configuring-master-launch-file)
- [Resources](#resources)
  - [Dynamixel Instructions](#dynamixel-workbench)
  - [Intel RealSense Instructions](#ros-wrapper-for-intel-realsense-devices)
  - [RPLiDAR Instructions](#rplidar-ros-node-and-test-application)
- [Testing](#testing)
  - [Final Testing](#final-testing)
  - [Hello World for Individual Components](#hello-world-for-individual-components)
    - [RealSense](#realsense)
    - [Dynamixels](#dynamixels)
    - [RPLiDAR](#rplidar)
  - [Troubleshooting](#setting-up)
    - [Registering Dynamixels](#registering-dynamixels)
    - [Printing SolidWorks Drawings To Scale](#printing-solidworks-drawings-to-scale)
- [More Information](#more-information)
- [Authors](#authors)
- [Credits](#credits)
- [License](#license)

## Design principles

To accomplish our goal of designing and building a scalable robot with OTS technology for testing SLAM research through ROS-based software, there were several higher and lower-priority criteria.

1. Indoor SLAM testing
2. Inexpensive scalability
3. Robust design
4. UWB communications
5. Multiple sensing capabilities and room for more
6. Modular software programming
7. Multi-purpose design

Our design is centered around several principles, which are loosely ordered by importance above. Testing capabilities for SLAM were primarily important for research purposes, and scalability for testing swarms. A robust design is critical for repeated testing and multi-purpose experiments. Since our robot design's function is for indoors SLAM, the cheap yet effective ultra wideband was a solution for efficient indoor communications. Working in ROS, keeping the robot's software modular helps debugging while also keeping the system dynamic.

![image](https://user-images.githubusercontent.com/66733789/90139314-31badb00-dd46-11ea-90d8-fe72a71b2b9c.png)
![image](https://user-images.githubusercontent.com/66733789/90139395-4e571300-dd46-11ea-8965-1a4f023bb480.png)

## Getting Started

### Setting up

- Intel NUC7i3DNHNC Mini PC
- MAXOAK Power Bank
- SinKeu Power Bank
- DWM1001 Development Board
- Intel RealSense Depth Camera D435i
- Jetson Nano Developer Kit
- DFR0315 RPLIDAR A1 Development Kit
- Dynamixel XL430-W250-T
- TurtleBot3 Burger Wheel Set
- Ball Caster
- U2D2
- Dynamixel Power Hub

All listed components are the main parts of the robot, but other parts are needed for power distribution and connection. These include: one or more power banks with the required ports, custom JST connectors, barrel jack cables, USB-micro cables, USB-C cables, USB-TTL adaptors, hook and loop velcro tape, wheels, ball caster (for stability), and plates (we custom designed and laser-cut the plates).

![image](https://user-images.githubusercontent.com/66733789/90140916-69c31d80-dd48-11ea-9a26-2d006f112616.png)

### Complete Instructions

Here are the step-by-step instructions for building a swarm robot. Starting from the finished physical build, you can follow the process to get your first official test and 'hello world' with all components and nodes running together. Each step is explained in further detail below, where the individual sections on hardware/build, networking, software, and troubleshooting are expanded on.

1. Build the robot using CAD design, labeled parts, and their corresponding cables. A full list of the components is available on the Drive, as well as in the hardware section below.
2. Download Ubuntu 18.04 onto a USB-key and boot up NUC with the key. You will need to connect the NUC to a monitor and keyboard, as well as a suitable power source.
3. Download the Jetson Nano Developer Kit SD Card Image to a micro-SD card (128 GB is more than enough) and follow Nvidia's set-up instructions to boot up the Nano. The Nano must also be connected to the necessary peripherals as above.
4. Connect back to the NUC, and connect the NUC to the local network (e.g. home network).
5. Download Chrome and install SSH through 'sudo apt-get install openssh-server'.
6. Clone the SwarmRobot github into your terminal space.
7. Run the NUC ROS install file from 'roboswarm' folder. For all files from the SwarmRobot github, make them executable with chmod +x the_file_name to run them.)
8. Run the NUC install and Nano install bash scripts on the NUC, as the control pc needs both the NUC and Nano's install software.
9. Connect the NUC and Nano via ethernet cable, and create an ethernet connection using nm-connection-editor (sudo nm-connection-editor to edit connection). Select ‘connect automatically’ and 'shared to other computers' on the NUC's end and ‘shared to other computers’.
10. Configure the address network under 'shared to other computers' with Address = 10.42.0.1, Netmask = 255.255.255.0, and Gateway = 10.42.0.1. Restart with 'sudo /etc/init.d/networking restart' back in terminal.
11. Connect to the Nano to the display and controls, set up ssh.
12. Clone into the SwarmRobot Github.
13. Run the Nano ROS install file and Nano install bash scripts from the cloned repository, and make sure to set the files as executable.
14. Source and create a catkin workspace for future use and availability (you can follow ROS Wiki's detailed instructions for creating a catkin workspace).
15. Set up the ethernet connection on the Nano's end by editing the connection in nm-connection-editor (sudo nm-connection-editor to edit connection). On the Nano's end, the ethernet is set to 'Automatic (DHCP)'. Restart the network with 'sudo /etc/init.d/networking restart'. If the connection isn't able to turn on, make sure that the NUC is powered on.
16. Set a static IP address for the Nano, explained in-depth in the Networking section. This is critical for connecting to and communicating with the Nano, as its IP address can shift to a range of IDs.
17. Configure the system's environment variables and host keys as explained in the Networking section.
18. If you're working on an Ubuntu machine (the remote control machine, not the NUC or Nano), X11 is automatically set-up (otherwise you can download an applicable X11 server). With ssh, you can work remotely from your pc after this point. X11 will become important with testing, as explained in the Final Tests section.
19. ssh -Y into the NUC (the command looks something like hostname@192.1...). We're using -Y to get the graphical display of the NUC, so we can visualize its GUI and visual outputs.
20. Add 'export ROS_IP=10.42.0.25' and 'export ROS_MASTER_URI=http://10.42.0.25:11311' to your sourcing, at 'gedit ~/.bashrc'. Here, you'll put in the IP address of your current Nano (which you can find via ifconfig).
21. Download Dynamixel Wizard 2.0 and register the Dynamixels by scanning for Protocol 2.0, Baud Rate 57600, and port USB0.
22. Once in, change the first Dynamixel's ID to 2, reverse direction, change Operating Mode to velocity-based (mode 1). Connect the second Dynamixel to the first (to daisy-chain their communication and power sources ) and keep its ID to 1 and change its Operating Mode as well.
23. Create new files as in 'Registering Dynamixel and RPLiDAR USB Ports' and copy in the respective information by following the instructions.
24. Go into the dynamixel_controller.launch and test_rplidar.launch files in your NUC's opt/ros/source... environment and hardcode the symlinks that you just created. This is done by simply writing in /dev/dynamixel and /dev/rplidar into the respective USB port fields, once again explained below in 'Registering Dynamixel and RPLiDAR USB Ports'.
25. Once in the dynamixel_controller launch file, you must also set use_cmd_vel to 'true' to be able to control the wheels.

This is the basic step-by-step guide to getting your Swarm Robot up and running! To begin testing and modifying the components, the next steps are explained in 'Testing'. Many of the above steps are reiterated and expanded on in later sections, but this beginning list is a good resource for general steps. The next few sections cover specific steps and troubleshooting tips, with a larger troubleshooting section at the end of this document.

If you have any questions, there are many great resources (including but not limited to): ROS Wiki, JetsonHacks, RealSense repo, Dynamxiel repo, Dynamixel Workbench e-manual, Nvidia forums, Stack Overflow, and general search queries. The MRG Drive also has additional resources and in-depth explanations. Feel free to reach out as well, either with Slack or through GMail, to the author.

## Hardware Setup

We based our design around similar principles as the Turtlebot3, a popular industry swarm robot. The major differences between the two are from additional sensing capabilities and different components, e.g. swapping the Jetson Nano for the Raspberry Pi computer.

![IMG_20200811_091244 (1)](https://user-images.githubusercontent.com/66733789/90140625-0507c300-dd48-11ea-98be-9b012315e0c9.jpg)
![IMG_20200810_095200](https://user-images.githubusercontent.com/66733789/90140713-1ea90a80-dd48-11ea-95b0-4f4a6d481a8c.jpg)

Main Parts:

- (ROS Control Module) Intel NUC7i3DNHNC Mini PC
- (Computer) Jetson Nano Developer Kit
- (UWB/Communications) DWM1001 Development Board
- (Camera) Intel RealSense Depth Camera D435i
- (LiDAR) DFR0315 RPLIDAR A1 Development Kit
- (Actuators) Dynamixel XL430-W250-T
- (Actuator Comms) U2D2*
- (Actuator Power) U2D2 Power Hub**
- (Battery #1) MaxOak 185Wh/50000mAh Power Bank
- (Battery #2) Sinkeu Power Bank
- (Robot Balance) Ball Caster
- (Robot Movement) Wheels + Tires

\* U2D2 is needed for Dynamixel-to-computer communication
\** U2D2 Power Hub supplies constant power to Dynamixels, necessary for required voltage and current

Cables:

- (NUC-UWB) USB-TTL Cable
- (NUC-Nano) Ethernet Cable
- (NUC-UWB) USB-micro Cable
- (NUC-DYN) USB-micro Cable
- (NUC-RPLiDAR) USB-micro Cable
- (DYN-DYN) JST Cable
- (DYN-U2D2) JST Cable
- (U2D2-PHB) JST Cable
- (NUC-MaxOak) Barrel Jack Connector
- (DYN-MaxOak) Barrel Jack Connector
- (Nano-SinKeu) USB-micro Cable
- (RealSense-Nano) USB-c Cable

### Connection Diagram

![screenshot](https://user-images.githubusercontent.com/66733789/86042497-3e44d780-ba15-11ea-9029-2bfb11db3b1c.png)
![image](https://user-images.githubusercontent.com/66733789/90141328-02599d80-dd49-11ea-8848-1765b3c7a7d8.png)

### Multi-Robot Swarm Connections Diagram

![Screenshot 2020-06-29 at 5 33 57 PM](https://user-images.githubusercontent.com/66733789/86058475-f632ae80-ba2e-11ea-8996-7b5bd60f3f86.png)

## Network setup

The step-by-step network set up is defined above, in the 'Getting Started' section. Here, you can find more detailed explanations and debugging tips.

### Setting Static IP Addresses for the NUC and Nano

Setting the Nano IP address permanently was tricky due to the wandering range of IP addresses that the DCHP would accept/look for. Most of the time, the Nano connected to the NUC using 10.42.0.25, but every so often, 10.42.0.26 might be the correct address.

The solution to this is to go into the nm-manager-editor settings on the Nano (through a hard connection to the monitor + keyboard/mouse) and 'MANUALLY' setting the IP configuration to 10.42.0.25 through netmask 255.255.255.255 (IPv4 settings).

Reload your new network settings via sudo /etc/init.d/networking restart.

### Registering Dynamixel and RPLiDAR USB Ports

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
cd ~/catkin_ws && catkin_make
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch use_cmd_vel:=true usb_port:=/dev/dynamixel
roslaunch rplidar_ros view_rplidar.launch  serial_port:=/dev/rplidar
```

### Environment Variables and Host Key Setup

To visualize the Nano and NUC's displays while controlling the robot, a few variables and exports need to be set up. The main variable file in /opt/ros/melodic/env.sh needs to be edited to include specific environment variables on ~/.bashrc.

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

More helpful info here:
http://wiki.ros.org/roslaunch/XML/machine

SSH keys won't work by default, e.g. if you already connected to the Jetson through SSH. ROS needs a very specific kind of key in ~/.ssh/known_hosts.

If you get a weird error then you will need to remove this file (or at least the entry for the Jetson if you can find it) and then reconnect per these instructions:

```bash
ssh -oHostKeyAlgorithms='ssh-rsa' 10.42.0.25
```

## Software Setup

The ROS nodes and communications will be running on Ubuntu 18.04 with ROS Melodic. This can also be configured using previous versions of Ubuntu and ROS with most available software. Packages available for reference from ROS Wiki are listed below:

- Navigation Stack = http://wiki.ros.org/navigation
- Dynamixel Workbench Metapackage = http://wiki.ros.org/dynamixel_workbench
- RPLiDAR = http://wiki.ros.org/rplidar
- RealSense = http://wiki.ros.org/RealSense

### Configuring Master Launch File

Running multiple launch files with one or more arguments is a little more tricky. We found that you needed to pass in the specific launch file arguments through the component launch files. The minimum format for the master launch file is included in this repository, 'NUC.launch'. Since this is the master launch file, it also loops in the Nano's launch file, 'Nano.launch'. Instructions on setting up this exchange are explained in the 'Setting-Up' section, with a few troubleshooting tips below.

As an example, setting the USB port for the RPLiDAR isn’t passed in through NUC.launch file, rather you have to go into the test_rplidar.launch file and manually set it through nano test_rplidar.launch, from /dev/ttyUSB0 to /dev/rplidar.

#### ROS System Architecture Diagram

![Screenshot 2020-06-29 at 2 37 17 PM](https://user-images.githubusercontent.com/66733789/86043502-d5f6f580-ba16-11ea-907d-89f3ed522685.png)

## Resources

### Dynamixel Workbench

This borrowed repository covers control and communication between Dynamixel motors, which can be done with one or daisy-chained together for shared power and movement. Deciding on software platforms largely depends on three things: type of Dynamixel motor, other components, and software configuration. Our system architecture was built using the Dynamixel Workbench due to larger control over the adjustments and fine-tuning. We suggest going through the Dynamixel Workbench for full instructions and support.

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

Following the steps outlined in the Dynamixel Workbench e-manual worked successfully for us. Configuring the environment and packages for ROS was relatively simple, and it was possible to test the Dynamixels from the computer after a few tweaks outlined in debugging. These are important debugging steps that utilize Dynamixel Wizard 2.0 (available on Windows, Linux, and Mac) for long-term motor configuration before they can communicate with the NUC remotely.

Once properly set up (either with test computer or NUC), the Dynamixels can be controlled from the keyboard with a few Linux commands.

```bash
cd ~/catkin_ws && catkin_make
roslaunch dynamixel_workbench_operators wheel_operator.launch
roslaunch dynamixel_workbench_controllers dynamixel_controllers.launch use_cmd_vel:=true
```

<!-- ### ROS Wrapper for Intel RealSense Devices
TODO (alan) review this section and update

The Intel RealSense is a great camera for high-quality RGB and depth streams for 3D scanning and video capture. The integrated IMU sensor allows for improved navigation capabilities and motion tracking, useful for any SLAM testing. We went with the RealSense 435i for its global shuttering and larger field of view.
(Full workbench can be found at https://github.com/IntelRealSense/realsense-ros)

LibRealSense supported version: v2.29.0 (see [realsense2_camera release notes](https://github.com/IntelRealSense/realsense-ros/releases))

#### Installation Instructions

The simplest way to install on a clean machine is to follow the instructions on the [.travis.yml](https://github.com/intel-ros/realsense/blob/development/.travis.yml) file. It basically summarizes the elaborate instructions in the following 3 steps:

Step 1: Install the latest Intel&reg; RealSense&trade; SDK 2.0

- Install from [Debian Package](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) - In that case treat yourself as a developer. Make sure you follow the instructions to also install librealsense2-dev and librealsense-dkms packages.

OR

- Build from sources by downloading the latest [Intel&reg; RealSense&trade; SDK 2.0](https://github.com/IntelRealSense/librealsense/releases/tag/v2.29.0) and follow the instructions under [Linux Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

Step 2: Install the ROS distribution

- Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu), on Ubuntu 18.04

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

Note that setting this sourcing can complicate having more than one catkin workspaces, but for this project, it works well. -->

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
4. Designate Image Topic and Topic to the visual input that you want to generate the pointcloud from (it doesn't automatically select those options, which may result in 'no image found')

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

The RPLiDAR A1M8 from Roboshop is a great, inexpensive LiDAR with 360 degree FOV and a large ranging distance that integrates with SLAMAlg. It is also used on the TurtleBot3 Burger. Full workbench can be found at https://github.com/robopeak/rplidar_ros)

Visit following websites for more details about RPLIDAR:

- [rplidar roswiki](http://wiki.ros.org/rplidar)
- [rplidar HomePage](http://www.slamtec.com/en/Lidar)
- [rplidar SDK](https://github.com/Slamtec/rplidar_sdk)
- [rplidar Tutorial](https://github.com/robopeak/rplidar_ros/wiki)

#### How to build RPLiDAR ros package

1) Clone this project to your catkin's workspace src folder
2) Running catkin_make to build rplidarNode and rplidarNodeClient

#### How to run RPLIDAR ROS package

There are two ways to run RPLIDAR ROS package
<!-- TODO talk about how the lidar can be configured -->

#### 1. Run RPLIDAR node and view in rviz

roslaunch rplidar_ros view_rplidar.launch (for RPLIDAR A1/A2)
or
roslaunch rplidar_ros view_rplidar_a3.launch (for RPLIDAR A3)

You should see the rplidar scan result in the rviz window.

#### 2. Run RPLIDAR node and view using test application

roslaunch rplidar_ros rplidar.launch (for RPLIDAR A1/A2)
or
roslaunch rplidar_ros rplidar_a3.launch (for RPLIDAR A3)

rosrun rplidar_ros rplidarNodeClient

You should see the rplidar scan result in the console

Notice: the different is serial_baudrate between A1/A2 and A3

#### RPLidar frame

RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png

## Testing

### Final Testing

<!-- TODO (alan) rewrite this section to comply with latest version -->

<!-- Once everything is configured, you can run the NUC launch file to get all of the components moving! The Dynamixels are teleoperable with the w,a,s,d,x keys and all nodes are live (with the Jetson nodes ssh'd through the NUC ssh-key). If you're looking to go one step further and visualize the robot's camera and lidar data, this is where the X11 setup really comes in.

To do so, ssh -Y into the NUC and call roslaunch nuc.launch

- Once the program is running, generate a separate NUC terminal and run rviz.
- You can visualize the RealSense data (all the way from the Nano!) by inputting 'camera_link' and adding in 'Image'. Designate the Image 'topic' with your desired input and the realtime camera feed should appear in the bottom left screen.
- You can add in the laser scan data by then typing 'scan' where camera_link is (the camera feed will stay). Add in the option 'LaserScan' through the option 'published nodes'. Input its topic as /scan and a laser scan readout should appear.
- You can now drive the robot around with realtime camera imaging and laser output. -->

### Hello World for Individual Components

Outlined below is the 'Hello World' for controlling/testing individual components.

#### RealSense

1. Run roslaunch realsense2_camera rs_camera.launch to start the realsense node
2. Run realsense-viewer to open the graphical display (best method for testing because user can see camera output)
3. Run roslaunch realsense2_camera rs-camera.launch filters:=pointcloud and keep open
4. In another terminal, run rviz to generate 3D pointcloud
  a) Switch Fixed Frame to camera_link
  b) Add in Image and PointCloud2

Debugging: designate Image Topic and Topic to the visual input that you want to generate the pointcloud from (it doesn't automatically select those options, which may result in 'no image found')

#### Dynamixels

Once the Dynamixels are configured with Dynamixel Wizard 2.0 (explained in greater detail under 'Troubleshooting', you can control the motors through the terminal. Dynamixel Workbench allow for python and cpp, and we recommend following the manual procedures for a ROS-based platform from the Dynamixel-Workbench GitHub repository.

Summing up the process:

- There are three repositories to download (workbench, msgs, and SDK)
- Setup SDK and Workbench libraries with 64bit arch (skip sample code from SDK)
- Copy rules files, check USB port
- Finding Dynamixels is waste of time as it only registers one (don't be alarmed by that, only one is connected)
- Test connection with Workbench by

```bash
cd  ~/dynamixel-workbench/dynamixel_workbench_toolbox/examples/build
./position /dev/ttyUSB0 57600 1 (will rotate pi degrees 6 times)
```

#### RPLiDAR

The LiDAR starts spinning as soon as micro-USB is plugged into pc, and you can test the node control with roslaunch rplidar_ros view_rplidar.launch. This program generates a rviz graphical readout where pointcloud is continuously made from LiDAR data.

Debugging:

An error might occur where the RPLiDAR couldn't bind to the specified serial port /dev/tty/USB0. This seems to be common LiDAR problem, remedied through the command:

```bash
sudo chmod 666 /dev/ttyUSB0
```

This changes the permissions of RPLiDAR rules file from 0777 to 0666, as it automatically reverts to 0777 when the LiDAR starts running or is plugged in. Once the command is in, everything runs smoothly. (This is separate from long-term set-up with setting sym-links to the RPLiDAR USB port, aimed towards quick initial testing.)

### Troubleshooting

General:

- Be sure to use USB3.x for the RealSense camera, otherwise you will run into limitations on depth+rgb throughput when testing in ROS.
- Set Jetson to low power mode to run off of USB (board comes with jumper to switch to barrel-jack input, not needed for low-power mode.
- MaxOak auto-sleeps without enough current draw, keep RPLiDAR plugged in for initial testing and running.
- If NUC won't turn on try testing the power cables connecting it to the battery. Sometimes they are not fully in and result in a poor connection. Also, sometimes just try flipping the ends of the cable (switching what they are plugged into). It is unclear why this works, but it does :)

Dynamixels (in-depth explanation below):

- The Dynamixels require direct configuration from the Dynamixel Wizard 2.0 application, which can be downloaded onto any platform. For the computer to be able to communicate with each Dynamixel, we found that registering them on Wizard was the most simple.
  - After connecting the Dynamixels to the NUC and a power source, scan for the connected motors on the Wizard. The XL430-W250 motors that we used were registered with 57600 baud rate and Protocol 2.0.
  - Once they were registered via scanning, it was necessary to configure them to 'wheel mode' from the automatic 'joint mode'. This could be done through the control panel with 'Operating Mode', changing the mode to '1' or velocity-based. Editing the system controls is only possible when torque mode is turned off, but it should be turned on for running tests.
  - Since the motors are mirrored, it's necessary to configure them to run in the same direction. This can also be done through Dynamixel Wizard 2.0, by checking 'Reverse-drive mode'.
- Print Solidworks Drawings to scale using custom scale and pdf-poster generator
- Dynamixel threaded holes are only 4mm deep, can breach inner chamber if not careful

#### Registering Dynamixels

As mentioned above, you can register and configure the Dynamixel motors using Dynamixel Wizard 2.0. This was the most simple platform for a quick set-up, and runs on Windows, Linux, and Mac.

- Set up the Dynamixels with the U2D2, U2D2 Power Hub, and the power source.
- Select Dynamixel X and port COM4 (or the port that the Dynamixel automatically connects to via USB)
  - You can find the connected port on the pc using Device Manager
- It will appear to connect, then run a search using baud rate
- XL-430 connects with any baud rate, can go with the standard '57600'
- Once connected, test out connection with Control Table. Turn on Torque Enable and move around through Goal Position. You can also reverse direction of one or more motors to get opposite motors running in parallel.

Debugging: MaxOak will shut down if there's not a high enough current draw. When lights aren't on/power button hasn't been pressed in x amount of time, that's because the two main outputs shut down with 150 and 200 mA.

<!-- 
#### Printing SolidWorks Drawings To Scale

Printing a full-size drawing of a design can be helpful in the prototyping or manufacturing phase, depending on your set-up and available tools.

Here are some steps for successfully creating a to-scale representation:

1. Create the part in SolidWorks
2. Open as Drawing (DRWG)
3. Depending on the size of the part, make custom paper size (in multiples of 8.5"x11") if larger than letter paper
4. Put goal view on page and select 'scale to page'
5. Change page scale to be 1:1 (most printers aren't accurate enough to print 1:1. Print out a test measurement ex. a six inch line, and modify the first '1' to that value.)
6. Our printer's scale was closer to 1.03225:1, which I put into the custom scale properties
7. Save drawing as pdf
8. Open pdf in a pdf reader (make sure the app can print posters if part is larger than letter paper, I used Adobe Acrobat.)
9. Print as poster in pdf reader, spanning across pages with 100% scaling
10. Cut pages at lines and assemble paper design with tape for a simple to-scale drawing -->

## More information

Documentation is available on the MRG Drive with full instructions and explanations. If you have any questions that aren't answered from the GitHub or Google Drive, please feel free to reach out to Sophia Franklin on slack or at sophia.franklin@gmail.com.

<details>

## Authors

- **Sophia Franklin**
- **Alan Papalia**

## Credits

- **ROBOTIS-GIT/dynamixel-workbench**
- **kintzhao and the rest of the contributors for https://github.com/robopeak/rplidar_ros**
- **Intel and the contributors at https://github.com/IntelRealSense/realsense-ros**
- **JetsonHacks at jetsonhacks.com**

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details











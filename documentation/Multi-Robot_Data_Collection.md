# Steps for Collecting Data from a Multi-Robot Set-up

## The SwarmRobots have been configured with a complete platform for both controlling and collecting data from the various on-board sensors and software. While there are many ways of running the robot’s programs, the optimal set-up is through the ROS binary for Procman. Procman is a platform for managing many programs on several computers at once, which can be used to control each function and sensor of the SwarmBots. 

## This write-up is designed with a master computer and one or more swarm robots in mind, however, it can be modified depending on the situation. From configuring the master-agent relationships between the robots, to replaying a ROSBag to visualize the collected data, these steps go through the full set-up process.

1. The master computer and agent robot networks must be configured properly in the bashrc file before any other software is downloaded.
..a. sudo nano ~/.bashrc
..b* ctrl-end
..c* Verify “export ROS_IP=<insert IP address here> matches” the computer’s IP address under wlp0 when running ifconfig
..d* Change export ROS_MASTER_URI=http://<insert IP address here>:11311 to the IP address of the master computer
2. Download Procman-ROS on each of the robots and the master computer.
Procman-ROS depends on LCM, python2, and PyGTK
LCM can be built from source through this link
sudo apt install python-minimal
sudo apt install python-gtk2
cd ~/catkin_ws/src
git clone https://github.com/ashuang/procman.git
cd procman_ros
mkdir build && cd build
cmake ..
make -j4
sudo make install
Assuming that the required software for each device is installed and built, and each robot’s devices are connected, Procman-ROS can be fired up.
On each robot, attach a monitor + keyboard + mouse (preferably Bluetooth for ease of access)
For each robot agent, type in “rosrun procman_ros deputy”
cd ~/catkin_ws/src/procman_ros/test (where the Procman-ROS launch file script should be written
For the master computer, type in “rosrun procman_ros sheriff swarm_manager_3robot.cfg” as well as “rosrun procman_ros deputy”
If the master computer is running anaconda, run “conda deactivate” to avoid python conflicts
With Procman-ROS configured on each respective robot and the master robot, data collection can begin with several commands.
Each robot should be visible in the Procman-ROS GUI up on the master robot
Start all processes/devices required for data collection
In a separate terminal, record a rosbag
The information recorded in the rosbag can either be all published topics with “‘-a” or the specific topics listed after “rosbag record”
Examples of recorded topics: rtabmap/odom, <robot name>/laser, <robot name>/uwb, tf, tf_static, camera/compressed etc
The rosbag will save to whichever location the command is run in
If RTABMap isn’t running through the Procman-ROS GUI, ssh into the robot’s terminal and start RTABMap from there. This output will still be recorded through rosbag record
Once a satisfactory dataset has been recorded, its output can be analyzed with Rviz. 
Play back the recorded rosbag, to simulate real-time data collection, with rosbag play <name of recorded rosbag>
Simultaneously, launch rviz
Add in all topics to be visualized through the broadcasted topics option
Repeat steps as needed to go through different subsets of data from the robot’s collected information

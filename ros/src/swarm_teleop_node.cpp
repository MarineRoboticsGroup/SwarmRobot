/**
 * @file swarmbot_wheel_operator_node.cpp
 * @author Alan Papalia (alanpapalia@gmail.com)
 * @brief the node which handles all teleop control of the wheels for the MRG swarmbots
 * @version 0.1
 * @date 2020-10-25
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <ros/ros.h>
#include "swarm_teleop.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "swarm_teleop_node");

  ros::NodeHandle nh(""), nh_param("~");
  swarm_teleop::SwarmTeleop swarm_teleop(&nh, &nh_param);

  ros::spin();
}
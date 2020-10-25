/**
 * @brief this file allows for driving the robot through predetermined trajectories
 *
 * Authors: Alan Papalia
 */

#include <tuple>
#include <list>
#include <exception>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "swarm_teleop.h"

// moves are defined by a twist and move duration (seconds)
typedef std::tuple<geometry_msgs::Twist, float> MoveInfo;

/**
 * @brief Generates the move info for a sweeping left turn
 *
 * @param radius the radius of the turn
 * @param forward_vel the velocity of the robot during the turn
 * @param degrees the degrees around a circle this turn should last
 * @return MoveInfo the movement parameters to recreate the turn
 */
MoveInfo make_left_turn(float radius, float forward_vel, float degrees)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = forward_vel;
  float angular_vel = forward_vel / radius;
  twist_msg.angular.z = angular_vel;
  float angle_radians = degrees / 180.0 * M_PI;
  float dist_traveled = 2 * radius * angle_radians;
  float time_travel = dist_traveled / forward_vel;
  MoveInfo move = std::make_tuple(twist_msg, time_travel);
  return move;
}

/**
 * @brief Generates the move info for a sweeping right turn
 *
 * @param radius the radius of the turn
 * @param forward_vel the velocity of the robot during the turn
 * @param degrees the degrees around a circle this turn should last
 * @return MoveInfo the movement parameters to recreate the turn
 */
MoveInfo make_right_turn(float radius, float forward_vel, float degrees)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = forward_vel;
  float angular_vel = -forward_vel / radius;
  twist_msg.angular.z = angular_vel;
  float angle_radians = degrees / 180.0 * M_PI;
  float dist_traveled = 2 * radius * angle_radians;
  float time_travel = dist_traveled / forward_vel;
  MoveInfo move = std::make_tuple(twist_msg, time_travel);
  return move;
}

/**
 * @brief generates move information to move in straight line
 *
 * @param dist distance to move
 * @param forward_vel the velocity to move at (negative indicates backwards movement)
 * @return MoveInfo all of the required move information
 */
MoveInfo make_straight_move(float dist, float forward_vel)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = forward_vel;
  float time_travel = dist / forward_vel;
  MoveInfo move = std::make_tuple(twist_msg, time_travel);
  return move;
}

/**
 *
 * @brief generates move information for stationary left turn
 *
 * @param degrees degrees to turn
 * @param angular_vel turning velocity
 * @return MoveInfo all of the required move information
 */
MoveInfo make_stationary_left_turn(float degrees, float angular_vel)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.z = angular_vel;
  float angle_radians = degrees / 180.0 * M_PI;
  float time_travel = angle_radians / angular_vel;
  MoveInfo move = std::make_tuple(twist_msg, time_travel);
  return move;
}

/**
 * @brief generates move information for stationary right turn
 *
 * @param degrees degrees to turn
 * @param angular_vel turning velocity
 * @return MoveInfo all of the required move information
 */
MoveInfo make_stationary_right_turn(float degrees, float angular_vel)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.angular.z = angular_vel;
  float angle_radians = degrees / 180.0 * M_PI;
  float time_travel = -angle_radians / angular_vel;
  MoveInfo move = std::make_tuple(twist_msg, time_travel);
  return move;
}

std::list<MoveInfo> generate_rectangle_traj(float long_side_len, float short_side_len, float max_linear_vel, float max_ang_vel, int num_loops)
{
  std::list<MoveInfo> moves;
  for (int i = 0; i < num_loops; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      moves.push_back(make_straight_move(long_side_len, max_linear_vel));
      moves.push_back(make_stationary_left_turn(90.0, max_ang_vel));
      moves.push_back(make_straight_move(short_side_len, max_linear_vel));
      moves.push_back(make_stationary_left_turn(90.0, max_ang_vel));
    }
  }
  return moves;
}

std::list<MoveInfo> generate_figure_8_traj(float radius, float linear_vel, int num_loops)
{
  std::list<MoveInfo> moves;
  for (int i = 0; i < num_loops; i++)
  {
    moves.push_back(make_left_turn(radius, linear_vel, 360));
    moves.push_back(make_right_turn(radius, linear_vel, 360));
  }
  return moves;
}

// TODO pass in variables from .yaml file
int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "swarmbot_traj_wheel_operator");
  ros::NodeHandle node_handle("");
  ros::Publisher cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber joy_sub = node_handle.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  ros::Rate loop_rate(100);

  if (argc > 1)
  {
    // lin_vel_step = atof(argv[1]);
    // ang_vel_step = atof(argv[2]);
  }

  std::list<MoveInfo> move_queue;
  std::list<MoveInfo> new_moves;
  bool add_to_moves = false;
  move_queue = generate_figure_8_traj(1.0, 0.1, 1);

  auto move_it = move_queue.begin();
  MoveInfo current_move = *move_it;
  geometry_msgs::Twist twist_msg;
  float move_duration;
  std::tie(twist_msg, move_duration) = current_move;

  ros::Time start_move_time = ros::Time::now();
  ros::Time curr_time = ros::Time::now();
  while (ros::ok())
  {
    // TODO finish the button press code
    // figure 8 (radius 2 m each turn)
    if (button_press == A)
    {
      new_moves = generate_figure_8_traj(2.0, 0.1, 1);
      add_to_moves = true;
    }
    // rectangle (3 x 1.5)
    elif (button_press == X)
    {
      new_moves = generate_rectangle_traj(3.0, 1.5, 0.1, 1, 1);
      add_to_moves = true;
    }
    // square (2 x 2)
    elif (button_press == Y)
    {
      new_moves = generate_rectangle_traj(2.0, 2.0, 0.1, 1, 1);
      add_to_moves = true;
    }

    if (add_to_moves)
    {
      move_queue.splice(move_queue.end(), new_moves);
    }

    if (move_it == move_queue.end())
    {
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
      cmd_vel_pub.publish(twist_msg);
    }
    else
    {
      curr_time = ros::Time::now();
      if ((curr_time - start_move_time).toSec() > move_duration)
      {
        ++move_it;
        current_move = *move_it;
        std::tie(twist_msg, move_duration) = current_move;
        start_move_time = ros::Time::now();
      }
    }
    cmd_vel_pub.publish(twist_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

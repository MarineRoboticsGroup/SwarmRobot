/**
 * @brief this file allows for driving the robot freely via teleop along with preprogrammed trajectories which can be triggered by buttons on the controller
 *
 * Authors: Alan Papalia
 */

#include <tuple>
#include <list>
#include <queue>
#include <exception>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "swarm_teleop.h"

namespace swarm_teleop
{
  // moves are defined by a twist and move duration (seconds)
  typedef std::tuple<geometry_msgs::Twist, float> MoveInfo;
  typedef std::queue<MoveInfo> MoveSequence;

  struct SwarmTeleop::Impl
  {
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void perform_traj_in_queue();

    MoveInfo make_stop();
    MoveInfo make_left_turn(float radius, float forward_vel, float degrees);
    MoveInfo make_right_turn(float radius, float forward_vel, float degrees);
    MoveInfo make_straight_move(float dist, float forward_vel);
    MoveInfo make_stationary_left_turn(float degrees, float angular_vel);
    MoveInfo make_stationary_right_turn(float degrees, float angular_vel);
    MoveSequence generate_rectangle_traj(float long_side_len, float short_side_len, float linear_vel, float angular_vel, int num_loops);
    MoveSequence generate_figure_8_traj(float radius, float linear_vel, int num_loops);

    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub;

    std::map<std::string, int> axis_linear_map;
    std::map<std::string, double> scale_linear_map;
    std::map<std::string, int> axis_angular_map;
    std::map<std::string, double> scale_angular_map;

    MoveSequence move_queue;
    MoveInfo current_move;

    float linear_vel, angular_vel;

    int enable_button;
    int rect_button;
    int figure_8_button;
    int square_button;

    bool sent_disable_msg;
  };

  double getVal(const sensor_msgs::Joy::ConstPtr &joy_msg, const std::map<std::string, int> &axis_map,
                const std::map<std::string, double> &scale_map, const std::string &fieldname)
  {
    if (axis_map.find(fieldname) == axis_map.end() ||
        scale_map.find(fieldname) == scale_map.end() ||
        joy_msg->axes.size() <= axis_map.at(fieldname))
    {
      return 0.0;
    }

    return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
  }

  void SwarmTeleop::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    // Initializes with zeros by default.
    geometry_msgs::Twist cmd_vel_msg;

    cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map, "x");
    cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map, "y");
    cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map, "z");
    cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map, "yaw");
    cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map, "pitch");
    cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map, "roll");

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }

  /**
 * Constructs SwarmTeleop. Just sets up the publishers and subscribers and then
 * points everything to the pimpl
 *
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
  SwarmTeleop::SwarmTeleop(ros::NodeHandle *nh, ros::NodeHandle *nh_param)
  {
    pimpl_ = new Impl;
    pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
    pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &SwarmTeleop::Impl::joyCallback, pimpl_);

    nh_param->param<float>("linear_vel", pimpl_->linear_vel, 0.1);
    nh_param->param<float>("angular_vel", pimpl_->angular_vel, 0.5);

    nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
    nh_param->param<int>("rect_button", pimpl_->rect_button, -1);
    nh_param->param<int>("figure_8_button", pimpl_->figure_8_button, -1);
    nh_param->param<int>("square_button", pimpl_->square_button, -1);

    // fill in joystick controller info
    if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
    {
      nh_param->getParam("scale_linear", pimpl_->scale_linear_map);
    }
    else
    {
      nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
      nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["x"], 0.5);
    }

    if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
    {
      nh_param->getParam("scale_angular", pimpl_->scale_angular_map);
    }
    else
    {
      nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
      nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["yaw"], 0.5);
    }

    ROS_INFO_NAMED("SwarmTeleop", "Teleop enable button %i.", pimpl_->enable_button);

    pimpl_->sent_disable_msg = false;
  }

  /**
  * @brief Generates the move info for the robot to stop
  *
  * @return MoveInfo the parameters to stop moving
  */
  MoveInfo SwarmTeleop::Impl::make_stop()
  {
    geometry_msgs::Twist twist_msg;
    float time_travel = 0.001;
    MoveInfo move = std::make_tuple(twist_msg, time_travel);
    return move;
  }

  /**
 * @brief Generates the move info for a sweeping left turn
 *
 * @param radius the radius of the turn
 * @param forward_vel the velocity of the robot during the turn
 * @param degrees the degrees around a circle this turn should last
 * @return MoveInfo the movement parameters to recreate the turn
 */
  MoveInfo SwarmTeleop::Impl::make_left_turn(float radius, float forward_vel, float degrees)
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
  MoveInfo SwarmTeleop::Impl::make_right_turn(float radius, float forward_vel, float degrees)
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
  MoveInfo SwarmTeleop::Impl::make_straight_move(float dist, float forward_vel)
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
  MoveInfo SwarmTeleop::Impl::make_stationary_left_turn(float degrees, float angular_vel)
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
  MoveInfo SwarmTeleop::Impl::make_stationary_right_turn(float degrees, float angular_vel)
  {
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = angular_vel;
    float angle_radians = degrees / 180.0 * M_PI;
    float time_travel = -angle_radians / angular_vel;
    MoveInfo move = std::make_tuple(twist_msg, time_travel);
    return move;
  }

  /**
   * @brief Generates a move sequence to trace out a number of rectangles
   *
   * @param long_side_len long length of the rectangle
   * @param short_side_len short length of the rectangle
   * @param linear_vel the linear velocity during straight moves
   * @param angular_vel the angular velocity during stationary turns
   * @param num_loops the number of rectangles to trace
   * @return MoveSequence all moves to trace the rectangle
   */
  MoveSequence SwarmTeleop::Impl::generate_rectangle_traj(float long_side_len, float short_side_len, float linear_vel, float angular_vel, int num_loops)
  {
    MoveSequence moves;
    for (int i = 0; i < num_loops; i++)
    {
      for (int j = 0; j < 2; j++)
      {
        moves.push(SwarmTeleop::Impl::make_straight_move(long_side_len, linear_vel));
        moves.push(make_stationary_left_turn(90.0, angular_vel));
        moves.push(make_straight_move(short_side_len, linear_vel));
        moves.push(make_stationary_left_turn(90.0, angular_vel));
      }
    }
    moves.push(make_stop());
    return moves;
  }

  /**
   * @brief Generates a move sequence to trace out a number of figure 8s
   *
   * @param radius radius of the figure 8s
   * @param linear_vel linear velocity while traveling
   * @param num_loops number of figure 8s to trace
   * @return MoveSequence the move sequence to trace
   */
  MoveSequence SwarmTeleop::Impl::generate_figure_8_traj(float radius, float linear_vel, int num_loops)
  {
    MoveSequence moves;
    for (int i = 0; i < num_loops; i++)
    {
      moves.push(make_left_turn(radius, linear_vel, 360));
      moves.push(make_right_turn(radius, linear_vel, 360));
    }
    moves.push(make_stop());
    return moves;
  }

  void SwarmTeleop::Impl::perform_traj_in_queue()
  {
    geometry_msgs::Twist twist_msg;
    float move_duration;
    while (!move_queue.empty())
    {
      current_move = move_queue.front();
      std::tie(twist_msg, move_duration) = current_move;
      cmd_vel_pub.publish(twist_msg);
      ros::Duration(move_duration).sleep();
      move_queue.pop();
    }
  }

  void SwarmTeleop::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    ROS_INFO("I AM HERE IN THE CALLBACK");
    // if move queue still has moves we are going to ignore this callback
    if (!move_queue.empty())
    {
      return;
    }

    // int enable_button;
    // int rect_button;
    // int figure_8_button;
    // int square_button;
    if (rect_button >= 0 &&
        joy_msg->buttons.size() > rect_button &&
        joy_msg->buttons[rect_button])
    {
      move_queue = generate_rectangle_traj(3.0, 1.5, linear_vel, angular_vel, 1);
      SwarmTeleop::Impl::perform_traj_in_queue();
    }
    else if (figure_8_button >= 0 &&
             joy_msg->buttons.size() > figure_8_button &&
             joy_msg->buttons[figure_8_button])
    {
      move_queue = generate_figure_8_traj(2.0, linear_vel, 1);
      SwarmTeleop::Impl::perform_traj_in_queue();
    }
    else if (square_button >= 0 &&
             joy_msg->buttons.size() > square_button &&
             joy_msg->buttons[square_button])
    {
      move_queue = generate_rectangle_traj(2.0, 2.0, linear_vel, angular_vel, 1);
      SwarmTeleop::Impl::perform_traj_in_queue();
    }
    else if (enable_button >= 0 &&
             joy_msg->buttons.size() > enable_button &&
             joy_msg->buttons[enable_button])
    {
      sendCmdVelMsg(joy_msg);
    }
    else
    {
      // When enable button is released, immediately send a single no-motion command
      // in order to stop the robot.
      if (!sent_disable_msg)
      {
        // Initializes with zeros by default.
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_pub.publish(cmd_vel_msg);
        sent_disable_msg = true;
      }
    }
  }

} // namespace swarm_teleop
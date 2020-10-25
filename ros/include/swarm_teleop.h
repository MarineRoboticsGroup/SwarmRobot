#ifndef SWARM_WHEEL_OPERATOR_H
#define SWARM_WHEEL_OPERATOR_H

namespace ros
{
  class NodeHandle;
}

namespace swarm_wheel_operator
{

  /**
 * Class implementing the control of the swarmbot wheels via joystick
 */
  class SwarmWheelOperator
  {
  public:
    SwarmWheelOperator(ros::NodeHandle *nh, ros::NodeHandle *nh_param);

  private:
    struct Impl;
    Impl *pimpl_;
  };

} // namespace swarm_wheel_operator

#endif // SWARM_WHEEL_OPERATOR_H

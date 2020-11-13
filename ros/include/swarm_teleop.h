#ifndef SWARM_TELEOP_H
#define SWARM_TELEOP_H

namespace ros
{
  class NodeHandle;
}

namespace swarm_teleop
{

  /**
 * Class implementing the control of the swarmbot wheels via joystick
 */
  class SwarmTeleop
  {
  public:
    SwarmTeleop(ros::NodeHandle *nh, ros::NodeHandle *nh_param);

  private:
    struct Impl;
    Impl *pimpl_;
  };

} // namespace swarm_teleop

#endif // SWARM_TELEOP_H

#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtAction.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace BT;
using WallFollowerAction = nav2_msgs::action::FollowPath;

class WallFollower : public BtAction<WallFollowerAction>
{
public:
  WallFollower(const std::string & name, const BT::NodeConfiguration & config)
  : BtAction<WallFollowerAction>(name, config) {}

  WallFollowerAction::Goal populate_goal() override
  {
    WallFollowerAction::Goal goal;
    nav_msgs::msg::Path path;

    geometry_msgs::msg::PoseStamped pose;

    pose.pose.position.x = 0.1;
    pose.pose.position.y = 0.2;
    pose.pose.position.z = 0.3;

    path.poses.push_back(pose);
    goal.path = path;
    return goal;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<WallFollower>("WallFollower");
}

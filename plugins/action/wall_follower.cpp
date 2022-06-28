#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtAction.hpp"


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
    goal.path = path;
    return goal;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<WallFollower>("WallFollower");
}

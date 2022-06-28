#include "std_srvs/srv/trigger.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"


using namespace BT;
using WallFollowerService = std_srvs::srv::Trigger;

class WallFollower : public BtService<WallFollowerService>
{
public:
  WallFollower(const std::string & name, const BT::NodeConfiguration & config)
  : BtService<WallFollowerService>(name, config) {}

  WallFollowerService::Request::SharedPtr populate_request() override
  {
    WallFollowerService::Request::SharedPtr request;
    return request;
  }

  BT::NodeStatus handle_response(WallFollowerService::Response::SharedPtr response) override
  {
    RCLCPP_INFO(_node->get_logger(),  "Service call complete: " + response->message);
    return BT::NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<WallFollower>("WallFollowerService");
}

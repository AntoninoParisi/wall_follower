#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>

using namespace std;
using namespace BT;
using BT::NodeStatus;


// la parte di IO è presa da qua
//https://bitbucket.org/theconstructcore/exploring-ros2-with-wheeled-robot/commits/82249822e38db55a69a42ec2c5256228efa344c4#chg-CMakeLists.txt
//https://docs.ros.org/en/eloquent/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

/*
class RobotIO : public rclcpp::Node {
  public:
    RobotIO() : Node("RobotIO") {
      publisher_ = this->create_publisher<geometry_msgs::msg::twist>("cmd_vel", 10);
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "laser_scan", 
          rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
          std::bind(&ReadingLaser::laser_callback, this, std::placeholders::_1));
    }

  private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr_sub _msg) {
      lidar = _msg->ranges;
      lidar_len =  lidar.size();

      regions[0] = lidar[0];
      
      //TODO: divide in regions
      //regions[0] = min(lidar[0], 1.0);
      //regions[1] = min(lidar[ floor(lidar_len/3) ] , 1.0);
      //regions[2] = min(lidar[ floor(lidar_len*2/3) ] , 1.0);
      //min(std::vector<float>(lidar.begin() + 1, lidar.end()))
      
      cout << regions << endl;
    }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr_sub subscription_;  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr_pub publisher_;

  std::vector<float> lidar;
  int lidar_len = 0;
  float regions[3] = {1.0};
};

*/

namespace Actions
{
  
  NodeStatus Find_Wall()
  {
    cout << "[ Finding a wall ]" << endl;
    //TODO:

    return NodeStatus::SUCCESS;
  }

  NodeStatus Align()
  {
    cout << "[ Aligning to the wall]" << endl;
    //TODO:

    return NodeStatus::SUCCESS;
  }

   NodeStatus Follow_Wall()
  {
    cout << "[ Following the wall ]" << endl;
    //TODO:

    return NodeStatus::SUCCESS;
  }
  
} // end namespace

#endif // SIMPLE_BT_NODES_H

#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include "std_msgs/msg/string.hpp"

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;
using namespace BT;

class Wall_Follower : public rclcpp::Node
{

  // VARIABILI DI CLASSE
  vector<float> lidar;
  int lidar_len;
  float regions[3] = {1};

  //  publisher_->publish(geometry_msgs::msg::Twist());
  Tree tree;
  geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist();
  sensor_msgs::msg::LaserScan scan_msg = sensor_msgs::msg::LaserScan();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  const char *xml_tree = R"(
                        <root main_tree_to_execute="MainTree">
                            <BehaviorTree ID="MainTree">
                                <Sequence name="Wall follower main sequence">	
                                  <Find_Wall name="Find wall"/>	
                                  <Align name="Align"/>
                                  <Follow_Wall name="Follow wall"/>		
                                </Sequence>
                            </BehaviorTree>
                        </root>
                        )";

public:
  Wall_Follower() : rclcpp::Node("Wall_Follower")
  {

    // rclcpp::sleep_for(chrono::milliseconds(5000));

    // COSTRUTTORE  (Albero, var per le funz, publ/sub)
    BehaviorTreeFactory factory;
    factory.registerSimpleAction("Find_Wall", bind(Wall_Follower::Find_Wall, &twist_msg, &scan_msg));
    factory.registerSimpleAction("Align", bind(Wall_Follower::Align));
    factory.registerSimpleAction("Follow_Wall", bind(Wall_Follower::Follow_Wall));
    tree = factory.createTreeFromText(xml_tree);

    lidar_len = 0;

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/laser_scan",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        bind(&Wall_Follower::laser_callback, this, placeholders::_1));
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Wall_Follower::timer_callback, this));

    // SE GUARDI L'HW QUI CREA UN TIMER LINKATO ALLA FUNZIONE CONTROL LOOP... NON SO QUANTO BENE POSSA FARE UN WHILE NEL COSTRUTTORE PERO'
    //  IN REALTA SAREBBE DA GUARDARE BENE CHE "ATTIVA" rclcpp::spin(wall_follower_node)
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    this->lidar = _msg->ranges;
    this->lidar_len = lidar.size();

    scan_msg = *_msg;

    NodeStatus status = NodeStatus::RUNNING;

    while (status == NodeStatus::RUNNING)
    {
      status = tree.tickRoot();
      rclcpp::sleep_for(chrono::milliseconds(500));
      cout << "TICK " << status << endl;
    }

    // TODO: divide in regions
    // regions[0] = min(lidar[0], 1.0);
    // regions[1] = min(lidar[ floor(lidar_len/3) ] , 1.0);
    // regions[2] = min(lidar[ floor(lidar_len*2/3) ] , 1.0);
    // min(vector<float>(lidar.begin() + 1, lidar.end()))
  }

  void timer_callback()
  {
    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(twist_msg);
  }

  // ACTIONS
  static NodeStatus Find_Wall(geometry_msgs::msg::Twist *vel, sensor_msgs::msg::LaserScan * scan)
  {
    cout << "[ Finding a wall sss]" << endl;
    // TODO:

    vel->linear.x += 1.0;

    // rclcpp::sleep_for(chrono::milliseconds(5000));

    cout << scan->range_min << endl;

    if(scan->range_min > 0)
      return NodeStatus::SUCCESS;
    else
      return NodeStatus::FAILURE;
  }

  static NodeStatus Align()
  {
    cout << "[ Aligning to the wall]" << endl;
    // TODO:

    rclcpp::sleep_for(chrono::milliseconds(5000));

    return NodeStatus::SUCCESS;
  }

  static NodeStatus Follow_Wall()
  {
    cout << "[ Following the wall ]" << endl;
    // TODO:
    rclcpp::sleep_for(chrono::milliseconds(5000));

    return NodeStatus::SUCCESS;
  }
};

#endif // SIMPLE_BT_NODES_H

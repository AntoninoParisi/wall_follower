#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

using namespace std;
using namespace BT;

//class Find_Wall;

class Wall_Follower : public rclcpp::Node
{
public:
  // VARIABILI DI CLASSE

  bool follow_right;
  vector<float> lidar;
  int lidar_len;
  float regions[3];
  float dist_th;

  Tree tree;

  geometry_msgs::msg::Twist twist_msg;
  sensor_msgs::msg::LaserScan scan_msg;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  // COSTRUTTORE 
  Wall_Follower(const char *xml_tree) : rclcpp::Node("Wall_Follower")
  {

    this->follow_right = true;
    this->lidar_len = 0;
    this->regions[0] = 1.0;
    this->regions[1] = 1.0;
    this->regions[2] = 1.0;
    this->dist_th = 0.3;
    
    BehaviorTreeFactory factory;
    //factory.registerNodeType<Find_Wall>("Find_Wall");     
    factory.registerSimpleAction("Find_Wall", bind(Wall_Follower::Find_Wall, this));
    factory.registerSimpleAction("Side_Choice", bind(Wall_Follower::Side_Choice, this));
    factory.registerSimpleAction("Align", bind(Wall_Follower::Align, this));
    factory.registerSimpleAction("Follow_Wall", bind(Wall_Follower::Follow_Wall, this));
    this->tree = factory.createTreeFromText(xml_tree);
    
    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    this->subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        bind(&Wall_Follower::laser_callback, this, placeholders::_1));

    this->twist_msg = geometry_msgs::msg::Twist();

    this->timer_ = this->create_wall_timer(50ms, std::bind(&Wall_Follower::control_loop, this));   
    
  }

  // METODI
private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)  //was "const sensor..."
  {
    //cout << "[ laser_callback ]" << endl;
    
    this->lidar = _msg->ranges;
    this->lidar_len = lidar.size();

    this->regions[0] = lidar[1];
    this->regions[1] = lidar[this->lidar_len/3];
    this->regions[2] = lidar[this->lidar_len*2/3];

    // TODO: divide into regions properly
    // regions[1] = min(lidar[ floor(lidar_len/3) ] , 1.0);
    // regions[2] = min(lidar[ floor(lidar_len*2/3) ] , 1.0);
    // min(vector<float>(lidar.begin() + 1, lidar.end()))
  }

  void control_loop()
  {  
    this->tree.tickRoot();
    publisher_->publish(twist_msg); 
    
  }
  
  // ACTIONS
 
 
  static NodeStatus Find_Wall(Wall_Follower *wall_follower)
  {
    cout << "[ Finding a wall ]" << endl;
            
    if(wall_follower->regions[0] > wall_follower->dist_th){
      wall_follower->twist_msg.linear.x = 1.0;
      return NodeStatus::FAILURE;
    }
    else{
      wall_follower->twist_msg.linear.x = 0.0;
      return NodeStatus::SUCCESS;
      }    
  }
  
  
  static NodeStatus Side_Choice(Wall_Follower *wall_follower)
  {

    wall_follower->follow_right = (wall_follower->regions[2] < wall_follower->regions[1]) ? true : false;

    if(wall_follower->follow_right)
      cout << "[ Follow Right ]" << endl;
    else  
      cout << "[ Follow Left ]" << endl;

    return NodeStatus::SUCCESS;
  }


  static NodeStatus Align(Wall_Follower *wall_follower)
  {
    cout << "[ Aligning to the wall]" << endl;

    if(wall_follower->regions[0] < wall_follower->dist_th){
      wall_follower->twist_msg.angular.z = (wall_follower->follow_right) ? -1.0 : 1.0;
      return NodeStatus::FAILURE;
    }
    else{
      wall_follower->twist_msg.angular.z = 0.0;
      return NodeStatus::SUCCESS;
      }    
  }

  static NodeStatus Follow_Wall(Wall_Follower *wall_follower)
  {
    cout << "[ Following the wall ]" << endl;

    if(wall_follower->regions[0] > wall_follower->dist_th){
      wall_follower->twist_msg.linear.x = 1.0;
      return NodeStatus::FAILURE;
    }
    else{
      wall_follower->twist_msg.linear.x = 0.0;
      return NodeStatus::SUCCESS;
      }    
  }
};







/*

class Find_Wall : public AsyncActionNode 
  {

  Wall_Follower *wall_follower;

  public:
    Find_Wall(const string& name)
      : AsyncActionNode(name, {}){
        //this->wall_follower = Wall_Follower;
      }     

    NodeStatus tick() override
    {
      cout << "[ Finding a wall ]" << endl;
              
      if(wall_follower->regions[0] > wall_follower->dist_th){
        wall_follower->twist_msg.linear.x = 1.0;
        return NodeStatus::RUNNING;
      }
      else{
        wall_follower->twist_msg.linear.x = 0.0;
        return NodeStatus::SUCCESS;
        }

      cout << "[ Something goes wrong! ]" << endl; 
      return NodeStatus::FAILURE;  
    }
  };

*/

#endif // SIMPLE_BT_NODES_H





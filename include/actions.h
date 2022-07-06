#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace BT;

int slice_dim, mid;
float side;
float slim_side;
string action_name;

// TODO: check if we can declare the following variables somehow outside the laser_callback
// vector<float> reg0a, reg0b, reg0, reg1, reg2;

class Wall_Follower : public rclcpp::Node
{
public:
  // VARIABILI DI CLASSE

  bool follow_right;
  vector<float> lidar;
  int lidar_len;
  float regions[3];
  float dist_th;
  float max_vel;

  float slimRays[2]; // rays of left and right  


  bool lastWasFollowWall;

  Tree tree;
  string actual_node;

  geometry_msgs::msg::Twist twist_msg;
  sensor_msgs::msg::LaserScan scan_msg;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  Wall_Follower(const char *xml_tree) : rclcpp::Node("Wall_Follower")
  {
    this->follow_right = true;
    this->lidar_len = 0;
    this->regions[0] = 1.0;
    this->regions[1] = 1.0;
    this->regions[2] = 1.0;
    this->dist_th = 0.4;
    this->max_vel = 0.1;

    this->slimRays[0] = 1.0;
    this->slimRays[1] = 1.0;

    // devo prendermi esattamente i raggi a dx e sx che corrispondono a -180+-2 e 0+-2
    // se uno di questi due lati non rispetta la tresh allora sono di fronte a uno slim wall

    BehaviorTreeFactory factory;
    factory.registerSimpleAction("Find_Wall", bind(Wall_Follower::Find_Wall, this));
    factory.registerSimpleAction("Side_Choice", bind(Wall_Follower::Side_Choice, this));
    factory.registerSimpleAction("Align", bind(Wall_Follower::Align, this));
    factory.registerSimpleAction("Follow_Wall", bind(Wall_Follower::Follow_Wall, this));
    factory.registerSimpleCondition("Side_Empty", bind(Wall_Follower::Side_Empty, this));
    factory.registerSimpleAction("Follow_Slim_Wall", bind(Wall_Follower::Follow_Slim_Wall, this));
    factory.registerSimpleCondition("IsSlimWall", bind(Wall_Follower::IsSlimWall, this)); 
    this->tree = factory.createTreeFromText(xml_tree);

    this->actual_node = "[ Inizializing ]";
    this->lastWasFollowWall = false;

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    this->subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        bind(&Wall_Follower::laser_callback, this, placeholders::_1));

    this->twist_msg = geometry_msgs::msg::Twist();

    this->timer_ = this->create_wall_timer(50ms, std::bind(&Wall_Follower::control_loop, this));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg) // was "const sensor..."
  {
    this->lidar = _msg->ranges;
    this->lidar_len = lidar.size();

    slice_dim = floor(this->lidar_len / 14);
    mid = floor(this->lidar_len / 2) - floor(this->lidar_len / 6);

    vector<float> reg0a(lidar.begin(), lidar.begin() + slice_dim);
    vector<float> reg0b(lidar.end() - slice_dim, lidar.end());
    vector<float> reg0 = reg0a;
    reg0.insert(reg0.end(), reg0b.begin(), reg0b.end());
    vector<float> reg1(lidar.begin() + slice_dim, lidar.begin() + mid);
    vector<float> reg2(lidar.end() - mid, lidar.end() - slice_dim);

    this->regions[0] = min(*min_element(reg0.begin(), reg0.end()), 10.0f);
    this->regions[1] = min(*min_element(reg1.begin(), reg1.end()), 10.0f);
    this->regions[2] = min(*min_element(reg2.begin(), reg2.end()), 10.0f);




    this->slimRays[0] = max(this->lidar[270], max(this->lidar[265],this->lidar[275]));
    this->slimRays[1] = max(this->lidar[45], max(this->lidar[50],max(this->lidar[40],this->lidar[42])));

  }

  void control_loop()
  {
    this->tree.tickRoot();
    publisher_->publish(twist_msg);
  }

  // ACTIONS

  static NodeStatus Find_Wall(Wall_Follower *wall_follower)
  {
    // Go straight until a wall in front region is detected   (TODO: any reagions??)

    action_name = "[ Finding a wall ]";
    if (wall_follower->actual_node != action_name)
    {
      wall_follower->lastWasFollowWall = false;
      wall_follower->actual_node = action_name;
      cout << action_name << endl;
    }

    if (wall_follower->regions[0] > wall_follower->dist_th)
    {
      wall_follower->twist_msg.linear.x = (wall_follower->regions[0] * wall_follower->regions[0] < wall_follower->max_vel) ? wall_follower->regions[0] * wall_follower->regions[0] : wall_follower->max_vel;
      wall_follower->twist_msg.angular.z = -0.2;
      return NodeStatus::FAILURE;
    }
    else
    {
      wall_follower->twist_msg.linear.x = 0.0;
      wall_follower->twist_msg.angular.z = 0.0;
      return NodeStatus::SUCCESS;
    }
  }

  static NodeStatus Side_Choice(Wall_Follower *wall_follower)
  {
    // Set the side to follow according to the closest wall

    wall_follower->follow_right = (wall_follower->regions[1] > wall_follower->regions[2]) ? true : false;
    wall_follower->lastWasFollowWall = false;
    if (wall_follower->follow_right)
      cout << "[ Follow right ]" << endl; // THE WALL IS ON THE ROBOT RIGHT
    else
      cout << "[ Follow left ]" << endl; // THE WALL IS ON THE ROBOT LEFT

    return NodeStatus::SUCCESS;
  }

  static NodeStatus Align(Wall_Follower *wall_follower)
  {
    // Turn, left or right according to the chosen side, until the front region is empty

    action_name = "[ Aligning to the wall ]";
    if (wall_follower->actual_node != action_name)
    {
      wall_follower->lastWasFollowWall = false;
      wall_follower->actual_node = action_name;
      cout << action_name << endl;
    }

    if (wall_follower->regions[0] < wall_follower->dist_th)
    {
      // TO DO: set a angular velocity according to the index of the minimal ray (es: more lateral = more vel)
      wall_follower->twist_msg.angular.z = (wall_follower->follow_right) ? 0.5 : -0.5;
      return NodeStatus::FAILURE;
    }
    else
    {
      wall_follower->twist_msg.angular.z = 0.0;
      return NodeStatus::SUCCESS;
    }
  }

  static NodeStatus Follow_Wall(Wall_Follower *wall_follower)
  {
    // Go straight until a wall in the front region is detected or the side to follow region is empty

    action_name = "[ Following the wall ]";
    if (wall_follower->actual_node != action_name)
    {
      wall_follower->lastWasFollowWall = true;
      wall_follower->actual_node = action_name;
      cout << action_name << endl;
    }

    side = (wall_follower->follow_right) ? wall_follower->regions[2] : wall_follower->regions[1];
    slim_side = (wall_follower->follow_right) ? wall_follower->slimRays[0] : wall_follower->slimRays[1];

    if(wall_follower->regions[0] > wall_follower->dist_th && slim_side > wall_follower->dist_th)
    {
      wall_follower->twist_msg.linear.x = 0.0;
      wall_follower->twist_msg.angular.z = 0.0;
      return NodeStatus::SUCCESS;
    }
    

    if (wall_follower->regions[0] > wall_follower->dist_th && side < wall_follower->dist_th)
    {
      wall_follower->twist_msg.linear.x = (wall_follower->regions[0] * wall_follower->regions[0] < wall_follower->max_vel) ? wall_follower->regions[0] * wall_follower->regions[0] : wall_follower->max_vel;
      cout << "regions " << wall_follower->slimRays[0] << " | " << wall_follower->slimRays[1] << endl;
      return NodeStatus::FAILURE;
    }
    else
    {
      wall_follower->twist_msg.linear.x = 0.0;
      return NodeStatus::SUCCESS;
    }
  }

  static NodeStatus Follow_Slim_Wall(Wall_Follower *wall_follower)
  {
    // Go in circle until a wall appears in the side to follow region (and a wall in the front region is detected)

    action_name = "[ Following a slim wall ]";
    if (wall_follower->actual_node != action_name)
    {
      wall_follower->actual_node = action_name;
      cout << action_name << endl;
    }

    side = (wall_follower->follow_right) ? wall_follower->regions[1] : wall_follower->regions[2];

    if (side > wall_follower->dist_th)
    {
      wall_follower->twist_msg.linear.x = 0.1;
      wall_follower->twist_msg.angular.z = (wall_follower->follow_right) ? -0.5 : 0.5;
      return NodeStatus::FAILURE;
    }
    else
    {
      wall_follower->twist_msg.linear.x = 0.0;
      wall_follower->twist_msg.angular.z = 0.0;
      return NodeStatus::SUCCESS;
    }
  }

  // CONDITIONS

  static NodeStatus Side_Empty(Wall_Follower *wall_follower)
  {
    // Check if the side to follow is empty (If it is the follow fall ends because a slim wall)

    side = (wall_follower->follow_right) ? wall_follower->regions[1] : wall_follower->regions[2];

    if (side > wall_follower->dist_th)
    {
      cout << "The side to follow is empty" << endl;
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }

  static NodeStatus IsSlimWall(Wall_Follower *wall_follower)
  {
    // Go straight until a wall in the front region is detected or the side to follow region is empty

    action_name = "[ IsSlimWall ]";
    if (wall_follower->actual_node != action_name)
    {
      wall_follower->actual_node = action_name;
      cout << action_name << endl;
    }

    side = (wall_follower->follow_right) ? wall_follower->regions[2] : wall_follower->regions[1];

    cout << " is slim ?????  : " << side << " | "<<wall_follower->dist_th<< " | "<<  wall_follower->regions[0]<< " | "<<  (wall_follower->regions[0] > 0.35 && side > 0.2) << endl;

    if (wall_follower->regions[0] > 0.35 && side > 0.2)
    {
      cout << "ok slim" << endl;
      wall_follower->twist_msg.angular.z = (wall_follower->follow_right) ? -0.5 : 0.5;
      wall_follower->twist_msg.linear.x = 0.0;

      return NodeStatus::SUCCESS;
    }
    else
    {

      wall_follower->twist_msg.linear.x = 0.1;
      wall_follower->twist_msg.angular.z = 0.0;

      if(wall_follower->lastWasFollowWall){ // is slim wall only if before was a follow wall action, and now there is no wall to follow neither in front nor side region
        wall_follower->lastWasFollowWall = false;
        return NodeStatus::SUCCESS;
      }
      return NodeStatus::FAILURE;
    }
  }
};

#endif

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
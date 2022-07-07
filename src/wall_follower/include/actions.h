#ifndef ACTIONS_H
#define ACTIONS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace BT;

string action_name;
float side;

class Find_Wall : public AsyncActionNode
{

  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Find_Wall(const string &name) : AsyncActionNode(name, {}) {}

  void init(geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_)
  {
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Go straight until a wall in front region is detected   (TODO: any reagions??)

    cout << "[ Finding a wall ]" << endl;

    while (this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = (this->regions[0] * this->regions[0] < this->max_vel) ? this->regions[0] * this->regions[0] : this->max_vel;
      this->twist_msg->angular.z = -0.15;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;
    return NodeStatus::SUCCESS;
  }
};

class Side_Choice : public AsyncActionNode
{

  float *regions;
  bool *follow_right;

public:
  Side_Choice(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, float *regions_)
  {
    this->follow_right = follow_right_;
    this->regions = regions_;
  }

  NodeStatus tick() override
  {
    /// Set the side to follow according to the closest wall

    *this->follow_right = (this->regions[1] < this->regions[2]) ? true : false;

    if (this->follow_right)
      cout << "[ Follow right ]" << endl; // THE WALL IS ON THE ROBOT RIGHT
    else
      cout << "[ Follow left ]" << endl; // THE WALL IS ON THE ROBOT LEFT

    return NodeStatus::SUCCESS;
  }
};

class Align : public AsyncActionNode
{

  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Align(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_)
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Turn, left or right according to the chosen side, until the front region is empty

    cout << "[ Aligning ]" << endl;

    while (this->regions[0] < this->dist_th)
    {
      this->twist_msg->angular.z = (this->follow_right) ? 0.5 : -0.5;
    }

    this->twist_msg->angular.z = 0.0;
    return NodeStatus::SUCCESS;
  }
};

class Follow_Wall : public AsyncActionNode
{
  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Follow_Wall(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_)
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Go straight until a wall in the front region is detected or the side to follow region is empty

    cout << "[ Following a wall ]" << endl;

    side = (this->follow_right) ? this->regions[1] : this->regions[2];

    while (side < this->dist_th && this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = 0.3;

      side = (this->follow_right) ? this->regions[1] : this->regions[2];
    }

    this->twist_msg->linear.x = 0.0;
    return NodeStatus::SUCCESS;
  }
};

class Follow_Corner : public AsyncActionNode
{
  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Follow_Corner(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_)
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Go in circle until a wall appears in the side to follow region (and a wall in the front region is detected)

    cout << "[ Following a corner ]" << endl;

    while (this->regions[0] > this->dist_th )
    {
      this->twist_msg->linear.x = 0.12;
      this->twist_msg->angular.z = (this->follow_right) ? -0.5 : 0.5;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;
    return NodeStatus::SUCCESS;
  }
};

// CONDITIONS

class Side_Empty : public AsyncActionNode
{

  bool *follow_right;
  float *regions;
  float dist_th;

public:
  Side_Empty(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, float (&regions_)[3], float dist_th_)
  {
    this->follow_right = follow_right_;
    this->regions = regions_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Check if the side to follow is empty (If it is the follow fall ends because a slim wall)

    cout << "[ Is the side empty? ";

    side = (this->follow_right) ? this->regions[1] : this->regions[2];

    if (side > this->dist_th)
    {
      cout << "Yes ]" <<endl;

      return NodeStatus::SUCCESS;
    }
    else
    {
      cout << "No ]" <<endl;

      return NodeStatus::FAILURE;
    }
  }  
};

#endif
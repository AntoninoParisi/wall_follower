#ifndef ACTIONS_H
#define ACTIONS_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <stdio.h>
#include <sys/select.h>
#include <chrono>

using namespace std;
using namespace BT;

string action_name;
float side;
int action_counter=0;
chrono::steady_clock::time_point t_start;

class Find_Wall : public AsyncActionNode
{

  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;
  string *history_actions;  
  float *history_times;

public:
  Find_Wall(const string &name) : AsyncActionNode(name, {}) {}

  void init(geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    // Go straight until a wall in front region is detected

    cout << "[ Finding a wall ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = (this->regions[0] * this->regions[0] < this->max_vel) ? this->regions[0] * this->regions[0] : this->max_vel;
      this->twist_msg->angular.z = -0.15;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    this->history_actions[action_counter] = "Find_Wall";
    this->history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
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

    //*this->follow_right = (this->regions[1] < this->regions[2]) ? true : false;

    if (*(this->follow_right))
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
  string *history_actions;  
  float *history_times;

public:
  Align(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    // Turn, left or right according to the chosen side, until the front region is empty

    cout << "[ Aligning ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] < this->dist_th)
    {
      this->twist_msg->angular.z = (*(this->follow_right)) ? 0.5 : -0.5;
    }

    this->twist_msg->angular.z = 0.0;

    this->history_actions[action_counter] = "Align\t";
    this->history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
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
  string *history_actions;  
  float *history_times;

public:
  Follow_Wall(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    // Go straight until a wall in the front region is detected or the side to follow region is empty

    cout << "[ Following a wall ]" << endl;

    t_start = chrono::steady_clock::now();
    
    side = *(this->follow_right) ? this->regions[1] : this->regions[2];

    while (side < this->dist_th && this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = 0.3;

      side = *(this->follow_right) ? this->regions[1] : this->regions[2];
    }

    this->twist_msg->linear.x = 0.0;

    this->history_actions[action_counter] = "Follow_Wall";
    this->history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
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
  string *history_actions;  
  float *history_times;

public:
  Follow_Corner(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float (&regions_)[3], float max_vel_, float dist_th_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    // Go in circle until a wall appears in the side to follow region (and a wall in the front region is detected)

    cout << "[ Following a corner ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] > this->dist_th )
    {
      this->twist_msg->linear.x = 0.12;
      this->twist_msg->angular.z = *(this->follow_right) ? -0.5 : 0.5;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    this->history_actions[action_counter] = "Follow_Corner";
    this->history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }
};

class Turn : public AsyncActionNode
{
  geometry_msgs::msg::Twist *twist_msg;
  string *history_actions;  
  float *history_times;

public:
  Turn(const string &name) : AsyncActionNode(name, {}) {}

  void init(geometry_msgs::msg::Twist *twist_msg_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->twist_msg = twist_msg_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    // Go in circle until a wall appears in the side to follow region (and a wall in the front region is detected)

    cout << "[ Turning ]" << endl;

    t_start = chrono::steady_clock::now();
    
    int angle = 180;
    int time_ = 1;
    float side = 1;

    while(chrono::duration<float>(chrono::steady_clock::now()- t_start).count() < time_){
      this->twist_msg->linear.x = 0.0;        
      this->twist_msg->angular.z = side*(angle*3.14/180)/time_;
    }
    this->twist_msg->angular.z = 0.0;
  
    this->history_actions[action_counter] = "Turn";
    this->history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }
};

class Rewind : public AsyncActionNode
{
  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float dist_th;
  string *history_actions;  
  float *history_times;

public:
  Rewind(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float dist_th_, string (&history_actions_)[100], float (&history_times_)[100])
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->dist_th = dist_th_;
    this->history_actions = history_actions_;
    this->history_times = history_times_;
  }

  NodeStatus tick() override
  {
    //repeat all action in history

    cout << "[ Rewind ]" << endl;

    *this->follow_right = *(this->follow_right) ? false : true;

    for (int i = action_counter-2; i>0; i--){
      cout << history_actions[i] << "\t|\t" << history_times[i] <<endl;
      t_start = chrono::steady_clock::now();
      while(chrono::duration<float>(chrono::steady_clock::now()- t_start).count() < history_times[i]);
      //TODO: Execute the action
    }
    action_counter = 0;

    *this->follow_right = *(this->follow_right) ? false : true;

    return NodeStatus::SUCCESS;
  }
};

class Exiting : public AsyncActionNode
{
public:
  Exiting(const string &name) : AsyncActionNode(name, {}) {}

  NodeStatus tick() override
  {
    // Tool to advise the running reach the final node on the tree

    cout << "[ Exiting ]" << endl;

    return NodeStatus::SUCCESS;
  }
};



// CONDITIONS

class Side_Occupied : public AsyncActionNode
{
  bool *follow_right;
  float *regions;
  float dist_th;

public:
  Side_Occupied(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, float (&regions_)[3], float dist_th_)
  {
    this->follow_right = follow_right_;
    this->regions = regions_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Check if the side to follow is empty (If it is the follow fall ends because a slim wall)

    cout << "[ Is the side Occupied? ";

    side = *(this->follow_right) ? this->regions[1] : this->regions[2];

    if (side < this->dist_th)
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

class Collision_Detector : public AsyncActionNode
{
  float *regions;
  float dist_th;

public:
  Collision_Detector(const string &name) : AsyncActionNode(name, {}) {}

  void init(float (&regions_)[3], float dist_th_)
  {
    this->regions = regions_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Return FAILURE if something appear in the frontal region

    cout << "." << endl;

    if (this->regions[0] < this->dist_th*0.9)
    {
      cout << "[ Collision Detected ]" <<endl;

      return NodeStatus::FAILURE;
    }
    else
    {
      return NodeStatus::SUCCESS;
    }
  }  
};

class Key_Detector : public AsyncActionNode
{
public:
  Key_Detector(const string &name) : AsyncActionNode(name, {}) {}

  bool inputAvailable()  
  { 
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(0, &fds);
  }
  void clean_stdin(void)
  {
    int c;
    do {
        c = getchar();
    } while (getchar() != '\n' && c != EOF);
  }

  NodeStatus tick() override
  {
    // Return FAILURE when a key is pressed

    if (inputAvailable()){
    clean_stdin();
    cout << "[ keyboard pressed ]" << endl; 
    return NodeStatus::FAILURE;
    } 
    else{ 
      return NodeStatus::SUCCESS;
    }
  }  
};

#endif
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

#include <iostream>
#include <iomanip>

using namespace std;
using namespace BT;

class Find_Wall : public AsyncActionNode
{
  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Find_Wall(const string &name) : AsyncActionNode(name, {}) {}

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float *regions_, float max_vel_, float dist_th_ )
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Go straight until a wall in front region is detected

    // cout << "[ Finding a wall ]" << endl;

    float ang_vel = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX*2.0)) - 1.0;

    while (this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = (this->regions[0] * this->regions[0] < this->max_vel) ? this->regions[0] * this->regions[0] : this->max_vel;
      this->twist_msg->angular.z = ang_vel;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    return NodeStatus::SUCCESS;
  }
  void halt() override{}
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
    
    /*
    if (*this->follow_right)
      cout << "[ Follow right ]" << endl; // THE WALL IS ON THE ROBOT RIGHT
    else
      cout << "[ Follow left ]" << endl; // THE WALL IS ON THE ROBOT LEFT
    */

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

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float *regions_, float max_vel_, float dist_th_ )
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

    // cout << "[ Aligning ]" << endl;
    
    while (this->regions[0] < this->dist_th)
    {
      this->twist_msg->angular.z = (*(this->follow_right)) ? 0.5 : -0.5;     
    }

    this->twist_msg->angular.z = 0.0;

    return NodeStatus::SUCCESS;
  }
  void halt() override{}  
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

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float *regions_, float max_vel_, float dist_th_ )
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

    // cout << "[ Following a wall ]" << endl;
    
    float side = *(this->follow_right) ? this->regions[1] : this->regions[2];

    while (side < this->dist_th && this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = (this->regions[0] * this->regions[0] < this->max_vel) ? this->regions[0] * this->regions[0] : this->max_vel;

      side = *(this->follow_right) ? this->regions[1] : this->regions[2];
    }
    
    this->twist_msg->linear.x = 0.0;

    return NodeStatus::SUCCESS;
  }
  void halt() override{}
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

  void init(bool *follow_right_, geometry_msgs::msg::Twist *twist_msg_, float *regions_, float max_vel_, float dist_th_ )
  {
    this->follow_right = follow_right_;
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->max_vel = max_vel_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Follow a  circle path until a wall appears in the side to follow region (and a wall in the front region is detected)

    // cout << "[ Following a corner ]" << endl;
    
    while (this->regions[0] > this->dist_th )
    {
      this->twist_msg->linear.x = 0.12;
      this->twist_msg->angular.z = *(this->follow_right) ? -0.5 : 0.5;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    return NodeStatus::SUCCESS;
  }
  void halt() override{}
};

class Turn : public AsyncActionNode
{
  geometry_msgs::msg::Twist *twist_msg;

public:
  Turn(const string &name, const NodeConfiguration& config) : AsyncActionNode(name, config) {}

  void init(geometry_msgs::msg::Twist *twist_msg_ )
  {
    this->twist_msg = twist_msg_;
  }

  NodeStatus tick() override
  {
    // Do a rotation around z axes for a given angle in a given amount of seconds or until a wall appears in the back region.
 
    int angle = stoi(getInput<string>("angle").value()); 
    float time_ = stof(getInput<string>("time").value()); 
    int direction= (getInput<string>("direction").value() == "clockwise")? -1 : 1;

    cout << "[ Starting Turning " << angle << "° ... ]" << endl;

    /*
    this->twist_msg->linear.x = 0.0; 
    this->twist_msg->angular.z = 0.0;
    chrono::steady_clock::time_point t_start_halt = chrono::steady_clock::now();
    while(chrono::duration<float>(chrono::steady_clock::now()- t_start_halt).count() < 0.5);
    */

    chrono::steady_clock::time_point t_start_turn = chrono::steady_clock::now();
    while(chrono::duration<float>(chrono::steady_clock::now()- t_start_turn).count() < time_){
      this->twist_msg->linear.x = 0.0;        
      this->twist_msg->angular.z = direction*(angle*3.14/180)/time_;
    }

    this->twist_msg->angular.z = 0.0;
  
    return NodeStatus::SUCCESS;
  }
  void halt() override{}
};

class Go_Back : public AsyncActionNode
{
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float dist_th;

public:
  Go_Back(const string &name, const NodeConfiguration& config) : AsyncActionNode(name, config) {}

  void init(geometry_msgs::msg::Twist *twist_msg_ , float *regions_, float dist_th_)
  {
    this->twist_msg = twist_msg_;
    this->regions = regions_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Go back along x axis for a given distance in a given amount of seconds or until a wall appears in the back region
 
    float distance_ = stof(getInput<string>("distance").value()); 
    float time_ = stof(getInput<string>("time").value()); 

    cout << "[ Starting Go Back " << distance_ << " m ... ]" << endl;

    /*
    this->twist_msg->linear.x = 0.0; 
    this->twist_msg->angular.z = 0.0;
    chrono::steady_clock::time_point t_start_halt = chrono::steady_clock::now();
    while(chrono::duration<float>(chrono::steady_clock::now()- t_start_halt).count() < 0.5);
    */
    
    chrono::steady_clock::time_point t_start_turn = chrono::steady_clock::now();
    while(chrono::duration<float>(chrono::steady_clock::now()- t_start_turn).count() < time_ && this->regions[3] > this->dist_th*0.5){
      this->twist_msg->linear.x = - distance_/time_;  
      this->twist_msg->angular.z = 0.0;      
    }

    this->twist_msg->linear.x  = 0.0;  

    //cout << "[ Ending Go Back " << distance_ << " m ... ]" << endl;

    return NodeStatus::SUCCESS;
  }
    void halt() override{}
};


class Set_Save : public AsyncActionNode
{
  bool *saving_on;

public:
  Set_Save(const string &name, const NodeConfiguration& config) : AsyncActionNode(name, config) {}

  void init(bool *saving_on_)
  {
    this->saving_on = saving_on_;
  }

  NodeStatus tick() override
  {
    // Set the save mode on/off according to the port input 

    *(this->saving_on) = (getInput<string>("mode").value() == "off")? false : true ;
   
    return NodeStatus::SUCCESS;
  }
};


class Rewind : public AsyncActionNode
{
  geometry_msgs::msg::Twist *twist_msg;
  float *history_lin_vel;
  float *history_ang_vel;
  float *history_times;
  int *action_counter;

public:
  Rewind(const string &name) : AsyncActionNode(name, {}) {}

  void init(geometry_msgs::msg::Twist *twist_msg_, float *history_lin_vel_, float *history_ang_vel_, float *history_times_, int* action_counter_)
  {
    this->twist_msg = twist_msg_;
    this->history_lin_vel = history_lin_vel_;
    this->history_ang_vel = history_ang_vel_;
    this->history_times = history_times_;
    this->action_counter = action_counter_;
  }

  NodeStatus tick() override
  {
    // Repeat all action in history

    // cout << "[ Starting Rewind ... ]" << endl;


    for (int i = *(this->action_counter)-1; i > 0; i--){
      
      cout << "doing " << i << " :\t"<< this->history_lin_vel[i] << "\t| " << - this->history_ang_vel[i] << "\tx " << this->history_times[i] << " sec" << endl;
      
      chrono::steady_clock::time_point t_start_rew = chrono::steady_clock::now();
      while(chrono::duration<float>(chrono::steady_clock::now()- t_start_rew).count() < this->history_times[i]){
        this->twist_msg->linear.x = this->history_lin_vel[i];
        this->twist_msg->angular.z = - this->history_ang_vel[i];
      }
    }
  
    cout << "[ ... Rewind Complete ]" << endl;

    *(this->action_counter) = 0;
    
    return NodeStatus::SUCCESS;
  }

void halt() override
    {
      *(this->action_counter) = 0;
    }
};

// CONDITIONS

class Collision_Detector : public ConditionNode
{
  float *regions;
  float dist_th;
  float collision_rate;

public:
  Collision_Detector(const string &name) : ConditionNode(name, {}) {}

  void init(float *regions_, float dist_th_, float collision_rate_)
  {
    this->regions = regions_;
    this->dist_th = dist_th_;
    this->collision_rate = collision_rate_;
  }

  NodeStatus tick() override
  {
    // Return SUCCESS if something appear really close in the frontal region, FAILURE otherwise.

    // cout << "Is a collision detected?";
    // cout << " (" << this->regions[0] << " < " << this->dist_th*0.7 << " ?) ";

    if (this->regions[0] < this->dist_th * this->collision_rate)
    {
      // cout << " YES " <<endl;
      cout << "* collision detected *" << endl;
      return NodeStatus::SUCCESS;
    }
    else
    {
      // cout << " NO " <<endl;
      return NodeStatus::FAILURE;
    }
  }  
};

class Key_Detector : public ConditionNode
{
public:
  Key_Detector(const string &name) : ConditionNode(name, {}) {}

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
    // Return SUCCESS when a key is pressed, FAILURE otherwise.

    // cout << "Is a key detected?";

    if (inputAvailable()){
      clean_stdin();

      // cout << " YES " <<endl;
      cout << "* keyboard pressed *" << endl; 
      return NodeStatus::SUCCESS;
    } 
    else{ 
      // cout << " NO " <<endl;
      return NodeStatus::FAILURE;
    }
  }  
};

#endif
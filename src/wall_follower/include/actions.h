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

string history_actions[1000];  
float history_times[1000];


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

    cout << "[ Finding a wall ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = (this->regions[0] * this->regions[0] < this->max_vel) ? this->regions[0] * this->regions[0] : this->max_vel;
      this->twist_msg->angular.z = (*(this->follow_right)) ? 0.15 : -0.15;;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    history_actions[action_counter] = "Find_Wall";
    history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }

    void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
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
    
    
    if (*this->follow_right)
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

    cout << "[ Aligning ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] < this->dist_th)
    {
      this->twist_msg->angular.z = (*(this->follow_right)) ? 0.5 : -0.5;
    }
    
    this->twist_msg->angular.z = 0.0;

    history_actions[action_counter] = "Align";
    history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }

  void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
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

    cout << "[ Following a wall ]" << endl;

    t_start = chrono::steady_clock::now();
    
    side = *(this->follow_right) ? this->regions[1] : this->regions[2];

    while (side < this->dist_th && this->regions[0] > this->dist_th)
    {
      this->twist_msg->linear.x = 0.3;

      side = *(this->follow_right) ? this->regions[1] : this->regions[2];
    }

    this->twist_msg->linear.x = 0.0;

    history_actions[action_counter] = "Follow_Wall";
    history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }

  void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
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

    cout << "[ Following a corner ]" << endl;

    t_start = chrono::steady_clock::now();
    
    while (this->regions[0] > this->dist_th )
    {
      this->twist_msg->linear.x = 0.12;
      this->twist_msg->angular.z = *(this->follow_right) ? -0.5 : 0.5;
    }

    this->twist_msg->linear.x = 0.0;
    this->twist_msg->angular.z = 0.0;

    history_actions[action_counter] = "Follow_Corner";
    history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }

  void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
    }
};

class Turn : public AsyncActionNode
{
  geometry_msgs::msg::Twist *twist_msg;


public:
  Turn(const string &name) : AsyncActionNode(name, {}) {}

  void init(geometry_msgs::msg::Twist *twist_msg_ )
  {
    this->twist_msg = twist_msg_;
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
  
    history_actions[action_counter] = "Turn";
    history_times[action_counter++] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
    return NodeStatus::SUCCESS;
  }

  void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
    }
};

class Rewind : public AsyncActionNode
{
  bool *follow_right;
  geometry_msgs::msg::Twist *twist_msg;
  float *regions;
  float max_vel;
  float dist_th;

public:
  Rewind(const string &name) : AsyncActionNode(name, {}) {}

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
    //repeat all action in history

    cout << "[ Starting Rewind ... ]" << endl;

    cout << action_counter << endl;

    if(action_counter > 1){

      *(this->follow_right) = (*(this->follow_right) )? false : true;

      for (int i = action_counter-1; i>0; i--){

        t_start = chrono::steady_clock::now();
        
        if (history_actions[i] == "Find_Wall")
        {
          Find_Wall node("Find_Wall");
          node.init(this->follow_right, this->twist_msg, this->regions, this->max_vel, this->dist_th);
          node.tick();
        }
        else if (history_actions[i] == "Side_Choice")
        {
          Side_Choice node("Side_Choice");
          node.init(this->follow_right, this->regions);
          node.tick();
        }
        else if (history_actions[i] == "Align")
        {
          Align node("Align");
          node.init(this->follow_right, this->twist_msg, this->regions, this->max_vel, this->dist_th);
          node.tick();
        }
        else if (history_actions[i] == "Follow_Wall")
        {
          Follow_Wall node("Follow_Wall");
          node.init(this->follow_right, this->twist_msg, this->regions, this->max_vel, this->dist_th);
          node.tick();
        }
        else if (history_actions[i] == "Follow_Corner")
        {
          Follow_Corner node("Follow_Corner");
          node.init(this->follow_right, this->twist_msg, this->regions, this->max_vel, this->dist_th);
          node.tick();
        }
        else if (history_actions[i] == "Turn")
        {
          Turn node("Turn");
          node.init(this->twist_msg);
          node.tick();
        }
        while(chrono::duration<float>(chrono::steady_clock::now()- t_start).count() < history_times[i]);
      }
      
      *(this->follow_right) = (*(this->follow_right) )? false : true;
    
      cout << "[ ... Rewind Complete ]" << endl;

    }
    else{
      cout << "[ ERROR: No action stored in history ]" << endl;
    }

    action_counter = 0;
    return NodeStatus::SUCCESS;
  }

  void halt() override
    {
      this->twist_msg->linear.x = 0.0;
      this->twist_msg->angular.z = 0.0;
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

class Collision_Detector : public ConditionNode
{
  float *regions;
  float dist_th;

public:
  Collision_Detector(const string &name) : ConditionNode(name, {}) {}

  void init(float *regions_, float dist_th_)
  {
    this->regions = regions_;
    this->dist_th = dist_th_;
  }

  NodeStatus tick() override
  {
    // Return FAILURE if something appear in the frontal region

    // cout << "Is a collision detected?";
    // cout << " (" << this->regions[0] << " < " << this->dist_th*0.7 << " ?) ";

    if (this->regions[0] < this->dist_th*0.7)
    {
      // cout << " YES " <<endl;
      cout << "[ collision detected ]" << endl;
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
    // Return FAILURE when a key is pressed

    // cout << "Is a key detected?";

    if (inputAvailable()){
      clean_stdin();

      // cout << " YES " <<endl;
      cout << "[ keyboard pressed ]" << endl; 
      return NodeStatus::SUCCESS;
    } 
    else{ 
      // cout << " NO " <<endl;
      return NodeStatus::FAILURE;
    }
  }  
};

#endif
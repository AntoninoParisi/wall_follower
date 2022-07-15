#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"

#include "actions.h"

using namespace std;
using namespace BT;

int tick_c = 0;

class Wall_Follower : public rclcpp::Node
{
public:
  // VARIABILI DI CLASSE

  bool follow_right;
  vector<float> lidar;
  int lidar_len;
  float regions[4];
  float dist_th;
  float collision_rate;
  float max_vel;
  
  Tree tree;

  geometry_msgs::msg::Twist twist_msg;
  sensor_msgs::msg::LaserScan scan_msg;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_control_loop;
  rclcpp::TimerBase::SharedPtr timer_history;

  int controll_loop_timer_freq;
  int history_timer_freq;
  
  float history_lin_vel[500]; 
  float history_ang_vel[500]; 
  float history_times[500]; 
  bool saving_on;
  int action_counter;
  chrono::steady_clock::time_point t_start;
  

  Wall_Follower(const char *xml_tree, float dist_th_, float max_vel_, float collision_rate_) : rclcpp::Node("Wall_Follower")
  {
    this->dist_th = dist_th_;
    this->max_vel = max_vel_;
    this->collision_rate = collision_rate_;

    this->follow_right = true;
    this->lidar_len = 0;
    this->regions[0] = 1.0;
    this->regions[1] = 1.0;
    this->regions[2] = 1.0;

    this->controll_loop_timer_freq = 50;
    this->history_timer_freq = 10;
    
    BehaviorTreeFactory factory;
    factory.registerNodeType<Find_Wall>("Find_Wall");
    factory.registerNodeType<Side_Choice>("Side_Choice");
    factory.registerNodeType<Align>("Align");
    factory.registerNodeType<Follow_Wall>("Follow_Wall");
    factory.registerNodeType<Follow_Corner>("Follow_Corner");
    factory.registerNodeType<Key_Detector>("Key_Detector");
    factory.registerNodeType<Rewind>("Rewind"); 
    factory.registerNodeType<Collision_Detector>("Collision_Detector"); 
    factory.registerNodeType<Turn>("Turn", {InputPort<string>("angle"), InputPort<string>("time"), InputPort<string>("direction") }); 
    factory.registerNodeType<Go_Back>("Go_Back", {InputPort<string>("distance"), InputPort<string>("time")});
    factory.registerNodeType<Set_Save>("Set_Save", {InputPort<string>("mode")}); 
    this->tree = factory.createTreeFromText(xml_tree);

    for( auto& node: this->tree.nodes ){
      if( auto node_ = dynamic_cast<Find_Wall*>( node.get() ))
      {
          node_->init(&(this->follow_right), &(this->twist_msg), this->regions, this->max_vel, this->dist_th);
      }
      if( auto node_ = dynamic_cast<Side_Choice*>( node.get() ))
      {
          node_->init(&(this->follow_right), this->regions);
      }
      if( auto node_ = dynamic_cast<Align*>( node.get() ))
      {
          node_->init(&(this->follow_right), &(this->twist_msg), this->regions, this->max_vel, this->dist_th);
      }
      if( auto node_ = dynamic_cast<Follow_Wall*>( node.get() ))
      {
          node_->init(&(this->follow_right), &(this->twist_msg), this->regions, this->max_vel, this->dist_th);
      }
      if( auto node_ = dynamic_cast<Follow_Corner*>( node.get() ))
      {
          node_->init(&(this->follow_right), &(this->twist_msg), this->regions, this->max_vel, this->dist_th);
      }
      if( auto node_ = dynamic_cast<Rewind*>( node.get() ))
      {
          node_->init(&(this->twist_msg), this->history_lin_vel, this->history_ang_vel, this->history_times, &(this->action_counter), this->controll_loop_timer_freq);
      }
      if( auto node_ = dynamic_cast<Collision_Detector*>( node.get() ))
      {
          node_->init(this->regions, this->dist_th, this->collision_rate);
      }
      if( auto node_ = dynamic_cast<Turn*>( node.get() ))
      {
          node_->init(&(this->twist_msg));
      }
      if( auto node_ = dynamic_cast<Go_Back*>( node.get() ))
      {
          node_->init(&(this->twist_msg), this->regions, this->dist_th);
      }
      if( auto node_ = dynamic_cast<Set_Save*>( node.get() ))
      {
          node_->init(&(this->saving_on));
      }
    }

    this->publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    this->subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        bind(&Wall_Follower::laser_callback, this, placeholders::_1));

    this->twist_msg = geometry_msgs::msg::Twist();
    this->timer_control_loop = this->create_wall_timer(chrono::milliseconds(this->controll_loop_timer_freq), std::bind(&Wall_Follower::control_loop, this));
    this->timer_history = this->create_wall_timer(chrono::milliseconds(this->history_timer_freq), std::bind(&Wall_Follower::save_twist_msg, this));

    this->action_counter=0;
    this->t_start = chrono::steady_clock::now();
    this->saving_on = true;

    cout << "[ Starting Wall Follower]" << endl;
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    this->lidar = _msg->ranges;
    this->lidar_len = lidar.size();

    int front_dim = floor(this->lidar_len / 12);  //half of it
    int limit = floor(this->lidar_len / 4);

    vector<float> reg_c1(lidar.begin(), lidar.begin() + front_dim);
    vector<float> reg_c2(lidar.end() - front_dim, lidar.end());
    vector<float> reg_c = reg_c1;
    reg_c.insert(reg_c.end(), reg_c2.begin(), reg_c2.end());
    vector<float> reg_l(lidar.begin() + front_dim, lidar.begin() + limit);
    vector<float> reg_r(lidar.end() - limit, lidar.end() - front_dim);
    vector<float> reg_b(lidar.begin() + limit, lidar.end() - limit);

    this->regions[0] = min(*min_element(reg_c.begin(), reg_c.end()), 10.0f);
    this->regions[1] = min(*min_element(reg_r.begin(), reg_r.end()), 10.0f);
    this->regions[2] = min(*min_element(reg_l.begin(), reg_l.end()), 10.0f);
    this->regions[3] = min(*min_element(reg_b.begin(), reg_b.end()), 10.0f);

    //cout << this->regions[2] << " ! " << this->regions[0] << " ! " << this->regions[1] << endl;
  }

  void control_loop()
  {
    this->tree.tickRoot();
    publisher_->publish(twist_msg);

    // cout << setprecision(2) << " --- " <<  twist_msg.linear.x << " | " << twist_msg.angular.z << " --- " << endl;
    // cout << "\n--- executeTick() " << tick_c++ << " ---" << endl;
    // cout << "." << endl;
  }

  void save_twist_msg(){ 

    // Save only new values of velocity and when 'saving' is on

    if( (this->saving_on && ((history_lin_vel[action_counter] - this->twist_msg.linear.x)*(history_lin_vel[action_counter] - this->twist_msg.linear.x) > 0.01 || (history_ang_vel[action_counter] - this->twist_msg.angular.z)*(history_ang_vel[action_counter] - this->twist_msg.angular.z) > 0.01) ) 
      && !(this->twist_msg.linear.x == 0 && this->twist_msg.angular.z == 0) ) {
        
      history_lin_vel[action_counter+1] = this->twist_msg.linear.x; 
      history_ang_vel[action_counter+1] = this->twist_msg.angular.z;
      history_times[action_counter] = chrono::duration<float>(chrono::steady_clock::now()- t_start).count();
      t_start = chrono::steady_clock::now();

      cout << setprecision(2) << "Saving " << action_counter << " :\t" << history_lin_vel[action_counter] << "\t| " << history_ang_vel[action_counter] << "\tx " << history_times[action_counter] << " sec" << endl;
      action_counter++;
    }
  }

};

#endif
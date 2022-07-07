#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


class DirectionPublisher : public rclcpp::Node
{

public:
  DirectionPublisher()
      : Node("direction_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("/direction", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&DirectionPublisher::timer_callback, this));

    message = std_msgs::msg::Bool();
    message.data = true;
    actual_dir = 'r';

    RCLCPP_INFO(this->get_logger(), "Choose between left (L) and right (R); actual direction : RIGHT");
  }

private:
  void timer_callback()
  {
    char ch;
    cout << "per cambiare direzione gigitare L o R \n" << endl;
    cin >> ch;

    cout << "CHAR : " << (char) ch ;
    message.data = (char) ch == 'r' ? true : false;
    (char)ch == 'r' ?  actual_dir = 'r' : actual_dir = 'l';
    string msg_str = (char)actual_dir == 'r' ? "RIGHT" : "LEFT";
    cout << "Choose between left (L) and right (R); actual direction : " << msg_str << endl;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

  std_msgs::msg::Bool message;

  char actual_dir;
  
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionPublisher>());
  rclcpp::shutdown();
  return 0;
}
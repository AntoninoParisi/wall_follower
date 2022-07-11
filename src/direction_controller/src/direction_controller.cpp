#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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
    publisher_ = this->create_publisher<std_msgs::msg::String>("/direction", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&DirectionPublisher::timer_callback, this));

    message = std_msgs::msg::String();
    message.data = true;
    actual_dir = 'r';

    RCLCPP_INFO(this->get_logger(), "Choose between left (L) and right (R); actual direction : RIGHT");
  }

private:
  void timer_callback()
  {
    char ch;
    cout << "per cambiare direzione digitare L o R / altro taso == REWIND \n" << endl;
    cin >> ch;

    cout << "CHAR : " << (char) ch ;
    if((char) ch == 'l')
    {
      message.data = "LEFT";
    }
    else if((char) ch == 'r')
    {
      message.data = "RIGHT";
    }
    else{
      message.data = "REWIND";
    }


    cout << "Choose between left (L) and right (R); actual direction : " << message.data << endl;
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  std_msgs::msg::String message;

  char actual_dir;
  
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionPublisher>());
  rclcpp::shutdown();
  return 0;
}
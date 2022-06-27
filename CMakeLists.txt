cmake_minimum_required(VERSION 3.5)
project(wall_follower)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(BehaviorTreeV3 REQUIRED)




add_executable(talker src/publisher.cpp)
add_executable(listener src/listener.cpp)
add_executable(testerBT src/BTclass/ApproachObject.cpp)



ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(testerBT rclcpp std_msgs BehaviorTreeV3)
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()


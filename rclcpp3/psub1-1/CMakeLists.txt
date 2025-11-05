cmake_minimum_required(VERSION 3.5)
project(psub1-1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub1-1 src/pub1-1.cpp)
ament_target_dependencies(pub1-1 rclcpp std_msgs)

add_executable(sub1-1 src/sub1-1.cpp)
ament_target_dependencies(sub1-1 rclcpp std_msgs)

install(TARGETS
  pub1-1
  sub1-1
  DESTINATION lib/${PROJECT_NAME})

ament_package()

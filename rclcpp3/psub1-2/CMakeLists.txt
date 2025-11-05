cmake_minimum_required(VERSION 3.5)
project(psub1-2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pub1-2 src/pub1-2.cpp)
ament_target_dependencies(pub1-2 rclcpp std_msgs)

add_executable(sub1-2 src/sub1-2.cpp)
ament_target_dependencies(sub1-2 rclcpp std_msgs)

install(TARGETS
  pub1-2
  sub1-2
  DESTINATION lib/${PROJECT_NAME})

ament_package()

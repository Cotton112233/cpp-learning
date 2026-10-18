cmake_minimum_required(VERSION 3.16)
project(hip_topic_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hip_angle_publisher src/hip_angle_publisher.cpp)
ament_target_dependencies(hip_angle_publisher rclcpp std_msgs)

add_executable(hip_angle_subscriber src/hip_angle_subscriber.cpp)
ament_target_dependencies(hip_angle_subscriber rclcpp std_msgs)

install(TARGETS
  hip_angle_publisher
  hip_angle_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

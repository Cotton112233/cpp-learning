cmake_minimum_required(VERSION 3.16)
project(mujoco_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco/mujoco-3.13.0")

add_executable(mujoco_sim_node
  src/mujoco_sim_node.cpp
  src/pd_controller.cpp
)

ament_target_dependencies(mujoco_sim_node rclcpp std_msgs ament_index_cpp)

target_include_directories(mujoco_sim_node PRIVATE
  include
  "${MUJOCO_ROOT}/include"
)

target_link_libraries(mujoco_sim_node "${MUJOCO_ROOT}/lib/libmujoco.so")

set_target_properties(mujoco_sim_node PROPERTIES
  BUILD_RPATH "${MUJOCO_ROOT}/lib"
  INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)

install(TARGETS mujoco_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()

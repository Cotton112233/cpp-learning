cmake_minimum_required(VERSION 3.16)
project(robotics_basics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco/mujoco-3.13.0")

add_executable(fk 01_forward_kinematics.cpp)
add_executable(jacobian 02_jacobian.cpp)
add_executable(dynamics 03_dynamics.cpp)

target_include_directories(dynamics PRIVATE "${MUJOCO_ROOT}/include")
target_link_libraries(dynamics PRIVATE "${MUJOCO_ROOT}/lib/libmujoco.so")

cmake_minimum_required(VERSION 3.16)

project(joint_motion_statistics)

set(CMAKE_CXX_STANDARD 17)

add_executable(
    joint_motion_statistics
    main.cpp
    joint_replay.cpp
    joint_statistics.cpp
)

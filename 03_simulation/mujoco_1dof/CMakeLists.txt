cmake_minimum_required(VERSION 3.16)

project(mujoco_1dof)

set(CMAKE_CXX_STANDARD 17)

set(MUJOCO_ROOT "$ENV{HOME}/.local/mujoco/mujoco-3.13.0")

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(mujoco_1dof
    main.cpp
    viewer.cpp
    pd_controller.cpp
)

target_include_directories(mujoco_1dof PRIVATE
    "${MUJOCO_ROOT}/include"
)

target_link_directories(mujoco_1dof PRIVATE
    "${MUJOCO_ROOT}/lib"
)

target_link_libraries(mujoco_1dof PRIVATE
    mujoco
    glfw
    OpenGL::GL
)

set_target_properties(mujoco_1dof PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
)

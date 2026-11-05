cmake_minimum_required(VERSION 3.16)
project(trajectory_planning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# The algorithm library has no ROS or MuJoCo dependencies.
add_library(trajectory_planning STATIC
  src/step_linear.cpp
  src/polynomial.cpp
  src/trapezoidal.cpp
)
target_compile_features(trajectory_planning PUBLIC cxx_std_17)
target_include_directories(trajectory_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_cli src/trajectory_cli.cpp)
target_link_libraries(trajectory_cli PRIVATE trajectory_planning)

install(TARGETS trajectory_planning
  EXPORT export_trajectory_planning
  ARCHIVE DESTINATION lib
)
install(TARGETS trajectory_cli DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_trajectory_planning HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME linear_cli
    COMMAND trajectory_cli --algorithm linear --q0 0 --qf 1 --T 4 --dt 1)
  set_tests_properties(linear_cli PROPERTIES
    PASS_REGULAR_EXPRESSION "2,0.5,0.25,0")

  add_test(NAME step_cli
    COMMAND trajectory_cli --algorithm step --q0 0 --qf 1 --T 4 --dt 1)
  set_tests_properties(step_cli PROPERTIES
    PASS_REGULAR_EXPRESSION "0,1,nan,nan")

  add_test(NAME trapezoidal_cli
    COMMAND trajectory_cli --algorithm trapezoidal --q0 0 --qf 4 --vmax 1 --amax 1 --dt 1)
  set_tests_properties(trapezoidal_cli PROPERTIES PASS_REGULAR_EXPRESSION "2,1.5,1,0")

  add_test(NAME triangular_cli
    COMMAND trajectory_cli --algorithm trapezoidal --q0 0 --qf 1 --vmax 2 --amax 1 --dt 1)
  set_tests_properties(triangular_cli PROPERTIES PASS_REGULAR_EXPRESSION "1,0.5,1,nan")

  add_test(NAME stationary_cli
    COMMAND trajectory_cli --algorithm trapezoidal --q0 2 --qf 2)
  set_tests_properties(stationary_cli PROPERTIES PASS_REGULAR_EXPRESSION "0,2,0,0")

  add_test(NAME reject_zero_amax COMMAND trajectory_cli --algorithm trapezoidal --amax 0)
  set_tests_properties(reject_zero_amax PROPERTIES WILL_FAIL TRUE)

  add_test(NAME reject_zero_dt COMMAND trajectory_cli --dt 0)
  set_tests_properties(reject_zero_dt PROPERTIES WILL_FAIL TRUE)
endif()

ament_package()

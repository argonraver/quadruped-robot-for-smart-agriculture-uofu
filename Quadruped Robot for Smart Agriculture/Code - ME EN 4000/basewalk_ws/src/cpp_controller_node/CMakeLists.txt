cmake_minimum_required(VERSION 3.8)
project(cpp_controller_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_interfaces REQUIRED)

add_executable(cpp_controller_node src/IMU_node_cli.cpp)
target_link_libraries(cpp_controller_node rclcpp)
target_link_directories(cpp_controller_node PUBLIC /opt/ros/humble/lib/)
ament_target_dependencies(cpp_controller_node control_interfaces rclcpp)

# C++ Section
include_directories(include)

add_executable(imuboard src/imufilterservice.cpp)
ament_target_dependencies(imuboard control_interfaces rclcpp)

add_executable(pid_controller src/PID_controller.cpp)
ament_target_dependencies(pid_controller control_interfaces rclcpp)

install(TARGETS
  cpp_controller_node
  imuboard
  pid_controller
  DESTINATION "lib/${PROJECT_NAME}"
)


# Python Section
install(PROGRAMS
  scripts/imureader.py
  DESTINATION "lib/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

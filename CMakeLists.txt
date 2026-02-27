cmake_minimum_required(VERSION 3.8)
project(ros2_lifecycle)

# Set C++ standard to 17 as required by ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# 1. Add executable using src/lifecycle_sensor.cpp
add_executable(lifecycle_sensor src/lifecycle_sensor.cpp)

# 2. Add dependencies for rclcpp, rclcpp_lifecycle, std_msgs
ament_target_dependencies(lifecycle_sensor
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# 3. Install the target to the DESTINATION lib/${PROJECT_NAME}
install(TARGETS
  lifecycle_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: Install the launch files or other directories if you have them
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ALWAYS KEEP THIS LINE AT THE VERY END
ament_package()

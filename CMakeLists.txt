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

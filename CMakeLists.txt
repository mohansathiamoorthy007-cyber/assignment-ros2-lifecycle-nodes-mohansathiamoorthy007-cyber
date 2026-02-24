# Add the executable target
add_executable(lifecycle_sensor src/lifecycle_sensor.cpp)

# Link ROS2 dependencies
ament_target_dependencies(lifecycle_sensor 
  rclcpp 
  rclcpp_lifecycle 
  std_msgs
)

# Install the target so it's accessible after sourcing
install(TARGETS
  lifecycle_sensor
  DESTINATION lib/${PROJECT_NAME}
)

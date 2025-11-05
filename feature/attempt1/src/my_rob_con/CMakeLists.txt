cmake_minimum_required(VERSION 3.5)
project(my_rob_con)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)  # Your custom message package

# Install Python modules
install(
  DIRECTORY my_rob_con/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Install launch files
install(
  DIRECTORY my_rob_con/launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install any other resource files (URDF, YAML, scripts)
install(
  DIRECTORY my_rob_con/task4
  DESTINATION share/${PROJECT_NAME}/task4
)

# Install cone detection nodes
install(
  DIRECTORY my_rob_con/conenode
  DESTINATION share/${PROJECT_NAME}/conenode
)

ament_package()

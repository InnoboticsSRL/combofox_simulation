cmake_minimum_required(VERSION 3.5)
project(neo_simulation2)

find_package(ament_cmake REQUIRED)

# find_package(ament_cmake REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(std_msgs REQUIRED)


install(DIRECTORY launch
  configs
  maps
  worlds
  robots
  components
  models
  DESTINATION share/${PROJECT_NAME}/)

ament_package()

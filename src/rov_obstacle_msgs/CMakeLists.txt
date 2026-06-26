cmake_minimum_required(VERSION 3.8)
project(rov_obstacle_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle2D.msg"
  "msg/Obstacle2DArray.msg"
  "msg/AvoidanceDebug.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interface_files test/test_interface_files.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()


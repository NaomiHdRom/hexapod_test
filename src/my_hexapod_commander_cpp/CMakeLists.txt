cmake_minimum_required(VERSION 3.8)
project(my_hexapod_commander_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(
  test_moveit
  rclcpp
  moveit_ros_planning_interface
)

add_executable(gait_test src/gait_test.cpp)
ament_target_dependencies(
  gait_test
  rclcpp
  moveit_ros_planning_interface
)

install(
  TARGETS 
    test_moveit
    gait_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interface REQUIRED)

###########################################################################
add_executable(ma_robot_commander src/ma_robot_commander.cpp)
ament_target_dependencies(ma_robot_commander rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface)

add_executable(my_robot_commander src/my_robot_commander.cpp)
ament_target_dependencies(my_robot_commander rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface)

add_executable(my_robot_lifecycle src/my_robot_lifecycle.cpp)
ament_target_dependencies(my_robot_lifecycle rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface)
 
install(TARGETS ma_robot_commander my_robot_commander
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

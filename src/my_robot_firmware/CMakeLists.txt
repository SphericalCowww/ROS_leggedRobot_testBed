cmake_minimum_required(VERSION 3.8)
project(my_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_workbench REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

###########################################################################
add_executable(testRaspPi5_dynamixel_u2d2_channel0_xl430 src/testRaspPi5_dynamixel_u2d2_channel0_xl430.cpp)
ament_target_dependencies(testRaspPi5_dynamixel_u2d2_channel0_xl430 rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)
add_executable(testRaspPi5_dynamixel_u2d2_leg1swing_xl430 src/testRaspPi5_dynamixel_u2d2_leg1swing_xl430.cpp)
ament_target_dependencies(testRaspPi5_dynamixel_u2d2_leg1swing_xl430 rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)
add_executable(testRaspPi5_dynamixel_u2d2_leg1swipe_xl430 src/testRaspPi5_dynamixel_u2d2_leg1swipe_xl430.cpp)
ament_target_dependencies(testRaspPi5_dynamixel_u2d2_leg1swipe_xl430 rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)
 
install(TARGETS testRaspPi5_dynamixel_u2d2_channel0_xl430 testRaspPi5_dynamixel_u2d2_leg1swing_xl430 testRaspPi5_dynamixel_u2d2_leg1swipe_xl430
    DESTINATION lib/${PROJECT_NAME}
)

###########################################################################
add_library(ma_robot_hardware_interface SHARED
  src/hardware_interface_ma_robot_dynamixel_u2d2_xl430.cpp
)
ament_target_dependencies(ma_robot_hardware_interface rclcpp rclcpp_lifecycle dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface ma_robot_hardware_interface.xml)
install(TARGETS ma_robot_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

add_library(my_robot_hardware_interface SHARED
  src/hardware_interface_my_robot_dynamixel_u2d2_xl430.cpp
)
ament_target_dependencies(my_robot_hardware_interface rclcpp rclcpp_lifecycle dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox hardware_interface pluginlib)
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)
install(TARGETS my_robot_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY   include
  DESTINATION include
) 
ament_export_libraries(
    ma_robot_hardware_interface
    my_robot_hardware_interface
)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)



ament_package()




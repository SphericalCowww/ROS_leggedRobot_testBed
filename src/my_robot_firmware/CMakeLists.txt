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
add_executable(testRaspPi5_dynamixel_opencr1_channel0_xl430 src/testRaspPi5_dynamixel_opencr1_channel0_xl430.cpp)
ament_target_dependencies(testRaspPi5_dynamixel_opencr1_channel0_xl430 rclcpp dynamixel_sdk dynamixel_workbench dynamixel_workbench_toolbox)

install(TARGETS testRaspPi5_dynamixel_opencr1_channel0_xl430
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(my_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

###########################################################################
add_executable(testRaspPi5_dynamixel_u2d2_channel0_xl430 src/testRaspPi5_dynamixel_u2d2_channel0_xl430.cpp)
ament_target_dependencies(testRaspPi5_dynamixel_u2d2_channel0_xl430 rclcpp dynamixel_sdk)

install(TARGETS testRaspPi5_dynamixel_u2d2_channel0_xl430
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

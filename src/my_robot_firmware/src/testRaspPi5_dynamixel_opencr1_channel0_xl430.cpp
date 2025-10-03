#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dynamixel_sweep_node");
    RCLCPP_INFO(node->get_logger(), "Starting dynamixel servo sweep test.");

    rclcpp::shutdown();
    return 0;
}


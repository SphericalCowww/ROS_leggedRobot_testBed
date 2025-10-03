#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dynamixel_sweep_node");
    RCLCPP_INFO(node->get_logger(), "Starting dynamixel servo sweep test.");

    rclcpp::shutdown();
    return 0;
}

// Control table address
#define ADDR_GOAL_POSITION     116
#define ADDR_PRESENT_POSITION  132

// Protocol version
#define PROTOCOL_VERSION       2.0

// Default setting
#define DXL_ID                 1
#define BAUDRATE               57600
#define DEVICENAME             "/dev/ttyUSB0"
#define MIN_POSITION           0
#define MAX_POSITION           4095
#define SWEEP_STEP             100
#define SWEEP_DELAY            1 // seconds

int main() {
    // Initialize PortHandler and PacketHandler
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (!portHandler->openPort()) {
        std::cerr << "Failed to open the port!" << std::endl;
        return -1;
    }

    // Set port baudrate
    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to set the baudrate!" << std::endl;
        return -1;
    }

    // Sweep from MIN_POSITION to MAX_POSITION
    for (int position = MIN_POSITION; position <= MAX_POSITION; position += SWEEP_STEP) {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, position);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Failed to set goal position!" << std::endl;
            continue;
        }
        std::cout << "Moving to position: " << position << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(SWEEP_DELAY));
    }

    // Sweep back from MAX_POSITION to MIN_POSITION
    for (int position = MAX_POSITION; position >= MIN_POSITION; position -= SWEEP_STEP) {
        int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, position);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Failed to set goal position!" << std::endl;
            continue;
        }
        std::cout << "Moving to position: " << position << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(SWEEP_DELAY));
    }

    // Close port
    portHandler->closePort();

    return 0;
}




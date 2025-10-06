#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

// Setting
#define PORTNAME              "/dev/ttyUSB0"
#define BAUDRATE               57600
#define DXL_ID1                11
#define DXL_ID2                12
#define DXL_ID3                13
#define MIN_POSITION           0
#define MAX_POSITION           4095
#define SWEEP_STEP             300
#define SWEEP_DELAY            1 // seconds

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dynamixel_sweep_node");
    RCLCPP_INFO(node->get_logger(), "Starting dynamixel servo sweep test.");

    DynamixelWorkbench dxl_wb;
    uint16_t model_number = 0;
    const char *log;
    bool dxl_return = false;

    dxl_return = dxl_wb.init(PORTNAME, BAUDRATE, &log);
    if (dxl_return == false) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open the port!");
        return -1;
    } else {
        RCLCPP_INFO(node->get_logger(), "Initialize with baud rate: %d", BAUDRATE);     
    }

    dxl_return = dxl_wb.ping(DXL_ID1, &model_number, &log);
    dxl_wb.ping(DXL_ID2, &model_number, &log);
    dxl_wb.ping(DXL_ID3, &model_number, &log);
    if (dxl_return == false) {
        RCLCPP_ERROR(node->get_logger(), "Failed to ping!");
        return -1;
    } else {
        RCLCPP_INFO(node->get_logger(), "Pinging id: %d, model_number : %d\n", DXL_ID1, model_number);
    } 

    dxl_return = dxl_wb.jointMode(DXL_ID1, 0, 0, &log);
    dxl_wb.jointMode(DXL_ID2, 0, 0, &log);
    dxl_wb.jointMode(DXL_ID3, 0, 0, &log);
    if (dxl_return == false) {
        RCLCPP_ERROR(node->get_logger(), "Failed join position mode!");
        return -1;
    } else {
        RCLCPP_INFO(node->get_logger(), "Joining position mode for id: %d, model_number : %d\n", DXL_ID1, model_number);
        
    }

    // Sweep from MIN_POSITION to MAX_POSITION
    for (int position = MIN_POSITION; position <= MAX_POSITION; position += SWEEP_STEP) {
        dxl_return = dxl_wb.goalPosition(DXL_ID1, (int32_t) position);
        dxl_wb.goalPosition(DXL_ID2, (int32_t) position);
        dxl_wb.goalPosition(DXL_ID3, (int32_t) position);
        if (dxl_return == false) {
            RCLCPP_WARN(node->get_logger(), "Failed to set position: %i", position);
        } else {
            RCLCPP_INFO(node->get_logger(), "Moving to position: %i", position);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Sleep for 0.5 seconds
    }
    // Sweep back from MAX_POSITION to MIN_POSITION
    for (int position = MAX_POSITION; position >= MIN_POSITION; position -= SWEEP_STEP) {
        dxl_return = dxl_wb.goalPosition(DXL_ID1, (int32_t) position);
        dxl_wb.goalPosition(DXL_ID2, (int32_t) position);
        dxl_wb.goalPosition(DXL_ID3, (int32_t) position);
        if (dxl_return == false) {
            RCLCPP_WARN(node->get_logger(), "Failed to set position: %i", position);
        } else {
            RCLCPP_INFO(node->get_logger(), "Moving to position: %i", position);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Sleep for 0.5 seconds
    }
    return 0;
}




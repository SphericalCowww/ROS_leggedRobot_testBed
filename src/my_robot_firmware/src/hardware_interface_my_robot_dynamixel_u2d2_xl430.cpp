#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_my_robot_dynamixel_u2d2_xl430.hpp"



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_namespace {
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_init
        (const hardware_interface::HardwareComponentInterfaceParams &params) 
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        node_ = std::make_shared<rclcpp::Node>("HardwareInterfaceU2D2_my_robot_node");
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_init()");
 
        dxl_return_ = dxl_wb_.init(PORT_NAME, BAUD_RATE, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open the port %s!", PORT_NAME);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Initialize with baud rate: %d", BAUD_RATE);
        } 

        servo1_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("servo1_servo1_padding"));
        servo2_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("servo2_servo2_padding"));
        servo3_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("servo3_calfFeet"));
        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::read()");
        (void) time;
        (void) period;
        if (write_first_call_ == true) {
            start_time_ = time;
            write_first_call_ = false;
        }
        rclcpp::Duration lifetime = time - start_time_;
    
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        double servo1_position = channel_read_position_(servo1_channel_); 
        double servo2_position = channel_read_position_(servo2_channel_);
        double servo3_position = channel_read_position_(servo3_channel_);
        set_state("servo1_servo1_padding/position", servo1_position);
        set_state("servo2_servo2_padding/position", servo2_position);
        set_state("servo3_calfFeet/position",       servo3_position);
        RCLCPP_INFO(node_->get_logger(), "read position (servo1, servo2, servo3): (%f, %f, %f)", 
                    servo1_position, servo2_position, servo3_position);
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::write()");
        (void) time;
        (void) period; 
        
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        double servo1_position = get_command("servo1_servo1_padding/position");
        double servo2_position = get_command("servo2_servo2_padding/position");
        double servo3_position = get_command("servo3_calfFeet/position");
        if (std::isnan(servo1_position) | std::isnan(servo2_position) | std::isnan(servo3_position)) {
            servo1_position = DXL_PI;
            servo2_position = DXL_PI;
            servo3_position = DXL_PI;
        }
        channel_set_position_(servo1_channel_, servo1_position);
        channel_set_position_(servo2_channel_, servo2_position);
        channel_set_position_(servo3_channel_, servo3_position);
        RCLCPP_INFO(node_->get_logger(), "write position (servo1, servo2, servo3): (%f, %f, %f)",
                    servo1_position, servo2_position, servo3_position);
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure()");
        (void) previous_state;
        channel_init_(servo1_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        channel_init_(servo2_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        channel_init_(servo3_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_activate()");
        (void) previous_state;
        set_state("servo1_servo1_padding/position", DXL_PI);
        set_state("servo2_servo2_padding/position", DXL_PI);
        set_state("servo3_calfFeet/position",       DXL_PI);
 
        channel_set_position_(servo1_channel_, DXL_PI);
        channel_set_position_(servo2_channel_, DXL_PI);
        channel_set_position_(servo3_channel_, DXL_PI);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_deactivate()");
        (void) previous_state;
        channel_set_position_(servo1_channel_, DXL_PI);
        channel_set_position_(servo2_channel_, DXL_PI);
        channel_set_position_(servo3_channel_, DXL_PI);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void HardwareInterfaceU2D2_my_robot::channel_init_(int channel) {
        dxl_return_ = dxl_wb_.ping(channel, &model_number_, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to ping!");
            return;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Pinging id: %d, model_number : %d\n", channel, model_number_);
        }
        // int32_t velocity = 0, int32_t acceleration = 0 => position mode
        dxl_return_ = dxl_wb_.jointMode(channel, 0, 0, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "Failed join position mode!");
            return;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Joining position mode for id: %d, model_number : %d\n", channel, 
                        model_number_);
        }
    }
    void HardwareInterfaceU2D2_my_robot::channel_set_position_(int channel, double position) {
        dxl_position_ = (int32_t) ((position + DXL_PI)*(MAX_POSITION-MIN_POSITION)/(2*DXL_PI));
        dxl_return_ = dxl_wb_.goalPosition(channel, dxl_position_); 
        if (dxl_return_ == false) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set position: %f", position);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Channel %i setting position: %f, %i", channel, position, dxl_position_);
        }
    }
    double HardwareInterfaceU2D2_my_robot::channel_read_position_(int channel) {
        dxl_return_ = dxl_wb_.itemRead(channel, "Present_Position", &dxl_position_, &log_);
        double position = (double) ((dxl_position_-(MAX_POSITION-MIN_POSITION)/2)*2*DXL_PI/(MAX_POSITION-MIN_POSITION));
        if (dxl_return_ == false) {
            RCLCPP_WARN(node_->get_logger(), "Failed to read position!");
            return -1.0;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Channel %i reading position: %f, %i", channel, position, dxl_position_);
        }
        return position;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_namespace::HardwareInterfaceU2D2_my_robot, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










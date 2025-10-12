#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "my_robot_firmware/hardware_interface_ma_robot_dynamixel_u2d2_xl430.hpp"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace ma_robot_namespace {
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_ma_robot::on_init
        (const hardware_interface::HardwareComponentInterfaceParams &params) 
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        node_ = std::make_shared<rclcpp::Node>("HardwareInterfaceU2D2_ma_robot_node");
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::on_init()");
 
        dxl_return_ = dxl_wb_.init(PORT_NAME, BAUD_RATE, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open the port %s!", PORT_NAME);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Initialize with baud rate: %d", BAUD_RATE);
        } 

        joint1_servo_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("joint1_servo_channel"));
        joint2_servo_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("joint2_servo_channel"));
        joint3_servo_channel_ = std::stoi(params.hardware_info.hardware_parameters.at("joint3_servo_channel"));
        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfaceU2D2_ma_robot::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::read()");
        (void) time;
        (void) period;
        if (write_first_call_ == true) {
            start_time_ = time;
            write_first_call_ = false;
        }
        rclcpp::Duration lifetime = time - start_time_;
    
        // see: src/my_robot_description/urdf/ma_robot.ros2_control.xacro
        double joint1_position = channel_read_position_(joint1_servo_channel_); 
        double joint2_position = channel_read_position_(joint2_servo_channel_);
        double joint3_position = channel_read_position_(joint3_servo_channel_);
        set_state("joint1/position", joint1_position);
        set_state("joint2/position", joint2_position);
        set_state("joint3/position", joint3_position);
        RCLCPP_INFO(node_->get_logger(), "read position (joint1, joint2, joint3): (%f, %f, %f)", 
                    joint1_position, joint2_position, joint3_position);
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfaceU2D2_ma_robot::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::write()");
        (void) time;
        (void) period; 
        
        // see: src/my_robot_description/urdf/ma_robot.ros2_control.xacro
        double joint1_position = get_command("joint1/position");
        double joint2_position = get_command("joint2/position");
        double joint3_position = get_command("joint3/position");
        channel_set_position_(joint1_servo_channel_, joint1_position);
        channel_set_position_(joint2_servo_channel_, joint2_position);
        channel_set_position_(joint3_servo_channel_, joint3_position);
        RCLCPP_INFO(node_->get_logger(), "write position (joint1, joint2, joint3): (%f, %f, %f)",
                    joint1_position, joint2_position, joint3_position);
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_ma_robot::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::on_configure()");
        (void) previous_state;
        channel_init_(joint1_servo_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        channel_init_(joint2_servo_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        channel_init_(joint3_servo_channel_);
        if (dxl_return_ == false) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_ma_robot::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::on_activate()");
        (void) previous_state;
        set_state("joint1/position", 0.0);
        set_state("joint2/position", 0.0);
        set_state("joint3/position", 0.0);
        set_state("joint4/position", 0.0);
        set_state("joint5/position", 0.0);
        set_state("joint6/position", 0.0);
 
        channel_set_position_(joint1_servo_channel_, 0);
        channel_set_position_(joint2_servo_channel_, 0);
        channel_set_position_(joint3_servo_channel_, 0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_ma_robot::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfaceU2D2_ma_robot::on_deactivate()");
        (void) previous_state;
        channel_set_position_(joint1_servo_channel_, 0);
        channel_set_position_(joint2_servo_channel_, 0);
        channel_set_position_(joint3_servo_channel_, 0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void HardwareInterfaceU2D2_ma_robot::channel_init_(int channel) {
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
    void HardwareInterfaceU2D2_ma_robot::channel_set_position_(int channel, double position) {
        dxl_position_ = (int32_t) (position + DXL_PI)*(MAX_POSITION-MIN_POSITION)/DXL_PI;
        dxl_return_ = dxl_wb_.goalPosition(channel, dxl_position_); 
        if (dxl_return_ == false) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set position: %f", position);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Channel %i setting position: %f", channel, position);
        }
    }
    double HardwareInterfaceU2D2_ma_robot::channel_read_position_(int channel) {
        dxl_return_ = dxl_wb_.itemRead(channel, "Present_Position", &dxl_position_, &log_);
        double position = (double) (dxl_position_ - (MAX_POSITION-MIN_POSITION)/2)*DXL_PI/(MAX_POSITION-MIN_POSITION);
        if (dxl_return_ == false) {
            RCLCPP_WARN(node_->get_logger(), "Failed to read position!");
            return -1.0;
        } else {
            RCLCPP_INFO(node_->get_logger(), "Channel %i reading position: %f", channel, position);
        }
        return position;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ma_robot_namespace::HardwareInterfaceU2D2_ma_robot, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










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
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_init()");
        } 

        dxl_return_ = dxl_wb_.init(PORT_NAME, BAUD_RATE, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open the port %s!", PORT_NAME);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            if (debug_bool == true) {
                RCLCPP_DEBUG(node_->get_logger(), "Initialize with baud rate: %d", BAUD_RATE);
            }
        } 
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        servo_channels_[0] = std::stoi(params.hardware_info.hardware_parameters.at("servo1_channel"));
        servo_channels_[1] = std::stoi(params.hardware_info.hardware_parameters.at("servo2_channel"));
        servo_channels_[2] = std::stoi(params.hardware_info.hardware_parameters.at("servo3_channel"));
        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::read()");
        }
        (void) period;
        if (write_first_call_ == true) {
            start_time_ = time;
            write_first_call_ = false;
        }
        rclcpp::Duration lifetime = time - start_time_;
    
        dxl_return_ = dxl_wb_.syncRead(handler_index_read_, servo_channels_, servo_N_, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::read(): syncRead fails");
            return hardware_interface::return_type::ERROR;
        }
        dxl_wb_.getSyncReadData(handler_index_read_, servo_channels_, servo_N_, dxl_positions_, &log_);
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_[servo_idx] = (double) dxl_positions_[servo_idx]*(2.0*DXL_PI)/(MAX_POSITION-MIN_POSITION);
        }
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        set_state("servo1_servo1_padding/position", rad_positions_[0]);
        set_state("servo2_servo2_padding/position", rad_positions_[1]);
        set_state("servo3_calfFeet/position",       rad_positions_[2]);
        if (debug_bool == true) {
            for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
                RCLCPP_DEBUG(node_->get_logger(), "Ch %i position (rad, dxl): (%f, %i)",
                             servo_channels_[servo_idx],  rad_positions_[servo_idx], dxl_positions_[servo_idx]);
            }
        }   
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfaceU2D2_my_robot::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::write()");
        }
        (void) time;
        (void) period; 
        
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        rad_positions_[0] = get_command("servo1_servo1_padding/position");
        rad_positions_[1] = get_command("servo2_servo2_padding/position");
        rad_positions_[2] = get_command("servo3_calfFeet/position");
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            if (std::isnan(rad_positions_[servo_idx]) == true){
                rad_positions_[servo_idx] = DXL_PI;
            }
            dxl_positions_[servo_idx] = (int32_t)(rad_positions_[servo_idx]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        }
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_, servo_channels_, servo_N_, dxl_positions_, 1, &log_);
        if (dxl_return_ == false) {
            RCLCPP_ERROR(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::write(): syncWrite fails");
            return hardware_interface::return_type::ERROR;
        }
        if (debug_bool == true) {
            for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
                RCLCPP_DEBUG(node_->get_logger(), "Ch %i position (rad, dxl): (%f, %i)",
                             servo_channels_[servo_idx],  rad_positions_[servo_idx], dxl_positions_[servo_idx]);
            }
        }
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure()");
        }
        (void) previous_state;
        
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            dxl_return_ = dxl_wb_.ping(servo_channels_[servo_idx], &model_number_, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure(): Failed to ping!");
                return hardware_interface::CallbackReturn::ERROR;
            } else if (debug_bool == true) {
                RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure(): "
                                                  "Pinging id: %d, model_number : %d\n", 
                                                  servo_channels_[servo_idx], model_number_);
            }
            // int32_t velocity = 0, int32_t acceleration = 0 => position mode
            dxl_return_ = dxl_wb_.jointMode(servo_channels_[servo_idx], 0, 0, &log_);
            if (dxl_return_ == false) {
                RCLCPP_ERROR(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure(): "
                                                  "Failed join position mode!");
                return hardware_interface::CallbackReturn::ERROR;
            } else if (debug_bool == true) {
                RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_configure(): "
                                                  "Joining position mode for id: %d, model_number : %d\n", 
                                                  servo_channels_[servo_idx], model_number_);
            }
        }

        dxl_wb_.addSyncReadHandler (servo_channels_[0], "Present_Position", &log_);
        dxl_wb_.addSyncWriteHandler(servo_channels_[0], "Goal_Position",    &log_);
        handler_index_read_  = dxl_wb_.getTheNumberOfSyncReadHandler()  - 1;
        handler_index_write_ = dxl_wb_.getTheNumberOfSyncWriteHandler() - 1;
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_activate()");
        }
        (void) previous_state;
       
        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_[servo_idx] = DXL_PI;
            dxl_positions_[servo_idx] = (int32_t)(rad_positions_[servo_idx]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        }
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_, servo_channels_, servo_N_, dxl_positions_, 1, &log_); 
        // see: src/my_robot_description/urdf/my_robot.ros2_control.xacro
        set_state("servo1_servo1_padding/position", rad_positions_[0]);
        set_state("servo2_servo2_padding/position", rad_positions_[1]);
        set_state("servo3_calfFeet/position",       rad_positions_[2]);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfaceU2D2_my_robot::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        if (debug_bool == true) {
            RCLCPP_DEBUG(node_->get_logger(), "HardwareInterfaceU2D2_my_robot::on_deactivate()");
        }
        (void) previous_state;

        for (uint8_t servo_idx = 0; servo_idx < servo_N_; servo_idx++) {
            rad_positions_[servo_idx] = DXL_PI;
            dxl_positions_[servo_idx] = (int32_t)(rad_positions_[servo_idx]*(MAX_POSITION-MIN_POSITION)/(2.0*DXL_PI));
        }
        dxl_return_ = dxl_wb_.syncWrite(handler_index_write_, servo_channels_, servo_N_, dxl_positions_, 1, &log_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_namespace::HardwareInterfaceU2D2_my_robot, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










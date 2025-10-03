#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_base {
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_base::on_init
        (const hardware_interface::HardwareInfo & info) 
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        node_ = std::make_shared<rclcpp::Node>("HardwareInterfacePCA9685_base_node");
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::on_init()");
 
        info_ = info;        
        pwm_controller_ = std::make_shared<PCA9685>(i2c_bus_, i2c_address_);
        right_servo_channel_ = std::stoi(info_.hardware_parameters["right_servo_channel"]);
        left_servo_channel_  = std::stoi(info_.hardware_parameters["left_servo_channel"]);
        pwm_min_microSec_ = std::stoi(info_.hardware_parameters["pwm_min_microSec"]);
        pwm_max_microSec_ = std::stoi(info_.hardware_parameters["pwm_max_microSec"]);
        microSec_to_ticks(pwm_min_microSec_, pwm_freq_);
        microSec_to_ticks(pwm_max_microSec_, pwm_freq_);

        return hardware_interface::CallbackReturn::SUCCESS;     
    }
    hardware_interface::return_type HardwareInterfacePCA9685_base::read 
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::read()");
        (void) period;
        
        if (write_first_call == true) {
            start_time = time;
            write_first_call = false;
        }
        rclcpp::Duration lifetime = time - start_time;
    
        // note: feedback not available for pca9685
        double right_velocity = get_command("base_right_wheel_joint/velocity");  
        double left_velocity  = get_command("base_left_wheel_joint/velocity"); 
        if (std::isnan(right_velocity)) {
            right_velocity = 0.0;
        }
        if (std::isnan(left_velocity)) {
            left_velocity = 0.0;
        }
        RCLCPP_INFO(node_->get_logger(), "velocity (right, left): (%lf, %lf)", right_velocity, left_velocity);
        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        set_state("base_right_wheel_joint/velocity", right_velocity);
        set_state("base_left_wheel_joint/velocity",  left_velocity);
        set_state("base_right_wheel_joint/position", 
                  get_state("base_right_wheel_joint/position") + right_velocity*period.seconds());
        set_state("base_left_wheel_joint/position",
                  get_state("base_left_wheel_joint/position")  + left_velocity*period.seconds());
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type HardwareInterfacePCA9685_base::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        //RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::write()");
        (void) time;
        (void) period; 
        
        // see: /src/my_robot_description/urdf/mobile_base.ros2_control.xacro
        pwm_controller_->setPWM(right_servo_channel_, 0, 20*get_command("base_right_wheel_joint/velocity"));
        pwm_controller_->setPWM(left_servo_channel_,  0, 20*get_command("base_left_wheel_joint/velocity")); 
        return hardware_interface::return_type::OK;
    }   
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_base::on_configure 
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::on_configure()");
        (void) previous_state;
        pwm_controller_->setPWMFreq(pwm_freq_); 
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_base::on_activate  
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::on_activate()");
        (void) previous_state;
        pwm_controller_->setPWM(right_servo_channel_, 0, min_ticks_);
        pwm_controller_->setPWM(left_servo_channel_,  0, min_ticks_);

        set_state("base_right_wheel_joint/position", 0.0);
        set_state("base_left_wheel_joint/position",  0.0);
        set_state("base_right_wheel_joint/velocity", 0.0);
        set_state("base_left_wheel_joint/velocity",  0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn HardwareInterfacePCA9685_base::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
    {
        RCLCPP_INFO(node_->get_logger(), "HardwareInterfacePCA9685_base::on_deactivate()");
        (void) previous_state;
        pwm_controller_->setPWM(right_servo_channel_, 0, 0);
        pwm_controller_->setPWM(left_servo_channel_,  0, 0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_base::HardwareInterfacePCA9685_base, hardware_interface::SystemInterface)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










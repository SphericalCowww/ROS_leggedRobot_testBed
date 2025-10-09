#ifndef HARDWARE_INTERFACE_PCA9685_ARM_HPP
#define HARDWARE_INTERFACE_PCA9685_ARM_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace ma_robot_namespace {
    class HardwareInterfaceU2D2_ma_robot: public hardware_interface::SystemInterface
    {
        public:
            // interface override
            hardware_interface::CallbackReturn 
                on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::return_type 
                read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type 
                write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            // lifecycle node override
            hardware_interface::CallbackReturn 
                on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn 
                on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn 
                on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        private:
            const char* port_name_ = "/dev/ttyUSB0";
            int baud_rate_    = 57600;
            int min_position_ = 0;
            int max_position_ = 4095;
            uint16_t model_number_ = 0;

            std::shared_ptr<rclcpp::Node> node_;
            DynamixelWorkbench dxl_wb_;
            const char *log_;
            bool dxl_return_ = false;
            int32_t dxl_position_ = 0;

            int joint1_servo_channel_;
            int joint2_servo_channel_;
            int joint3_servo_channel_;

            bool write_first_call_ = true;
            rclcpp::Time start_time_;

            void channel_init_         (int channel);
            void channel_set_position_ (int channel, int position);
            void channel_read_position_(int channel);
    };    
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

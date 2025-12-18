#ifndef HARDWARE_INTERFACE_MY_ROBOT_DYNAMIXEL_U2D2_XL430_HPP
#define HARDWARE_INTERFACE_MY_ROBOT_DYNAMIXEL_U2D2_XL430_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PORT_NAME    "/dev/ttyUSB0"
#define BAUD_RATE    57600
#define MIN_POSITION 0.0
#define MAX_POSITION 4095.0
#define DXL_PI       3.1415926

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_namespace {
    class HardwareInterfaceU2D2_my_robot: public hardware_interface::SystemInterface
    {
        public:
            // interface override
            hardware_interface::CallbackReturn 
                on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
            hardware_interface::return_type 
                read(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            hardware_interface::return_type 
                write(const rclcpp::Time & time, const rclcpp::Duration &period) override;
            // lifecycle node override
            hardware_interface::CallbackReturn 
                on_configure(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_activate(const rclcpp_lifecycle::State &previous_state) override;
            hardware_interface::CallbackReturn 
                on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        private:
            std::shared_ptr<rclcpp::Node> node_;
            DynamixelWorkbench dxl_wb_;
            uint16_t model_number_ = 0;
            const char *log_;
            bool dxl_return_ = false;
            int32_t dxl_position_ = 0;

            int joint1_servo_channel_;
            int joint4_servo_channel_;
            int joint6_servo_channel_;
            double joint2_position_;
            double joint3_position_;
            double joint5_position_;

            bool write_first_call_ = true;
            rclcpp::Time start_time_;

            void   channel_init_         (int channel);
            void   channel_set_position_ (int channel, double position);
            double channel_read_position_(int channel);
    };    
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

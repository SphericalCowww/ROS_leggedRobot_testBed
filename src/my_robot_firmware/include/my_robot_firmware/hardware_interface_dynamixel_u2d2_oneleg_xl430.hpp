#ifndef HARDWARE_INTERFACE_PCA9685_BASE_HPP
#define HARDWARE_INTERFACE_PCA9685_BASE_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "pca9685/PCA9685.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace my_robot_base {
    class HardwareInterfacePCA9685_base: public hardware_interface::SystemInterface
    {
        public:
            // interface override
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::return_type read 
                (const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write
                (const rclcpp::Time & time, const rclcpp::Duration & period) override;
            // lifecycle node override
            hardware_interface::CallbackReturn on_configure (const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate  (const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        private:
            std::shared_ptr<rclcpp::Node> node_;
            std::shared_ptr<PCA9685> pwm_controller_;
            int i2c_bus_     = 1;
            int i2c_address_ = 0x40;
            int pwm_freq_    = 50;

            int right_servo_channel_;
            int left_servo_channel_;
            int pwm_min_microSec_;
            int pwm_max_microSec_;
            int min_ticks_;
            int max_ticks_;
            int microSec_to_ticks(int pulse_microSec, int pwm_freq) {
                double pulse_sec     = pulse_microSec/1000000.0;      
                double period_sec    = 1.0/pwm_freq;  
                double ticks_per_sec = 4096/period_sec;
                return static_cast<int>(pulse_sec* ticks_per_sec);
            }

            bool write_first_call = true;
            rclcpp::Time start_time;
    };    
}

#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

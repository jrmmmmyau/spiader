#ifndef MY_BOT__ARDUINO_HARDWARE_HPP_
#define MY_BOT__ARDUINO_HARDWARE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <string>
#include <libserial/SerialPort.h>


namespace my_bot
{
    class MyBotArduinoHardware : public hardware_interface::SystemInterface
    {
        public:
            MyBotArduinoHardware()=default;
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            struct Wheel {
                double position = 0;
                double velocity = 0;
                double command = 0;
                double enc = 0;
                double prev_enc = 0;
            };
            std::string device_;
            int baud_rate_;
            int timeout_ms_;
            int enc_counts_per_rev_;
            Wheel wheel_left_;
            Wheel wheel_right_;
            LibSerial::SerialPort serial_port_;
            
    };
}

#endif  // MY_BOT__ARDUINO_HARDWARE_HPP_
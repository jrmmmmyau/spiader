#include "my_bot/arduino_hardware.hpp"
#include <sstream>
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

namespace my_bot{
    hardware_interface::CallbackReturn MyBotArduinoHardware::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        device_=info_.hardware_parameters["device"];
        timeout_ms_=std::stoi(info_.hardware_parameters["timeout_ms"]);
        enc_counts_per_rev_=std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
        
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_configure(const rclcpp_lifecycle::State & previous_state){

        try{
            serial_port_.Open(device_);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception & ERROR){ 
            return CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_activate(const rclcpp_lifecycle::State & previous_state){

        try {
            serial_port_.Write("\r");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            serial_port_.FlushIOBuffers();
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception & e) {
            return CallbackReturn::ERROR;
        }

    }

    hardware_interface::CallbackReturn MyBotArduinoHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        try{
            serial_port_.Close();
            return CallbackReturn::SUCCESS;
        }
        catch(const std::exception & ERROR){ 
            return CallbackReturn::ERROR;
        }
        
    }
    std::vector<hardware_interface::StateInterface> MyBotArduinoHardware::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_left_.velocity);
        state_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_right_.velocity);
        state_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &wheel_left_.position);
        state_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_POSITION,
            &wheel_right_.position);
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MyBotArduinoHardware::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(
            "left_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_left_.command);
        command_interfaces.emplace_back(
            "right_wheel_joint",
            hardware_interface::HW_IF_VELOCITY,
            &wheel_right_.command);
        return command_interfaces;

    }

    hardware_interface::return_type MyBotArduinoHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        std::string response;
        serial_port_.Write("e\r");
        serial_port_.ReadLine(response, '\n', timeout_ms_);
        
        std::istringstream iss(response);
        int enc_left, enc_right;
        iss >> enc_left >> enc_right;

        double delta_left = (enc_left - wheel_left_.enc) * (2 * M_PI / enc_counts_per_rev_);
        double delta_right = (enc_right - wheel_right_.enc) * (2 * M_PI / enc_counts_per_rev_);

        wheel_left_.enc = enc_left;
        wheel_right_.enc = enc_right;

        wheel_left_.position += delta_left;
        wheel_right_.position += delta_right;

        wheel_left_.velocity = delta_left / period.seconds();
        wheel_right_.velocity = delta_right / period.seconds();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MyBotArduinoHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period){
        double left=wheel_left_.command;
        double right=wheel_right_.command;
        left=left/18.64*255;
        right=right/18.64*255;
        left = std::clamp(left, -255.0, 255.0);
        right = std::clamp(right, -255.0, 255.0);
        std::ostringstream cmd;
        cmd << "m " << (int)left << " " << (int)right << "\r";
        serial_port_.Write(cmd.str());
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(
    my_bot::MyBotArduinoHardware,
    hardware_interface::SystemInterface
)
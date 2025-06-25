#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <gpiod.h>
#include <vector>
#include <string>
#include <map>

namespace gpio_hw_interface
{
  class GPIOInterface : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(GPIOInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // std::vector<double> gpio_states_;
    std::map<std::string/*name*/,double> gpio_states_;
    // std::vector<double> gpio_commands_;
    std::map<std::string/*name*/,double> gpio_commands_;
    // std::vector<gpiod_line *> gpio_lines_;
    std::map<std::string/*name*/, gpiod_line*> gpio_lines_;
    gpiod_chip *gpio_chip_;
    // double hw_state_, hw_command_;
  };
} // namespace my_gpio_hw_interface

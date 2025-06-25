#include "gpio_hw_interface/gpio_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"

namespace gpio_hw_interface
{



  hardware_interface::CallbackReturn GPIOInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("GPIOInterface"), "HELLO HELLO ------------------");

    // gpio_chip_ = gpiod_chip_open_by_name("gpiochip4"); // Pi 5 J8 header
    // if (!gpio_chip_)
    // {
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("GPIOInterface"), "Opened gpiochip4 successfully.");

    // gpio_states_.resize(info.gpios.size(), 0.0);
    // gpio_commands_.resize(info.gpios.size(), 0.0);

    // for (size_t i = 0; i < info.gpios.size(); ++i)
    // {
      // int gpio_num = 17; // std::stoi(info.joints[i].parameters["gpio"]);
      // auto *line = gpiod_chip_get_line(gpio_chip_, gpio_num);
      // gpio_lines_.push_back(line);
    // }

    RCLCPP_INFO(rclcpp::get_logger("GPIOInterface"), "HELLO HELLO ------------------finish init");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn GPIOInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    // for (size_t i = 0; i < gpio_lines_.size(); ++i)
    // {
      gpiod_line_request_output(gpio_lines_[0], "ros2_control", 0);
    // }
    RCLCPP_INFO(rclcpp::get_logger("GPIOInterface"), "HELLO on activate ------------------on activate");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn GPIOInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // for (auto *line : gpio_lines_)
    // {
    //   gpiod_line_release(line);
    // }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> GPIOInterface::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger(""), "----------export_state_interfaces-------------");
    RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[0].name.c_str());
    // RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[1].name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), std::to_string(info_.gpios.size()).c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), std::to_string(info_.gpios[0].state_interfaces.size()).c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), "-----------cc------------");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back("xxx", "vacuum", &hw_state_);
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> GPIOInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // for (size_t i = 0; i < info_.joints.size(); ++i)
    // {
    //   command_interfaces.emplace_back(
    //       hardware_interface::CommandInterface(info_.joints[i].name, "gpio_command", &gpio_commands_[i]));
    // }
    RCLCPP_INFO(rclcpp::get_logger(""), "----------export_command_interfaces-------------");
    // read the data from the urdf
    RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[0].name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[0].state_interfaces[0].name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[0].command_interfaces[0].name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), info_.gpios[0].parameters["initial_value"].c_str());
    RCLCPP_INFO(rclcpp::get_logger(""), "-----------------------");
    // prefix_name, interface_name, value_ptr)
    command_interfaces.emplace_back("xxx", "vacuum", &hw_command_);
    return command_interfaces;
  }

  hardware_interface::return_type GPIOInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < gpio_lines_.size(); ++i)
    {
      gpio_states_[i] = gpiod_line_get_value(gpio_lines_[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type GPIOInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {
    for (size_t i = 0; i < gpio_lines_.size(); ++i)
    {
      gpiod_line_set_value(gpio_lines_[i], static_cast<int>(gpio_commands_[i]));
    }
    return hardware_interface::return_type::OK;
  }

} // namespace my_gpio_hw_interface

PLUGINLIB_EXPORT_CLASS(gpio_hw_interface::GPIOInterface, hardware_interface::SystemInterface)

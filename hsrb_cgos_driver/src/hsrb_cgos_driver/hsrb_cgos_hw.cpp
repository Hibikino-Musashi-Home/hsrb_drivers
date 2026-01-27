/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <string>
#include <utility>
#include <vector>

#include "hsrb_cgos_driver/hsrb_cgos_hw.hpp"
#include "hsrb_cgos_driver/hsrb_cgos_lib.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hsrb_cgos_driver {

GpioInput::GpioInput(const std::string& name, const uint32_t pin)
  : GpioBase(pin), state_interface_(name, "state", &value_) {}

hardware_interface::StateInterface GpioInput::getStateInterface() {
  return std::move(state_interface_);
}

GpioOutput::GpioOutput(const std::string& name, const uint32_t pin)
  : GpioBase(pin), command_interface_(name, "command", &value_) {}

hardware_interface::CommandInterface GpioOutput::getCommandInterface() {
  return std::move(command_interface_);
}

void GpioInput::Read(uint32_t read_value) {
  if (read_value & bit_mask_) {
    value_ = 1.0;
  } else {
    value_ = 0.0;
  }
}

void GpioOutput::Read(uint32_t /*read_value*/) {}

void GpioInput::Write(uint32_t& /*write_value*/) {}

void GpioOutput::Write(uint32_t& write_value) {
  if (value_ != 0.0) {
    write_value |= bit_mask_;
  } else {
    write_value &= ~bit_mask_;
  }
}

hardware_interface::CallbackReturn HsrbCgosHw::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_info.gpios.empty()) {
    RCLCPP_FATAL(logger_, "No GPIOs found in hardware info.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto& gpio : hardware_info.gpios) {
    if (!gpio.parameters.count("pin") || !gpio.parameters.count("direction")) {
      RCLCPP_FATAL(logger_, "GPIO '%s' is missing required parameters.", gpio.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    try {
      const int32_t pin = std::stoi(gpio.parameters.at("pin"));
      const std::string direction = gpio.parameters.at("direction");

      if ((direction == "in" && (pin < 0 || pin > 3)) ||
          (direction == "out" && (pin < 4 || pin > 7))) {
        RCLCPP_FATAL(logger_, "Unsupported pin or direction for GPIO '%s'.", gpio.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(logger_, "GPIO: %s", gpio.name.c_str());
      if (direction == "in") {
        gpios_.push_back(std::make_shared<GpioInput>(gpio.name, pin));
      } else if (direction == "out") {
        gpios_.push_back(std::make_shared<GpioOutput>(gpio.name, pin));
      } else {
        RCLCPP_FATAL(logger_, "Invalid direction: %s", direction.c_str());
        return CallbackReturn::ERROR;
      }
    } catch (const std::exception& e) {
      RCLCPP_FATAL(logger_, "Error parsing parameters for GPIO '%s': %s", gpio.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HsrbCgosHw::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!cgos_->Initialize()) {
    // Do not cause an error even if there is no driver
    RCLCPP_WARN(logger_, "CgosLib::Initialize failed.");
    cgos_handle_ = 0;
    return CallbackReturn::SUCCESS;
  }

  if (!cgos_->BoardOpen(kCgosBoardClassDefault, 0, kCgosBoardOpenFlagsDefault, &cgos_handle_)) {
    RCLCPP_WARN(logger_, "CgosLib::CgosBordOpen failed.");
    cgos_handle_ = 0;
    return CallbackReturn::SUCCESS;
  }

  uint32_t count = 0;
  if (cgos_->GetIOCount(cgos_handle_, &count)) {
    if (count > 0) {
      cgos_unit_ = 0;
    } else {
      RCLCPP_FATAL(logger_, "Invalid Unit No");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_FATAL(logger_, "CgosLib::CgosIOCount failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HsrbCgosHw::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (cgos_handle_ == 0) {
    // Do nothing if there is no CGOS
    return hardware_interface::return_type::OK;
  }
  uint32_t read_value = 0;
  if (cgos_->IORead(cgos_handle_, cgos_unit_, &read_value)) {
    for (const auto& gpio : gpios_) {
      gpio->Read(read_value);
    }
    return hardware_interface::return_type::OK;
  }
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HsrbCgosHw::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (cgos_handle_ == 0) {
    // Do nothing if there is no CGOS
    return hardware_interface::return_type::OK;
  }
  uint32_t write_value = 0;
  for (const auto& gpio : gpios_) {
    gpio->Write(write_value);
  }

  if (!cgos_->IOWrite(cgos_handle_, cgos_unit_, write_value)) {
    RCLCPP_ERROR(logger_, "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn HsrbCgosHw::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  cgos_->BoardClose(cgos_handle_);
  cgos_->Finalize();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HsrbCgosHw::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HsrbCgosHw::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HsrbCgosHw::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto& gpio : gpios_) {
    if (auto input = std::dynamic_pointer_cast<GpioInput>(gpio)) {
      state_interfaces.push_back(input->getStateInterface());
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HsrbCgosHw::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto& gpio : gpios_) {
    if (auto output = std::dynamic_pointer_cast<GpioOutput>(gpio)) {
      command_interfaces.push_back(output->getCommandInterface());
    }
  }
  return command_interfaces;
}

}  // namespace hsrb_cgos_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  hsrb_cgos_driver::HsrbCgosHw,
  hardware_interface::SystemInterface)

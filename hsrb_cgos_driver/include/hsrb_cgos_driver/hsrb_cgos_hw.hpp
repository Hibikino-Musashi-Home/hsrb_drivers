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
#ifndef HSRB_CGOS_DRIVER_HSRB_CGOS_HW_HPP_
#define HSRB_CGOS_DRIVER_HSRB_CGOS_HW_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include "hsrb_cgos_driver/hsrb_cgos_lib.hpp"

namespace hsrb_cgos_driver {

class GpioBase {
 public:
  explicit GpioBase(const uint32_t pin)
    : value_(0.0) {
    bit_mask_ = 1 << pin;
  }
  virtual ~GpioBase() = default;
  virtual void Read(const uint32_t read_value) = 0;
  virtual void Write(uint32_t& write_value) = 0;

 protected:
  double value_;
  uint32_t bit_mask_;
};

class GpioInput : public GpioBase {
 public:
  GpioInput(const std::string& name, const uint32_t pin);
  void Read(uint32_t read_value) override;
  void Write(uint32_t& write_value) override;
  hardware_interface::StateInterface getStateInterface();

 private:
  hardware_interface::StateInterface state_interface_;
};

class GpioOutput : public GpioBase {
 public:
  GpioOutput(const std::string& name, const uint32_t pin);
  void Read(uint32_t read_value) override;
  void Write(uint32_t& write_value) override;
  hardware_interface::CommandInterface getCommandInterface();

 private:
  hardware_interface::CommandInterface command_interface_;
};

class HsrbCgosHw : public hardware_interface::SystemInterface {
 public:
  HsrbCgosHw() : cgos_(std::make_shared<HsrbCgosLib>()) {}
  explicit HsrbCgosHw(std::shared_ptr<ICgosInterface> cgos) : cgos_(cgos) {}

  virtual ~HsrbCgosHw() {}

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  std::shared_ptr<ICgosInterface> cgos_;
  uint32_t cgos_handle_;
  uint32_t cgos_unit_;
  std::vector<std::shared_ptr<GpioBase>> gpios_;
  rclcpp::Logger logger_{rclcpp::get_logger("HsrbCgosHw")};
};

}  // namespace hsrb_cgos_driver

#endif  // HSRB_CGOS_DRIVER_HSRB_CGOS_HW_HPP_

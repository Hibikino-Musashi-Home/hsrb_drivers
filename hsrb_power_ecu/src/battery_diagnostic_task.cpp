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

#include "battery_diagnostic_task.hpp"

#include <string>

#include "get_parameter.hpp"

namespace hsrb_power_ecu {

BatteryDiagnosticTask::BatteryDiagnosticTask(const rclcpp::Node::SharedPtr& node,
                                             const PowerEcuProtocol::Ptr protocol)
    : DiagnosticTask("Battery updater") {
  battery_error_threshold_ = GetPositiveParameter(node, "battery_error_threshold", 20.0);
  battery_warning_threshold_ = GetPositiveParameter(node, "battery_warning_threshold", 50.0);

  full_charge_capacity_ = ExistParamPtr<double>(protocol, "battery_total_capacity");
  remaining_charge_ = ExistParamPtr<double>(protocol, "battery_remaining_capacity");
  electric_current_ = ExistParamPtr<double>(protocol, "electric_current");
  voltage_ = ExistParamPtr<double>(protocol, "battery_voltage");
  temperature_ = ExistParamPtr<double>(protocol, "battery_temperature");
  zero_percent_detected_ = ExistParamPtr<bool>(protocol, "is_battery_0per");
  discharge_enabled_ = ExistParamPtr<bool>(protocol, "is_battery_disc");
  over_discharge_ = ExistParamPtr<bool>(protocol, "is_battery_discov");
  full_charge_ = ExistParamPtr<bool>(protocol, "is_battery_full");
  learning_enabled_ = ExistParamPtr<bool>(protocol, "is_battery_std");
  triple_parallel_ = ExistParamPtr<bool>(protocol, "is_battery_23par");
  over_charge_ = ExistParamPtr<bool>(protocol, "is_battery_crgov");
  relative_capacity_ = ExistParamPtr<double>(protocol, "battery_relative_capacity");
}

void BatteryDiagnosticTask::run(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  std::string message = "Battery Level: " + std::to_string(*relative_capacity_) + " %";
  // The power ECU indicates charging with a negative current value
  // On the other hand, in general definitions (such as sensor_msgs::msg::BatteryState), charging is indicated by a positive current value, so the sign is inverted
  if (*electric_current_ < 0) {
    // Charging
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, message);
  } else if (*relative_capacity_  > battery_warning_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, message);
  } else if (*relative_capacity_  > battery_error_threshold_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);
  }

  stat.add("full_charge_capacity", *full_charge_capacity_);
  stat.add("remaining_charge", *remaining_charge_);
  stat.add("electric_current", -*electric_current_);
  stat.add("voltage", *voltage_);
  stat.add("temperature", *temperature_);
  stat.add("zero_percent_detected", *zero_percent_detected_);
  stat.add("discharge_enabled", *discharge_enabled_);
  stat.add("over_discharge", *over_discharge_);
  stat.add("full_charge", *full_charge_);
  stat.add("learning_enabled", *learning_enabled_);
  stat.add("triple_parallel", *triple_parallel_);
  stat.add("over_charge", *over_charge_);
  stat.add("relative_capacity", *relative_capacity_);
}

}  // namespace hsrb_power_ecu

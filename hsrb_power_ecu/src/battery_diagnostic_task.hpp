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
#ifndef HSRB_POWER_ECU_BATTERY_DIAGNOSTIC_TASK_HPP_
#define HSRB_POWER_ECU_BATTERY_DIAGNOSTIC_TASK_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "power_ecu_protocol.hpp"

namespace hsrb_power_ecu {

class BatteryDiagnosticTask : public diagnostic_updater::DiagnosticTask {
 public:
  BatteryDiagnosticTask(const rclcpp::Node::SharedPtr& node, const PowerEcuProtocol::Ptr protocol);
  virtual ~BatteryDiagnosticTask() = default;

  void run(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

 private:
  double battery_error_threshold_;
  double battery_warning_threshold_;

  double* full_charge_capacity_;
  double* remaining_charge_;
  double* electric_current_;
  double* voltage_;
  double* temperature_;
  bool* zero_percent_detected_;
  bool* discharge_enabled_;
  bool* over_discharge_;
  bool* full_charge_;
  bool* learning_enabled_;
  bool* triple_parallel_;
  bool* over_charge_;
  double* relative_capacity_;
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_BATTERY_DIAGNOSTIC_TASK_HPP_

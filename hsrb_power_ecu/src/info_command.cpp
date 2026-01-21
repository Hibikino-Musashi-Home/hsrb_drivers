/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
#include <algorithm>
#include <bitset>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>

#include "get_parameter.hpp"
#include "power_ecu_protocol.hpp"

// TODO(Takeshita) DiagnosticsPublisherを実装するときにコードを移動させる
const std::map<uint32_t, std::string> kDiagnosticsErrors = {
    {0, "SPI1 communication error"},
    {1, "A/D converter (control system) error"},
    {2, "A/D converter (drive system) error"},
    {3, "A/D converter (insulation secondary side) error"},
    {4, "12Vd0 error (rise)"},
    {5, "12Vd0 error (fall)"},
    {6, "12Vd0 overcurrent error"},
    {7, "12Vd1 error (rise)"},
    {8, "12Vd1 error (fall)"},
    {9, "12Vd1 overcurrent error"},
    {10, "12Vd2 error (rise)"},
    {11, "12Vd2 error (fall)"},
    {12, "12Vd2 overcurrent error"},
    {13, "12Vd3 error (rise)"},
    {14, "12Vd3 error (fall)"},
    {15, "12Vd3 overcurrent error"},
    {16, "5Vd1 error (rise)"},
    {17, "5Vd1 error (fall)"},
    {18, "5Vd1 overcurrent error"},
    {19, "5Vd2 error (rise)"},
    {20, "5Vd2 error (fall)"},
    {21, "5Vd2 overcurrent error"},
    {22, "5Vd3 error (rise)"},
    {23, "5Vd3 error (fall)"},
    {24, "5Vd3 overcurrent error"},
    {25, "5Vd4 error (rise)"},
    {26, "5Vd4 error (fall)"},
    {27, "5Vd4 overcurrent error"},
    {28, "5Vd5 error (rise)"},
    {29, "5Vd5 error (fall)"},
    {30, "5Vd5 overcurrent error"},
    {31, "5Va error (rise)"},
    {32, "5Va error (fall)"},
    {33, "12Vo1 error (rise)"},
    {34, "12Vo1 error (fall)"},
    {35, "12Vo1 overcurrent error"},
    {36, "12Vo2 error (rise)"},
    {37, "12Vo2 error (fall)"},
    {38, "12Vo2 overcurrent error"},
    {39, "ACDC error (rise)"},
    {40, "ACDC error (fall)"},
    {41, "ACDC overcurrent error"},
    {42, "ACDC current overload Level1"},
    {43, "ACDC current overload Level2"},
    {44, "BATT voltage error (rise: charging)"},
    {45, "BATT voltage error (rise: discharging)"},
    {46, "BATT voltage error (fall)"},
    {47, "BATT discharge current error"},
    {48, "BATT charge current error"},
    {49, "PBM voltage error (rise)"},
    {50, "PBM voltage error (fall)"},
    {51, "PBM overcurrent error"},
    {52, "PBM overcurrent error Level2"},
    {53, "BATT short circuit error"},
    {54, "PBM short circuit error"},
    {55, "Pump output error"},
    {56, "12Vd1 overcurrent error"},
    {57, "12Vd2 overcurrent error"},
    {58, "12Vd3 overcurrent error"},
    {59, "12Vo1 overcurrent error"},
    {60, "12Vo2 overcurrent error"},
    {61, "5Vd1 overcurrent error"},
    {62, "5Vd2 overcurrent error"},
    {63, "5Vd3 overcurrent error"},
    {64, "5Vd4 overcurrent error"},
    {65, "5Vd5 overcurrent error"},
    {66, "PBM overcurrent error"},
    {67, "Precharge error"},
    {68, "12Vd2 fault"},
    {69, "12Vd3 fault"},
    {70, "12Vo2 fault"},
    {71, "12Vd0/Vd1/Vo1 fault"},
    {80, "Regenerative circuit error"},
    {81, "Regenerative circuit overload error"},
    {104, "SPI4 communication error"},
    {105, "I2C communication error"},
    {106, "CPU communication error"},
    {107, "A/D converter (control system) error"},
    {108, "OFF process error 12Vd2 (ACDC power)"},
    {109, "OFF process error 12Vd2 (battery)"},
    {144, "Unable to create log file"},
    {145, "Unable to create folder"},
    {146, "Log recording RTC error"},
    {147, "FAT file system error"},
    {148, "SD card not inserted"},
    {160, "SPI6 communication error"},
    {161, "s1 initial error (initial)"},
    {162, "s2 initial error (initial)"},
    {163, "s3 initial error (initial)"},
    {164, "Gyro Pitch stuck error"},
    {165, "Gyro Roll stuck error"},
    {166, "Gyro Yaw stuck error"},
    {167, "Acceleration X stuck error"},
    {168, "Acceleration Y stuck error"},
    {169, "Acceleration Z stuck error"},
    {170, "Gyro double fault"},
    {171, "Acceleration double fault"},
    {172, "Quaternion reset error"},
    {173, "S1 acceleration posture error (initial)"},
    {174, "S2 acceleration posture error (initial)"},
    {175, "S3 acceleration posture error (initial)"},
    {176, "Ambient temperature error"},
    {177, "Board overheating error 1"},
    {178, "Board overheating error 2"},
    {179, "MCU temperature error"},
    {180, "Temperature sensor comparison error"},
    {208, "Battery communication error"},
    {209, "Battery status error"},
    {210, "Battery discharge temperature error (high temperature)"},
    {211, "Battery discharge temperature error (low temperature)"},
    {212, "Battery temperature comparison error"},
    {213, "Cell charge temperature warning"},
    {224, "Initial check voltage signal ① circuit error"},
    {225, "Initial check voltage signal ② circuit error"},
    {226, "Initial check voltage signal ③ circuit error"},
    {227, "ACDC power A/D converter error"},
    {228, "ACDC power (PCA) communication error"},
    {229, "Power ECU communication error"},
    {230, "Output connector error"},
    {231, "ACDC power operation error"},
    {232, "PWR-003 power MOS error"},
    {233, "DC output overvoltage"},
    {234, "DC output voltage drop"},
    {235, "Output overcurrent"},
    {236, "Connection error"},
    {237, "Current value error"},
    {238, "Robot connection check input voltage drop"},
    {239, "Communication error"}
};

std::vector<uint32_t> ParseDiagStatus(const std::string& diag_status) {
  std::vector<uint32_t> result;
  for (auto i = 0u; i < diag_status.size(); ++i) {
    const auto bitset_value = std::bitset<4>(
        std::stoi(diag_status.substr(diag_status.size() - i - 1, 1), nullptr, 16));
    for (auto j = 0u; j < 4; ++j) {
      if (bitset_value.test(j)) {
        result.push_back(i * 4 + j);
      }
    }
  }
  return result;
}


void PrintDiagStatus(const std::string& diag_status) {
  std::cout << "diag_status: ";

  // Assuming multiples of 16bit like 256bit
  uint32_t space_count = 4;
  for (auto i = 0u; i < diag_status.size(); ++i) {
    if (space_count == 0) {
      space_count = 3;
      std::cout << " ";
    } else {
      --space_count;
    }
    std::cout << diag_status[i];
  }
  std::cout << std::endl;

  const auto error_bits = ParseDiagStatus(diag_status);
  for (const auto x : error_bits) {
    std::cout << "  " << kDiagnosticsErrors.at(x) << std::endl;
  }
}


/**
 * @brief Version check command
 */
int32_t main(int32_t argc, char** argv) {
  // Command line parsing
  std::string port_name;  //!< Port name

  {
    bool can_run = true;
    bool is_display_help = false;

    if (argc == 2) {
      // When there is one option
      if (std::string(argv[1]) == "--help") {
        // No error returned when --help is specified
        can_run = false;
        is_display_help = true;
      } else {
        port_name = argv[1];
      }
    } else {
      // Error when there are no command line arguments or more than two
      can_run = false;
    }

    if (!can_run) {
      std::cout << std::endl;
      std::cout << "This program gets the version of power ecu." << std::endl << std::endl;
      std::cout << "Usage: ros2 run hsrb_power_ecu info /dev/tty***" << std::endl << std::endl;
      if (is_display_help) {
        return 0;
      } else {
        return 1;
      }
    }
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("power_ecu_info_command");

  const auto network = boost::make_shared<hsrb_power_ecu::SerialNetwork>();
  network->Configure("device_name", port_name);
  network->Configure("receive_timeout_ms", 1);
  auto protocol = boost::make_shared<hsrb_power_ecu::PowerEcuProtocol>(network, node);

  if (!protocol->Open()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Open failed.");
    exit(EXIT_FAILURE);
  }
  if (!protocol->Init()) {
    RCLCPP_FATAL(node->get_logger(), "Protocol Init Failed");
    exit(EXIT_FAILURE);
  }
  if (protocol->Start() != boost::system::errc::success) {
    RCLCPP_FATAL(node->get_logger(), "start failed");
    exit(EXIT_FAILURE);
  }

  hsrb_power_ecu::PowerEcuVersions versions;
  if (protocol->GetPowerEcuVersions(versions)) {
    std::cout << "power ecu version : " << versions.power_ecu_version << std::endl;
    std::cout << "protocol version : " << versions.power_ecu_com_version << std::endl << std::endl;

    const auto diag_status = *hsrb_power_ecu::ExistParamPtr<std::string>(protocol, "diag_status");
    PrintDiagStatus(diag_status);
  } else {
    RCLCPP_FATAL(node->get_logger(), "GetPowerEcuVersions failed.");
    exit(EXIT_FAILURE);
  }

  protocol->Close();
  return EXIT_SUCCESS;
}

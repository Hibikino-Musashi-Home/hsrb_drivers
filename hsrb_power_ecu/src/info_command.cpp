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
    {4, "12Vd0 anomaly (rise)"},
    {5, "12Vd0 anomaly (fall)"},
    {6, "12Vd0 overcurrent anomaly"},
    {7, "12Vd1 anomaly (rise)"},
    {8, "12Vd1 anomaly (fall)"},
    {9, "12Vd1 overcurrent anomaly"},
    {10, "12Vd2 anomaly (rise)"},
    {11, "12Vd2 anomaly (fall)"},
    {12, "12Vd2 overcurrent anomaly"},
    {13, "12Vd3 anomaly (rise)"},
    {14, "12Vd3 anomaly (fall)"},
    {15, "12Vd3 overcurrent anomaly"},
    {16, "5Vd1 anomaly (rise)"},
    {17, "5Vd1 anomaly (fall)"},
    {18, "5Vd1 overcurrent anomaly"},
    {19, "5Vd2 anomaly (rise)"},
    {20, "5Vd2 anomaly (fall)"},
    {21, "5Vd2 overcurrent anomaly"},
    {22, "5Vd3 anomaly (rise)"},
    {23, "5Vd3 anomaly (fall)"},
    {24, "5Vd3 overcurrent anomaly"},
    {25, "5Vd4 anomaly (rise)"},
    {26, "5Vd4 anomaly (fall)"},
    {27, "5Vd4 overcurrent anomaly"},
    {28, "5Vd5 anomaly (rise)"},
    {29, "5Vd5 anomaly (fall)"},
    {30, "5Vd5 overcurrent anomaly"},
    {31, "5Va anomaly (rise)"},
    {32, "5Va anomaly (fall)"},
    {33, "12Vo1 anomaly (rise)"},
    {34, "12Vo1 anomaly (fall)"},
    {35, "12Vo1 overcurrent anomaly"},
    {36, "12Vo2 anomaly (rise)"},
    {37, "12Vo2 anomaly (fall)"},
    {38, "12Vo2 overcurrent anomaly"},
    {39, "ACDC anomaly (rise)"},
    {40, "ACDC anomaly (fall)"},
    {41, "ACDC overcurrent anomaly"},
    {42, "ACDC current overload Level1"},
    {43, "ACDC current overload Level2"},
    {44, "BATT voltage anomaly (rise: charge)"},
    {45, "BATT voltage anomaly (rise: discharge)"},
    {46, "BATT voltage anomaly (fall)"},
    {47, "BATT discharge current anomaly"},
    {48, "BATT charge current anomaly"},
    {49, "PBM voltage anomaly (rise)"},
    {50, "PBM voltage anomaly (fall)"},
    {51, "PBM overcurrent anomaly"},
    {52, "PBM overcurrent anomaly Level2"},
    {53, "BATT short anomaly"},
    {54, "PBM short anomaly"},
    {55, "Pump output anomaly"},
    {56, "12Vd1 overcurrent anomaly"},
    {57, "12Vd2 overcurrent anomaly"},
    {58, "12Vd3 overcurrent anomaly"},
    {59, "12Vo1 overcurrent anomaly"},
    {60, "12Vo2 overcurrent anomaly"},
    {61, "5Vd1 overcurrent anomaly"},
    {62, "5Vd2 overcurrent anomaly"},
    {63, "5Vd3 overcurrent anomaly"},
    {64, "5Vd4 overcurrent anomaly"},
    {65, "5Vd5 overcurrent anomaly"},
    {66, "PBM overcurrent anomaly"},
    {67, "Precharge anomaly"},
    {68, "12Vd2 fault"},
    {69, "12Vd3 fault"},
    {70, "12Vo2 fault"},
    {71, "12Vd0/Vd1/Vo1 fault"},
    {80, "Regenerative circuit anomaly"},
    {81, "Regenerative circuit overload anomaly"},
    {104, "SPI4 communication error"},
    {105, "I2C communication error"},
    {106, "CPU communication error"},
    {107, "A/D converter (control system) error"},
    {108, "OFF processing error 12Vd2 (ACDC power supply)"},
    {109, "OFF processing error 12Vd2 (battery)"},
    {144, "Cannot create log file"},
    {145, "Cannot create folder"},
    {146, "Log recording RTC error"},
    {147, "FAT file system error"},
    {148, "SD card not inserted"},
    {160, "SPI6 communication error"},
    {161, "s1 initial error (initial)"},
    {162, "s2 initial error (initial)"},
    {163, "s3 initial error (initial)"},
    {164, "Gyro Pitch sticking error"},
    {165, "Gyro Roll sticking error"},
    {166, "Gyro Yaw sticking error"},
    {167, "Acceleration X sticking error"},
    {168, "Acceleration Y sticking error"},
    {169, "Acceleration Z sticking error"},
    {170, "Gyro double fault"},
    {171, "Acceleration double fault"},
    {172, "Quaternion reset error"},
    {173, "S1 acceleration attitude error (initial)"},
    {174, "S2 acceleration attitude error (initial)"},
    {175, "S3 acceleration attitude error (initial)"},
    {176, "Ambient temperature anomaly"},
    {177, "Board overheating anomaly 1"},
    {178, "Board overheating anomaly 2"},
    {179, "MCU temperature anomaly"},
    {180, "Temperature sensor comparison error"},
    {208, "Battery communication error"},
    {209, "Battery status error"},
    {210, "Battery discharge temperature anomaly (high temperature)"},
    {211, "Battery discharge temperature anomaly (low temperature)"},
    {212, "Battery temperature comparison error"},
    {213, "Cell charge temperature warning"},
    {224, "Initial check voltage signal ① circuit anomaly"},
    {225, "Initial check voltage signal ② circuit anomaly"},
    {226, "Initial check voltage signal ③ circuit anomaly"},
    {227, "ACDC power supply A/D converter error"},
    {228, "ACDC power supply (PCA) communication error"},
    {229, "Power ECU communication error"},
    {230, "Output connector anomaly"},
    {231, "ACDC power supply operation anomaly"},
    {232, "PWR-003 power MOS anomaly"},
    {233, "DC output overvoltage"},
    {234, "DC output voltage drop"},
    {235, "Output overcurrent"},
    {236, "Connection anomaly"},
    {237, "Current value anomaly"},
    {238, "Robot connection confirmation input voltage drop"},
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

  // 256bit like 16bit multiples assumed
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
  // Command line analysis
  std::string port_name;  //!< Port name

  {
    bool can_run = true;
    bool is_display_help = false;

    if (argc == 2) {
      // When there is one option
      if (std::string(argv[1]) == "--help") {
        // --help specified does not return an error
        can_run = false;
        is_display_help = true;
      } else {
        port_name = argv[1];
      }
    } else {
      // Error when there are no command line arguments or when there are 2 or more
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

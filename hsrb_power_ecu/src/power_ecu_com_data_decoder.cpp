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
#include "power_ecu_com_data_decoder.hpp"

#include <cstdlib>

#include <algorithm>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include <rclcpp/rclcpp.hpp>

namespace {
const size_t kEcu1DataDecoderPacketSize = 326;        //!< Packet size of basic information packet
const char kEcu1DataDecoderPacketName[] = "ecu1_";    //!< Packet type of basic information packet
const size_t kEcu1DateLengh = 14 + 1;                 //!< Number of characters for date and time + \0
const size_t kEcu1DiagStatusLength = 65 + 1;          //!< Number of characters for diagnostic information + \0
const size_t kEcu1PowerEcuStatusFlagLength = 3 + 1;   //!< Number of characters for power ECU status + \0
const size_t kEcu1EcuStatusLength = 17 + 1;           //!< Number of characters for power ECU status + \0
const size_t kEcu1GyroStatusLength = 17 + 1;          //!< Number of characters for gyro attitude angle calculation status + /0
const size_t kEcu1TempStringLength = 9 + 1;           //!< Maximum number of characters for temporary string
const size_t kEcu2DataDecoderPacketSize = 291;        //!< Packet size of optional information packet
const char kEcu2DataDecoderPacketName[] = "ecu2_";    //!< Packet type of optional information packet
const size_t kEcu2PowerEcuVersionLength = 41 + 1;     //!< Number of characters for power ECU firmware version + \0
const size_t kEcu2PowerEcuComVersionLength = 41 + 1;  //!< Number of characters for power ECU communication structure HASH + \0
const size_t kRxackDataDecoderPacketSize = 15;        //!< Packet size of response packet
const char kRxackDataDecoderPacketName[] = "rxack";   //!< Packet type of response packet
const size_t kVerDataDecoderPacketSize = 95;          //!< Packet size of version information packet
const char kVerDataDecoderPacketName[] = "ver__";     //!< Packet type of version information packet
}  // anonymous namespace

namespace hsrb_power_ecu {
/**
 * @brief Constructor
 * @param[in] packet_data Packet data
 */
PowerEcuComEcu1DataDecoder::PowerEcuComEcu1DataDecoder()
    : IPowerEcuComDataDecoder(kEcu1DataDecoderPacketSize, kEcu1DataDecoderPacketName), packet_out_() {
  packet_out_.ecu1_date.reserve(kEcu1DateLengh);
  packet_raw_data_.date.reserve(kEcu1DateLengh);
  packet_out_.diag_status.reserve(kEcu1DiagStatusLength);
  packet_raw_data_.diag_status.reserve(kEcu1DiagStatusLength);
  packet_out_.power_ecu_status_flag.reserve(kEcu1PowerEcuStatusFlagLength);
  packet_raw_data_.power_ecu_status_flag.reserve(kEcu1PowerEcuStatusFlagLength);
  packet_out_.gyro_status.reserve(kEcu1GyroStatusLength);
  packet_raw_data_.gyro_status.reserve(kEcu1GyroStatusLength);
  packet_raw_data_.power_ecu_status.reserve(kEcu1EcuStatusLength);
  temp_str.reserve(kEcu1TempStringLength);

  // Create decode instruction list
  // Push back from the beginning of packet data
  // First argument: Element decode class and type specification after decoding
  // Second argument: Assignment destination after decoding
  // Third argument: Number of digits
  //!< Timestamp           Decimal 10 digits [ms]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint32_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint32_t>(packet_raw_data_.time_stamp, 10)));
  //!< Date and time       Characters (YYYYMMDDhhmmss)
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.date, 14)));
  //!< Power ECU status    Hexadecimal 2 digits      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.power_ecu_status_flag, 2 + 1)));
  //!< Power ECU state     Hexadecimal 16 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.power_ecu_status, 16 + 1)));
  //!< Diagnostic information Hexadecimal 64 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.diag_status, 64 + 1)));
  //!< Total battery capacity Signed decimal 5 digits  [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_total_capacity, 5)));
  //!< Remaining battery capacity Signed decimal 5 digits  [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_remaining_capacity, 5)));
  //!< Current value       Signed decimal 5 digits  [mA]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_raw_data_.electric_current, 5)));
  //!< Battery voltage     Signed decimal 5 digits  [mV]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_raw_data_.battery_voltage, 5)));
  //!< Battery temperature Signed decimal 3 digits  [℃]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int8_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int8_t>(packet_raw_data_.battery_temperature, 3)));
  //!< Battery status flag Hexadecimal 4 digits      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint16_t>(packet_raw_data_.battery_state_flag, 4)));
  //!< Initial battery learning capacity Decimal 5 digits      [mAh]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint16_t>(packet_raw_data_.battery_initial_learning_capacity, 5)));
  //!< Battery abnormal status Hexadecimal 4 digits -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint16_t>(packet_raw_data_.battery_error_status, 4)));
  //!< Relative capacity   Decimal 3 digits [%]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintDecoder<uint8_t>(packet_raw_data_.battery_relative_capacity, 3)));
  //!< Proximity, bumper sensor status Hexadecimal 2 digits      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_raw_data_.bumper_status, 2)));
  //!< Gyro attitude angle calculation status Hexadecimal 16 digits uint8*8
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_raw_data_.gyro_status, 16 + 1)));
  //!< quaternion_t        Signed decimal 10 digits x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_t, 10)));
  //!< quaternion_x        Signed decimal 10 digits x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_x, 10)));
  //!< quaternion_y        Signed decimal 10 digits x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_y, 10)));
  //!< quaternion_z        Signed decimal 10 digits x10^-1
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.quaternion_z, 10)));
  //!< Angular velocity x  Signed decimal 10 digits x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_x, 10)));
  //!< Angular velocity y  Signed decimal 10 digits x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_y, 10)));
  //!< Angular velocity z  Signed decimal 10 digits x10^-2[rad/s]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.angular_velocity_z, 10)));
  //!< Acceleration x      Signed decimal 10 digits x10^-2[m/s^2]
  element_decoder_list_.push_back(
      boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(new hsrb_power_ecu::ElementIntDecoder<int32_t>(
          packet_raw_data_.acceleration_x, 10)));  //!< Acceleration y      Signed decimal 10 digits x10^-2[m/s^2]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.acceleration_y, 10)));
  //!< Acceleration z      Signed decimal 10 digits x10^-2[m/s^2]
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int32_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int32_t>(packet_raw_data_.acceleration_z, 10)));
  //!< Automatic charging status Hexadecimal 8 digits      -
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_raw_data_.charger_state, 2)));

  parameter_map_.Register("time_stamp", &packet_out_.time_stamp);
  parameter_map_.Register("ecu1_date", &packet_out_.ecu1_date);
  parameter_map_.Register("power_ecu_status_flag", &packet_out_.power_ecu_status_flag);
  parameter_map_.Register("diag_status", &packet_out_.diag_status);
  parameter_map_.Register("battery_total_capacity", &packet_out_.battery_total_capacity);
  parameter_map_.Register("battery_remaining_capacity", &packet_out_.battery_remaining_capacity);
  parameter_map_.Register("electric_current", &packet_out_.electric_current);
  parameter_map_.Register("battery_voltage", &packet_out_.battery_voltage);
  parameter_map_.Register("battery_temperature", &packet_out_.battery_temperature);
  parameter_map_.Register("is_battery_crgov", &packet_out_.is_battery_crgov);
  parameter_map_.Register("is_battery_23par", &packet_out_.is_battery_23par);
  parameter_map_.Register("is_battery_std", &packet_out_.is_battery_std);
  parameter_map_.Register("is_battery_full", &packet_out_.is_battery_full);
  parameter_map_.Register("is_battery_discov", &packet_out_.is_battery_discov);
  parameter_map_.Register("is_battery_chg", &packet_out_.is_battery_chg);
  parameter_map_.Register("is_battery_disc", &packet_out_.is_battery_disc);
  parameter_map_.Register("is_battery_0per", &packet_out_.is_battery_0per);
  parameter_map_.Register("is_battery_45par", &packet_out_.is_battery_45par);
  parameter_map_.Register("is_battery_sel", &packet_out_.is_battery_sel);
  parameter_map_.Register("is_battery_bal", &packet_out_.is_battery_bal);
  parameter_map_.Register("battery_initial_learning_capacity", &packet_out_.battery_initial_learning_capacity);
  parameter_map_.Register("battery_error_status", &packet_out_.battery_error_status);
  parameter_map_.Register("battery_relative_capacity", &packet_out_.battery_relative_capacity);
  parameter_map_.Register("power_ecu_internal_state", &packet_out_.power_ecu_internal_state);
  parameter_map_.Register("is_powerecu_bat_stat", &packet_out_.is_powerecu_bat_stat);
  parameter_map_.Register("is_powerecu_sw_kinoko", &packet_out_.is_powerecu_sw_kinoko);
  parameter_map_.Register("is_powerecu_sw_pwr", &packet_out_.is_powerecu_sw_pwr);
  parameter_map_.Register("is_powerecu_sw_drv", &packet_out_.is_powerecu_sw_drv);
  parameter_map_.Register("is_powerecu_sw_latch", &packet_out_.is_powerecu_sw_latch);
  parameter_map_.Register("is_powerecu_sw_w_sel", &packet_out_.is_powerecu_sw_w_sel);
  parameter_map_.Register("is_powerecu_sw_w_stop", &packet_out_.is_powerecu_sw_w_stop);
  parameter_map_.Register("is_bumper_bumper2", &packet_out_.is_bumper_bumper2);
  parameter_map_.Register("is_bumper_bumper1", &packet_out_.is_bumper_bumper1);
  parameter_map_.Register("is_bumper_prox5", &packet_out_.is_bumper_prox5);
  parameter_map_.Register("is_bumper_prox4", &packet_out_.is_bumper_prox4);
  parameter_map_.Register("is_bumper_prox3", &packet_out_.is_bumper_prox3);
  parameter_map_.Register("is_bumper_prox2", &packet_out_.is_bumper_prox2);
  parameter_map_.Register("is_bumper_prox1", &packet_out_.is_bumper_prox1);
  parameter_map_.Register("gyro_status", &packet_out_.gyro_status);
  parameter_map_.Register("imu_quaternions", &packet_out_.imu_quaternions);
  parameter_map_.Register("imu_angular_velocities", &packet_out_.imu_angular_velocities);
  parameter_map_.Register("imu_accelerations", &packet_out_.imu_accelerations);
  parameter_map_.Register("charger_state", &packet_out_.charger_state);
}

/**
 * @brief Post-decoding process
 * @return True on success
 */
bool PowerEcuComEcu1DataDecoder::Update() {
  //!< Timestamp           Decimal 10 digits [ms]
  packet_out_.time_stamp = packet_raw_data_.time_stamp;
  //!< Date and time       Characters (YYYYMMDDhhmmss)
  if (packet_raw_data_.date.size() <= packet_out_.ecu1_date.capacity()) {
    packet_out_.ecu1_date.clear();
    std::copy(packet_raw_data_.date.begin(), packet_raw_data_.date.end(), std::back_inserter(packet_out_.ecu1_date));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu1 date have not enough buffer.");
  }
  //!< Power ECU status    Hexadecimal 2 digits      -
  if ((packet_raw_data_.power_ecu_status_flag.size() - 1) <= packet_out_.power_ecu_status_flag.capacity()) {
    packet_out_.power_ecu_status_flag.clear();
    std::copy(packet_raw_data_.power_ecu_status_flag.begin() + 1, packet_raw_data_.power_ecu_status_flag.end(),
              std::back_inserter(packet_out_.power_ecu_status_flag));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu1 power_ecu_status have not enough buffer.");
  }

  // Diagnostic information Diagnostic
  // diag[0]      DIAG-P-001
  // diag[1]      DIAG-P-002
  // diag[2]      DIAG-P-003
  // diag[3]      DIAG-P-004
  // diag[4]      DIAG-P-005
  // diag[5]      DIAG-P-006
  // diag[6]      DIAG-P-007
  // diag[7]      DIAG-P-008
  //
  // diag[8]      DIAG-P-009
  // diag[9]      DIAG-P-010
  // diag[10]     DIAG-P-011
  // diag[11]     DIAG-P-012
  // diag[12]     DIAG-P-013
  // diag[13]     DIAG-P-014
  // diag[14]     DIAG-P-015
  // diag[15]     DIAG-P-016
  //
  // diag[16]     DIAG-P-017
  // diag[17]     DIAG-P-018
  // diag[18]     DIAG-P-019
  // diag[19]     DIAG-P-020
  // diag[20]     DIAG-P-021
  // diag[21]     DIAG-P-022
  // diag[22]     DIAG-P-023
  // diag[23]     DIAG-P-024
  //
  // diag[24]     DIAG-P-025
  // diag[25]     DIAG-P-026
  // diag[26]     DIAG-P-027
  // diag[27]     DIAG-P-028
  // diag[28]     DIAG-P-029
  // diag[29]     DIAG-P-030
  // diag[30]     DIAG-P-031
  // diag[31]     DIAG-P-032
  //
  // diag[32]     DIAG-P-033
  // diag[33]     DIAG-P-034
  // diag[34]     DIAG-P-035
  // diag[35]     DIAG-P-036
  // diag[36]     DIAG-P-037
  // diag[37]     DIAG-P-038
  // diag[38]     DIAG-P-039
  // diag[39]     DIAG-P-040
  //
  // diag[40]     DIAG-P-041
  // diag[41]     DIAG-E-001
  // diag[42]     DIAG-E-002
  // diag[43]     DIAG-C-001
  // diag[44]     DIAG-C-002
  // diag[45]     DIAG-C-003
  // diag[46]     DIAG-V-001
  // diag[47]
  //
  // diag[48]
  // diag[49]
  // diag[50]     DIAG-I-001
  // diag[51]     DIAG-I-002
  // diag[52]     DIAG-B-001
  // diag[53]     DIAG-B-002
  if ((packet_raw_data_.diag_status.size() - 1) <= packet_out_.diag_status.capacity()) {
    packet_out_.diag_status.clear();
    std::copy(packet_raw_data_.diag_status.begin() + 1, packet_raw_data_.diag_status.end(),
              std::back_inserter(packet_out_.diag_status));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("power_ecu_com_data_decoder"), "ecu2 diag_status have not enough buffer.");
  }

  // ①Total battery capacity   2byte 1mAH 00000～65535  0～65535mAH
  packet_out_.battery_total_capacity = static_cast<double>(packet_raw_data_.battery_total_capacity) * 1e-3;
  // ②Remaining battery capacity   2byte 1mAH 00000～65535  0～65535mAH
  packet_out_.battery_remaining_capacity = static_cast<double>(packet_raw_data_.battery_remaining_capacity) * 1e-3;
  // ③Battery current value   2byte 2mA  -65536～65534 -65536～65534mA ( - :Charging, +:Discharging)
  packet_out_.electric_current = static_cast<double>(packet_raw_data_.electric_current) * 1e-3;
  // ④Battery voltage value   2byte 1mV  00000～65535  0～65535mV
  packet_out_.battery_voltage = static_cast<double>(packet_raw_data_.battery_voltage) * 1e-3;
  // ⑤Battery temperature 1byte --   -128～127     -128～127℃
  packet_out_.battery_temperature = static_cast<double>(packet_raw_data_.battery_temperature);
  // ⑦Battery: Status flag
  // 1st byte
  // Bit Symbol Bit name Function
  // bit7   CRGOV    Overcharge   0:Not overcharged     1:Overcharged
  // bit6   23PAR    Parallel number   0:2 parallel            1:3 parallel
  // bit5   STD      Learning permission 0:Learning prohibited       1:Learning permitted
  // bit4   FULL     Full charge   0:Not fully charged 1:Fully charged
  // bit3   DISCOV   Overdischarge   0:Not overdischarged     1:Overdischarged
  // bit2   CHG      Charging permission 0:Charging stopped       1:Charging permitted
  // bit1   DISC     Discharge permission 0:Discharge stopped       1:Discharge permitted
  // bit0   0PER     0% detection   0:Not in 0% detection state 1:In 0% detection state
  //
  // 2nd byte
  // Bit     Symbol Bit name               Function
  // bit7～bit3 reserve  Reserved bits             0:Fixed value
  // bit2       45PAR    Parallel number                 0:4 parallel 1:5 parallel
  // bit1       SEL      Minimum cell voltage 0% detection state 0:Not in minimum cell voltage 0% detection state
  // 1:In minimum cell voltage 0% detection state
  // bit0       BAL      Cell balance disruption       0:No cell balance disruption
  // 1:Cell balance disruption
  packet_out_.is_battery_45par = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 2))) > 0);
  packet_out_.is_battery_sel = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 1))) > 0);
  packet_out_.is_battery_bal = ((packet_raw_data_.battery_state_flag & (0x1 << (8 + 0))) > 0);
  packet_out_.is_battery_crgov = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 7))) > 0);
  packet_out_.is_battery_23par = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 6))) > 0);
  packet_out_.is_battery_std = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 5))) > 0);
  packet_out_.is_battery_full = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 4))) > 0);
  packet_out_.is_battery_discov = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 3))) > 0);
  packet_out_.is_battery_chg = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 2))) > 0);
  packet_out_.is_battery_disc = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 1))) > 0);
  packet_out_.is_battery_0per = ((packet_raw_data_.battery_state_flag & (0x1 << (0 + 0))) > 0);
  //!< Initial battery learning capacity Decimal 5 digits      [mAh]
  packet_out_.battery_initial_learning_capacity = packet_raw_data_.battery_initial_learning_capacity;
  //!< Battery abnormal status Hexadecimal 4 digits - Content undefined in specifications
  packet_out_.battery_error_status = packet_raw_data_.battery_error_status;
  //!< Relative capacity   Decimal 3 digits [%]
  packet_out_.battery_relative_capacity = static_cast<double>(packet_raw_data_.battery_relative_capacity);

  // Power ECU state
  // Bit | Symbol        | Bit name                         | Function
  // 24～63 | reserve         | Reserved bits                       | 0:Fixed value
  // 16～23 | ECU_PROG_STATUS | "Number of S** in 4.5.3 Power ECU internal state" | 0～12
  // 7～15  | reserve         | Reserved bits                       |
  // 6      | BAT_STAT        | Battery charging state                 | 0:Not charging 1:Charging
  // 5      | SW_KINOKO       | Wired emergency stop SW                   | 0:Not pressed 1:Pressed
  // 4      | SW_PWR          | Power (Prius SW)                | 0:Not pressed 1:Pressed
  // 3      | SW_DRV          | Drive SW                           | 0:Drive system output 1:Drive system not output
  // 2      | SW_LATCH        | Latch release SW                     | 0:Not pressed 1:Pressed
  // 1      | SW_W_SEL        | Wireless switch SW                   | 0:Wireless emergency stop enabled 1:Wireless emergency stop disabled
  // 0      | SW_W_STOP       | Wireless emergency stop SW                   | 0:Not pressed 1:Pressed
  temp_str.clear();
  std::copy(packet_raw_data_.power_ecu_status.end() - 6, packet_raw_data_.power_ecu_status.end() - 4,
            std::back_inserter(temp_str));
  packet_out_.power_ecu_internal_state = std::strtol(temp_str.c_str(), NULL, 16);
  temp_str.clear();
  std::copy(packet_raw_data_.power_ecu_status.end() - 2, packet_raw_data_.power_ecu_status.end(),
            std::back_inserter(temp_str));
  uint8_t bits = std::strtol(temp_str.c_str(), NULL, 16);
  packet_out_.is_powerecu_bat_stat = (bits & (0x1 << 6)) != 0;
  // The output from the ECU for the wired emergency stop SW is reversed for 0 and 1
  // Since no changes are made on the ECU side, the upper software will handle it by reversing the judgment
  packet_out_.is_powerecu_sw_kinoko = (bits & (0x1 << 5)) == 0;
  packet_out_.is_powerecu_sw_pwr = (bits & (0x1 << 4)) != 0;
  // Ideally, is_powerecu_sw_drv should output whether the drive power is stopped
  // As of March 2018, the same state as is_powerecu_sw_kinoko is being output
  // Therefore, it cannot be treated as correct information.
  // Therefore, the output state of the drive power is determined separately by the internal state of the ECU.
  // TODO(T.Nishino) 電源ECUのファームウェアが修正された段階で修正する
  packet_out_.is_powerecu_sw_drv = (packet_out_.power_ecu_internal_state != 9);
  packet_out_.is_powerecu_sw_latch = (bits & (0x1 << 2)) != 0;
  packet_out_.is_powerecu_sw_w_sel = (bits & (0x1 << 1)) != 0;
  // The output from the ECU for the wireless emergency stop SW is reversed for 0 and 1
  // Since no changes are made on the ECU side, the upper software will handle it by reversing the judgment
  packet_out_.is_powerecu_sw_w_stop = (bits & (0x1 << 0)) == 0;

  // ⑨Power board: Proximity sensor, bumper sensor status
  // Bit Symbol Bit name              Function
  // bit7   reserve  Reserved bits            0:Fixed value
  // bit6   BUMPER2  Bumper sensor status 2     0:No contact   1:Contact
  // bit5   BUMPER1  Bumper sensor status 1     0:No contact   1:Contact
  // bit4   PROX5    Proximity sensor latch status 5 0:No proximity 1:Proximity
  // bit3   PROX4    Proximity sensor latch status 4 0:No proximity 1:Proximity
  // bit2   PROX3    Proximity sensor latch status 3 0:No proximity 1:Proximity
  // bit1   PROX2    Proximity sensor latch status 2 0:No proximity 1:Proximity
  // bit0   PROX1    Proximity sensor latch status 1 0:No proximity 1:Proximity
  packet_out_.is_bumper_bumper2 = ((packet_raw_data_.bumper_status & (0x1 << 6)) > 0);
  packet_out_.is_bumper_bumper1 = ((packet_raw_data_.bumper_status & (0x1 << 5)) > 0);
  packet_out_.is_bumper_prox5 = ((packet_raw_data_.bumper_status & (0x1 << 4)) > 0);
  packet_out_.is_bumper_prox4 = ((packet_raw_data_.bumper_status & (0x1 << 3)) > 0);
  packet_out_.is_bumper_prox3 = ((packet_raw_data_.bumper_status & (0x1 << 2)) > 0);
  packet_out_.is_bumper_prox2 = ((packet_raw_data_.bumper_status & (0x1 << 1)) > 0);
  packet_out_.is_bumper_prox1 = ((packet_raw_data_.bumper_status & (0x1 << 0)) > 0);

  //!< Gyro attitude angle calculation status Hexadecimal 16 digits
  packet_out_.gyro_status.clear();
  std::copy(packet_raw_data_.gyro_status.begin() + 1, packet_raw_data_.gyro_status.end(),
            std::back_inserter(packet_out_.gyro_status));

  //!< quaternion x,y,z,t -1.0~1.0
  packet_out_.imu_quaternions[0] = packet_raw_data_.quaternion_x * 1e-9;  //!< quaternion_x        Signed decimal 10 digits x10^-1
  packet_out_.imu_quaternions[1] = packet_raw_data_.quaternion_y * 1e-9;  //!< quaternion_y        Signed decimal 10 digits x10^-1
  packet_out_.imu_quaternions[2] = packet_raw_data_.quaternion_z * 1e-9;  //!< quaternion_z        Signed decimal 10 digits x10^-1
  packet_out_.imu_quaternions[3] = packet_raw_data_.quaternion_t * 1e-9;  //!< quaternion_t        Signed decimal 10 digits x10^-1

  //!< Angular velocity x,y,z [rad/s]
  packet_out_.imu_angular_velocities[0] =
      packet_raw_data_.angular_velocity_x * 1e-8;  //!< Angular velocity x  Signed decimal 10 digits x10^-2[rad/s]
  packet_out_.imu_angular_velocities[1] =
      packet_raw_data_.angular_velocity_y * 1e-8;  //!< Angular velocity y  Signed decimal 10 digits x10^-2[rad/s]
  packet_out_.imu_angular_velocities[2] =
      packet_raw_data_.angular_velocity_z * 1e-8;  //!< Angular velocity z  Signed decimal 10 digits x10^-2[rad/s]

  //!< Acceleration x,y,z [m/s^2]
  packet_out_.imu_accelerations[0] =
      packet_raw_data_.acceleration_x * 1e-8;  //!< Acceleration x      Signed decimal 10 digits x10^-2[m/s^2]
  packet_out_.imu_accelerations[1] =
      packet_raw_data_.acceleration_y * 1e-8;  //!< Acceleration y      Signed decimal 10 digits x10^-2[m/s^2]
  packet_out_.imu_accelerations[2] =
      packet_raw_data_.acceleration_z * 1e-8;  //!< Acceleration z      Signed decimal 10 digits x10^-2[m/s^2]

  //!< Automatic charging status
  packet_out_.charger_state = packet_raw_data_.charger_state;

  return true;
}

/**
 * @brief Constructor
 * @param[in] packet_data Packet data
 */
PowerEcuComEcu2DataDecoder::PowerEcuComEcu2DataDecoder()
    : IPowerEcuComDataDecoder(kEcu2DataDecoderPacketSize, kEcu2DataDecoderPacketName), packet_data_() {
  // std::string* date;     //!<  Date and time (YYYYMMDDhhmmss) Characters
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(packet_data_.ecu2_date, 14)));
  // uint16_t* d12V_D0_V;   //!<  12Vd0 voltage        [mV] Signed 1 digit + Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D0_V, 5)));
  // int16_t* d12V_D0_A;    //!<  12Vd0 current        [mA] Signed 1 digit + Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D0_A, 5)));
  // uint16_t* d12V_D1_V;   //!<  12Vd1 voltage        [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D1_V, 5)));
  // int16_t* d12V_D1_A;    //!<  12Vd1 current        [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D1_A, 5)));
  // uint16_t* d12V_D2_V;   //!<  12Vd2 voltage        [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D2_V, 5)));
  // int16_t* d12V_D2_A;    //!<  12Vd2 current        [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D2_A, 5)));
  // uint16_t* d12V_D3_V;   //!<  12Vd3 voltage        [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_D3_V, 5)));
  // int16_t* d12V_D3_A;    //!<  12Vd3 current        [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_D3_A, 5)));
  // uint16_t* d12V_O1_V;   //!<  12Vo1 voltage        [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_O1_V, 5)));
  // int16_t* d12V_O1_A;    //!<  12Vo1 current        [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_O1_A, 5)));
  // uint16_t* d12V_O2_V;   //!<  12Vo2 voltage        [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d12V_O2_V, 5)));
  // int16_t* d12V_O2_A;    //!<  12Vo2 current        [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d12V_O2_A, 5)));
  // uint16_t* d5VA_V;      //!<  5Va voltage          [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VA_V, 5)));
  // uint16_t* d5VD1_V;     //!<  5Vd1 voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD1_V, 5)));
  // int16_t* d5VD1_A;      //!<  5Vd1 current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD1_A, 5)));
  // uint16_t* d5VD2_V;     //!<  5Vd2 voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD2_V, 5)));
  // int16_t* d5VD2_A;      //!<  5Vd2 current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD2_A, 5)));
  // uint16_t* d5VD3_V;     //!<  5Vd3 voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD3_V, 5)));
  // int16_t* d5VD3_A;      //!<  5Vd3 current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD3_A, 5)));
  // uint16_t* d5VD4_V;     //!<  5Vd4 voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD4_V, 5)));
  // int16_t* d5VD4_A;      //!<  5Vd4 current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD4_A, 5)));
  // uint16_t* d5VD5_V;     //!<  5Vd5 voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d5VD5_V, 5)));
  // int16_t* d5VD5_A;      //!<  5Vd5 current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.d5VD5_A, 5)));
  // int16_t* Chgsense;     //!<  Automatic sequential power insertion/removal terminal voltage [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.Chgsense, 5)));
  // uint16_t* d2V5VDA1_V;  //!<  2.5Va1 voltage (A/D1) [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d2V5VDA1_V, 5)));
  // uint16_t* d2V5VDA2_V;  //!<  2.5Va2 voltage (A/D2) [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.d2V5VDA2_V, 5)));
  // uint16_t* ACDC_V;      //!<  ACDC voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.ACDC_V, 5)));
  // int16_t* ADCD_A;       //!<  ACDC current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ADCD_A, 5)));
  // uint16_t* BATT_V;      //!<  BATT voltage         [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.BATT_V, 5)));
  // int16_t* BATT_A;       //!<  BATT current         [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.BATT_A, 5)));
  // int16_t* BATT_A2;      //!<  BATT current 2        [10mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.BATT_A2, 5)));
  // uint16_t* PBM_V;       //!<  PBM voltage          [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.PBM_V, 5)));
  // int16_t* PBM_A;        //!<  PBM current          [mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.PBM_A, 5)));
  // int16_t* PBM_A2;       //!<  PBM current 2        [10mA] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.PBM_A2, 5)));
  // uint16_t* PUMP_V;      //!<  Pump sensor voltage [mV] Signed 1 digit Decimal 5 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<uint16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<uint16_t>(packet_data_.PUMP_V, 5)));
  // int8_t* ECU_TEMP;      //!<  Power ECU temperature     [℃] Signed 1 digit Decimal 3 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP, 3)));
  // int8_t* ECU_TEMP1;     //!< Power ECU temperature 1     [℃] Signed 1 digit Decimal 3 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP1, 3)));
  // int8_t* ECU_TEMP2;     //!< Power ECU temperature 2     [℃] Signed 1 digit Decimal 3 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP2, 3)));
  // int8_t* ECU_TEMP3;     //!< Power ECU temperature 3     [℃] Signed 1 digit Decimal 3 digits
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementIntDecoder<int16_t> >(
      new hsrb_power_ecu::ElementIntDecoder<int16_t>(packet_data_.ECU_TEMP3, 3)));

  parameter_map_.Register("ecu2_date", &packet_data_.ecu2_date);
  parameter_map_.Register("d12V_D0_V", &packet_data_.d12V_D0_V);
  parameter_map_.Register("d12V_D0_A", &packet_data_.d12V_D0_A);
  parameter_map_.Register("d12V_D1_V", &packet_data_.d12V_D1_V);
  parameter_map_.Register("d12V_D1_A", &packet_data_.d12V_D1_A);
  parameter_map_.Register("d12V_D2_V", &packet_data_.d12V_D2_V);
  parameter_map_.Register("d12V_D2_A", &packet_data_.d12V_D2_A);
  parameter_map_.Register("d12V_D3_V", &packet_data_.d12V_D3_V);
  parameter_map_.Register("d12V_D3_A", &packet_data_.d12V_D3_A);
  parameter_map_.Register("d12V_O1_V", &packet_data_.d12V_O1_V);
  parameter_map_.Register("d12V_O1_A", &packet_data_.d12V_O1_A);
  parameter_map_.Register("d12V_O2_V", &packet_data_.d12V_O2_V);
  parameter_map_.Register("d12V_O2_A", &packet_data_.d12V_O2_A);
  parameter_map_.Register("d5VA_V", &packet_data_.d5VA_V);
  parameter_map_.Register("d5VD1_V", &packet_data_.d5VD1_V);
  parameter_map_.Register("d5VD1_A", &packet_data_.d5VD1_A);
  parameter_map_.Register("d5VD2_V", &packet_data_.d5VD2_V);
  parameter_map_.Register("d5VD2_A", &packet_data_.d5VD2_A);
  parameter_map_.Register("d5VD3_V", &packet_data_.d5VD3_V);
  parameter_map_.Register("d5VD3_A", &packet_data_.d5VD3_A);
  parameter_map_.Register("d5VD4_V", &packet_data_.d5VD4_V);
  parameter_map_.Register("d5VD4_A", &packet_data_.d5VD4_A);
  parameter_map_.Register("d5VD5_V", &packet_data_.d5VD5_V);
  parameter_map_.Register("d5VD5_A", &packet_data_.d5VD5_A);
  parameter_map_.Register("Chgsense", &packet_data_.Chgsense);
  parameter_map_.Register("d2V5VDA1_V", &packet_data_.d2V5VDA1_V);
  parameter_map_.Register("d2V5VDA2_V", &packet_data_.d2V5VDA2_V);
  parameter_map_.Register("ACDC_V", &packet_data_.ACDC_V);
  parameter_map_.Register("ADCD_A", &packet_data_.ADCD_A);
  parameter_map_.Register("BATT_V", &packet_data_.BATT_V);
  parameter_map_.Register("BATT_A", &packet_data_.BATT_A);
  parameter_map_.Register("BATT_A2", &packet_data_.BATT_A2);
  parameter_map_.Register("PBM_V", &packet_data_.PBM_V);
  parameter_map_.Register("PBM_A", &packet_data_.PBM_A);
  parameter_map_.Register("PBM_A2", &packet_data_.PBM_A2);
  parameter_map_.Register("PUMP_V", &packet_data_.PUMP_V);
  parameter_map_.Register("ECU_TEMP", &packet_data_.ECU_TEMP);
  parameter_map_.Register("ECU_TEMP1", &packet_data_.ECU_TEMP1);
  parameter_map_.Register("ECU_TEMP2", &packet_data_.ECU_TEMP2);
  parameter_map_.Register("ECU_TEMP3", &packet_data_.ECU_TEMP3);
}

/**
 * @brief Post-decoding process
 * @return True on success
 */
bool PowerEcuComEcu2DataDecoder::Update() {
  return true;
}

/**
 * @brief Constructor
 * @param[in] packet_data Packet data
 */
PowerEcuComRxackDataDecoder::PowerEcuComRxackDataDecoder()
    : IPowerEcuComDataDecoder(kRxackDataDecoderPacketSize, kRxackDataDecoderPacketName), packet_data_() {
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintDecoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintDecoder<uint8_t>(packet_data_.ack_value, 2)));

  parameter_map_.Register("is_receive_ack", &packet_data_.is_receive_ack);
  parameter_map_.Register("ack_value", &packet_data_.ack_value);
}

/**
 * @brief Post-decoding process
 * @return True on success
 */
bool PowerEcuComRxackDataDecoder::Update() {
  packet_data_.is_receive_ack = true;
  return true;
}

/**
 * @brief Constructor
 * @param[in] packet_data Packet data
 */
PowerEcuComVerDataDecoder::PowerEcuComVerDataDecoder()
    : IPowerEcuComDataDecoder(kVerDataDecoderPacketSize, kVerDataDecoderPacketName), packet_data_() {
  // ・Version information ver__ (This is returned instead of rxack in response to getv_ command)
  // Offset | Byte count | Example                                     | Content                      | Notation       | Evaluation
  // | Unit
  // 12         | 42       | hb5b862f5072d516a86e7c469bb015898e7cc631b, | Version information (ECUVER)    | Hexadecimal 40 digits | uint8
  // | -
  // 54         | 42       | hb5b862f5072d516a86e7c469bb015898e7cc632b, | Version information (ECUCOMVER) | Hexadecimal 40 digits | uint9
  // | -

  power_ecu_version_raw_.reserve(kEcu2PowerEcuVersionLength);
  packet_data_.ver_power_ecu_version.reserve(kEcu2PowerEcuVersionLength);
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(power_ecu_version_raw_, 40 + 1)));

  power_ecu_com_version_raw_.reserve(kEcu2PowerEcuComVersionLength);
  packet_data_.ver_power_ecu_com_version.reserve(kEcu2PowerEcuComVersionLength);
  element_decoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringDecoder>(
      new hsrb_power_ecu::ElementStringDecoder(power_ecu_com_version_raw_, 40 + 1)));

  parameter_map_.Register("ver_power_ecu_version", &packet_data_.ver_power_ecu_version);
  parameter_map_.Register("ver_power_ecu_com_version", &packet_data_.ver_power_ecu_com_version);
  parameter_map_.Register("is_receive_version", &packet_data_.is_receive_version);
}

/**
 * @brief Post-decoding process
 * @return True on success
 */
bool PowerEcuComVerDataDecoder::Update() {
  packet_data_.is_receive_version = true;
  packet_data_.ver_power_ecu_version.clear();
  std::copy(power_ecu_version_raw_.begin() + 1, power_ecu_version_raw_.end(),
            std::back_inserter(packet_data_.ver_power_ecu_version));
  packet_data_.ver_power_ecu_com_version.clear();
  std::copy(power_ecu_com_version_raw_.begin() + 1, power_ecu_com_version_raw_.end(),
            std::back_inserter(packet_data_.ver_power_ecu_com_version));
  return true;
}

}  // namespace hsrb_power_ecu

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
#include "power_ecu_com_data_encoder.hpp"

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/smart_ptr/make_shared.hpp>

#include "ros2_msg_utils.hpp"

namespace {
const char kTimePacketSize[] = "026";     //!< Packet size of the time synchronization command
const char kTimePacketName[] = "time_";   //!< Packet type of the time synchronization command
const char kStartPacketSize[] = "015";    //!< Packet size of the periodic communication start command
const char kStartPacketName[] = "start";  //!< Packet type of the periodic communication start command
const char kStopPacketSize[] = "011";     //!< Packet size of the periodic communication stop command
const char kStopPacketName[] = "stop_";   //!< Packet type of the periodic communication stop command
const char kHeartpacketSize[] = "027";    //!< Packet size of the heartbeat command
const char kHeartPacketName[] = "heart";  //!< Packet type of the heartbeat command
const char kPumpPacketSize[] = "013";     //!< Packet size of the pump switch command
const char kPumpPacketName[] = "pump_";   //!< Packet type of the pump switch command
const char kPbmswPacketSize[] = "013";    //!< Packet size of the drive system switch command
const char kPbmswPacketName[] = "pbmsw";  //!< Packet type of the drive system switch command
const char kLedcPacketSize[] = "023";     //!< Packet size of the multi-purpose LED color specification command
const char kLedcPacketName[] = "ledc_";   //!< Packet type of the multi-purpose LED color specification command
const char kGResPacketSize[] = "015";     //!< Packet size of the attitude angle calculation reset command
const char kGResPacketName[] = "g_res";   //!< Packet type of the attitude angle calculation reset command
const char kSolswPacketSize[] = "015";    //!< Packet size of the solenoid switch command
const char kSolswPacketName[] = "solsw";  //!< Packet type of the solenoid switch command
const char kPdcmdPacketSize[] = "015";    //!< Packet size of the power shutdown command
const char kPdcmdPacketName[] = "pdcmd";  //!< Packet type of the power shutdown command
const char kMutePacketSize[] = "015";     //!< Packet size of the audio amplifier mute command
const char kMutePacketName[] = "mute_";   //!< Packet type of the audio amplifier mute command
const char kGetvPacketSize[] = "015";     //!< Packet size of the version information acquisition command
const char kGetvPacketName[] = "getv_";   //!< Packet type of the version information acquisition command
const uint8_t kGetvpacketData = 0;        //!< Reserved value of the packet element for the version information acquisition command
const char kUndckPacketSize[] = "011";    //!< Packet size of the undock command
const char kUndckPacketName[] = "undck";  //!< Packet type of the undock command
const char k12VuPacketSize[] = "013";     //!< Packet size of the 12V USB enable command
const char k12VuPacketName[] = "12vu_";   //!< Packet type of the 12V USB enable command
const char k5Vd3PacketSize[] = "013";     //!< Packet size of the 5Vd3 enable command
const char k5Vd3PacketName[] = "5vd3_";   //!< Packet type of the 5Vd3 enable command
const char k5Vd4PacketSize[] = "013";     //!< Packet size of the 5Vd4 enable command
const char k5Vd4PacketName[] = "5vd4_";   //!< Packet type of the 5Vd4 enable command
const char k5Vd5PacketSize[] = "013";     //!< Packet size of the 5Vd5 enable command
const char k5Vd5PacketName[] = "5vd5_";   //!< Packet type of the 5Vd5 enable command
}  // anonymous namespace

namespace hsrb_power_ecu {
/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComTimeDataEncoder::PowerEcuComTimeDataEncoder()
    : IPowerEcuComDataEncoder(kTimePacketSize, kTimePacketName), packet_data_() {
  // Element registration
  //!< Date string 14 digits
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementStringEncoder>(
      new hsrb_power_ecu::ElementStringEncoder(packet_data_.start_time, 14)));

  parameter_map_.Register("start_time", &packet_data_.start_time);
}

/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComStartDataEncoder::PowerEcuComStartDataEncoder()
    : IPowerEcuComDataEncoder(kStartPacketSize, kStartPacketName), packet_data_() {
  // Element registration
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(
          new hsrb_power_ecu::ElementHexUintBitsEncoder(2));  // 2-digit hexadecimal
  // 6th bit ecu2
  bool ret;
  ret = p->RegisterBit(1, &packet_data_.is_enable_ecu2);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  // 7th bit ecu1
  ret = p->RegisterBit(0, &packet_data_.is_enable_ecu1);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_enable_ecu1", &packet_data_.is_enable_ecu1);
  parameter_map_.Register("is_enable_ecu2", &packet_data_.is_enable_ecu2);
}

/**
 * @brief Constructor
 */
PowerEcuComStopDataEncoder::PowerEcuComStopDataEncoder() : IPowerEcuComDataEncoder(kStopPacketSize, kStopPacketName) {}

/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComHeartDataEncoder::PowerEcuComHeartDataEncoder()
    : IPowerEcuComDataEncoder(kHeartpacketSize, kHeartPacketName), packet_data_() {
  //!< Error state 16-digit hexadecimal uint16
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint16_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint16_t>(packet_data_.error_state, 4)));
  //!< Count value (+1 for each transmission) 8-digit hexadecimal uint32
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint32_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint32_t>(packet_data_.counts, 8)));

  parameter_map_.Register("error_state", &packet_data_.error_state);
  parameter_map_.Register("counts", &packet_data_.counts);
}

/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComPumpDataEncoder::PowerEcuComPumpDataEncoder()
    : IPowerEcuComDataEncoder(kPumpPacketSize, kPumpPacketName), packet_data_() {
  //!< Pump switch (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.is_pump_enable, 1)));

  parameter_map_.Register("is_pump_enable", &packet_data_.is_pump_enable);
}

/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComPbmswDataEncoder::PowerEcuComPbmswDataEncoder()
    : IPowerEcuComDataEncoder(kPbmswPacketSize, kPbmswPacketName), packet_data_() {
  //!< Drive system switch (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.is_motor_enable, 1)));

  parameter_map_.Register("is_motor_enable", &packet_data_.is_motor_enable);
}

/**
 * @brief Constructor
 * @param packet_data Packet data
 */
PowerEcuComLedcDataEncoder::PowerEcuComLedcDataEncoder()
    : IPowerEcuComDataEncoder(kLedcPacketSize, kLedcPacketName), packet_data_() {
  //!< R intensity (0-255) 3-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.r, 3)));
  //!< G intensity (0-255) 3-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.g, 3)));
  //!< B intensity (0-255) 3-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementUintEncoder<uint8_t>(packet_data_.led_color.b, 3)));

  parameter_map_.Register("led_color", &packet_data_.led_color);
}

PowerEcuComGResDataEncoder::PowerEcuComGResDataEncoder()
    : IPowerEcuComDataEncoder(kGResPacketSize, kGResPacketName), packet_data_() {
  // Offset | Byte count | Example | Content         | Notation  | Evaluation | Unit
  // 12         | 4        | h00,   | Reset type     | 2-digit hexadecimal | uint8 | -

  // Reset type
  // Bit  | Label   | Content
  // 0    | res_q   | Set quaternion to 0
  // 1    | res_g   | Set gyro offset to 0
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret;
  ret = p->RegisterBit(0, &packet_data_.is_quaternion_reset);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  ret = p->RegisterBit(1, &packet_data_.is_gyro_reset);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_quaternion_reset", &packet_data_.is_quaternion_reset);
  parameter_map_.Register("is_gyro_reset", &packet_data_.is_gyro_reset);
}

PowerEcuComSolswDataEncoder::PowerEcuComSolswDataEncoder()
    : IPowerEcuComDataEncoder(kSolswPacketSize, kSolswPacketName), packet_data_() {
  // Offset | Byte count | Example | Content                           | Notation  | Evaluation | Unit
  // 12         | 4        | h00,   | Solenoid switch (0:OFF 1:ON)     | 1-digit hexadecimal | uint8 | -
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint8_t>(packet_data_.is_solenoid_enable, 2)));

  parameter_map_.Register("is_solenoid_enable", &packet_data_.is_solenoid_enable);
}

PowerEcuComPdcmdDataEncoder::PowerEcuComPdcmdDataEncoder()
    : IPowerEcuComDataEncoder(kPdcmdPacketSize, kPdcmdPacketName), packet_data_() {
  // Offset | Byte count | Example | Content               | Notation  | Evaluation | Unit
  // 12         | 4        | h00,   | Shutdown type        | 2-digit hexadecimal | uint8 | -

  // Shutdown type
  // Bit  | Label   | Content
  // 0    | pdcpu   | Shutdown internal CPU with 1
  // 1    | pdgpu   | Shutdown GPU with 1
  // 2    | pdex1   | Shutdown external CPU with 1
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret;
  ret = p->RegisterBit(0, &packet_data_.is_cpu_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  p->RegisterBit(1, &packet_data_.is_gpu_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  p->RegisterBit(2, &packet_data_.is_ex1_shutdown);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_cpu_shutdown", &packet_data_.is_cpu_shutdown);
  parameter_map_.Register("is_gpu_shutdown", &packet_data_.is_gpu_shutdown);
  parameter_map_.Register("is_ex1_shutdown", &packet_data_.is_ex1_shutdown);
}
PowerEcuComMuteDataEncoder::PowerEcuComMuteDataEncoder()
    : IPowerEcuComDataEncoder(kMutePacketSize, kMutePacketName), packet_data_() {
  // Offset | Byte count | Example | Content     | Notation  | Evaluation | Unit
  // 12         | 4        | h00,   | MUTE type  | 2-digit hexadecimal | uint8 | -

  // MUTE type
  // Bit  | Label   | Content
  // 0    | mutex   | 0:Sound on 1:Sound off
  boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder> p =
      boost::shared_ptr<hsrb_power_ecu::ElementHexUintBitsEncoder>(new hsrb_power_ecu::ElementHexUintBitsEncoder(2));
  bool ret = p->RegisterBit(0, &packet_data_.is_amp_mute);
  hsrb_power_ecu::Assert(ret, "RegisterBit failed.");
  element_encoder_list_.push_back(p);

  parameter_map_.Register("is_amp_mute", &packet_data_.is_amp_mute);
}

PowerEcuComGetvDataEncoder::PowerEcuComGetvDataEncoder()
    : IPowerEcuComDataEncoder(kGetvPacketSize, kGetvPacketName), packet_data_() {
  // Offset | Byte count | Example | Content                  | Notation  | Evaluation | Unit
  // 12         | 4        | h00,   | Version type (always 0) | 2-digit hexadecimal | uint8  | -
  packet_data_.reserved = kGetvpacketData;
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementHexUintEncoder<uint8_t> >(
      new hsrb_power_ecu::ElementHexUintEncoder<uint8_t>(packet_data_.reserved, 2)));
}

/**
 * @brief Constructor
 */
PowerEcuComUndckDataEncoder::PowerEcuComUndckDataEncoder()
    : IPowerEcuComDataEncoder(kUndckPacketSize, kUndckPacketName) {
  parameter_map_.Register("is_undck", &packet_data_.is_undck);
}

/**
 * @brief Constructor
 */
PowerEcuCom12VuDataEncoder::PowerEcuCom12VuDataEncoder()
    : IPowerEcuComDataEncoder(k12VuPacketSize, k12VuPacketName), packet_data_() {
  //!< 12V USB (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_12vu_enable, 1)));

  parameter_map_.Register("is_12vu_enable", &packet_data_.is_12vu_enable);
}

/**
 * @brief Constructor
 */
PowerEcuCom5Vd3DataEncoder::PowerEcuCom5Vd3DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd3PacketSize, k5Vd3PacketName), packet_data_() {
  //!< 5Vd3 (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd3_enable, 1)));

  parameter_map_.Register("is_5vd3_enable", &packet_data_.is_5vd3_enable);
}

/**
 * @brief Constructor
 */
PowerEcuCom5Vd4DataEncoder::PowerEcuCom5Vd4DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd4PacketSize, k5Vd4PacketName), packet_data_() {
  //!< 5Vd4 (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd4_enable, 1)));

  parameter_map_.Register("is_5vd4_enable", &packet_data_.is_5vd4_enable);
}

/**
 * @brief Constructor
 */
PowerEcuCom5Vd5DataEncoder::PowerEcuCom5Vd5DataEncoder()
    : IPowerEcuComDataEncoder(k5Vd5PacketSize, k5Vd5PacketName), packet_data_() {
  //!< 5Vd5 (0:OFF 1:ON) 1-digit decimal uint8
  element_encoder_list_.push_back(boost::shared_ptr<hsrb_power_ecu::ElementUintEncoder<bool> >(
      new hsrb_power_ecu::ElementUintEncoder<bool>(packet_data_.is_5vd5_enable, 1)));

  parameter_map_.Register("is_5vd5_enable", &packet_data_.is_5vd5_enable);
}

}  // namespace hsrb_power_ecu

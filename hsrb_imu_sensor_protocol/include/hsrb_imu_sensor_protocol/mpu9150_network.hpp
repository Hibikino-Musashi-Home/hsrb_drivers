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
/// @brief Provides a class for communicating with Invensense's gyroscope sensor MPU9150
#ifndef HSRB_IMU_SENSOR_PROTOCOL_MPU9150_NETWORK_HPP_
#define HSRB_IMU_SENSOR_PROTOCOL_MPU9150_NETWORK_HPP_

#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/noncopyable.hpp>
#include <boost/system/error_code.hpp>
#include <hsrb_imu_sensor_protocol/mpu9150_packet_parser.hpp>

namespace hsrb_imu_sensor_protocol {

/// Commands to send to MPU9150
enum MPU9150Instruction {
  /// Read sensor data
  kMPU9150InstructionReadData = 0x61,
  /// Reset
  kMPU9150InstructionReset = 0x72,
};

/// Gyroscope sensor values
struct MPU9150State {
  /// Orientation in quaternion order xyzw
  boost::array<double, 4> orientation;
  /// Angular velocity [rad/sec]
  boost::array<double, 3> angular_velocity;
  /// Acceleration [m/sec^2]
  boost::array<double, 3> linear_acceleration;
};

/// Class for communicating with Invensense's gyroscope sensor MPU9150
class MPU9150Network : private boost::noncopyable {
 public:
  /// @brief Constructor, establishes communication
  /// @param [in] device_name Device name
  /// @param [out] error_code Error code
  MPU9150Network(std::string device_name, boost::system::error_code& error_out);

  MPU9150Network(std::string device_name, boost::system::error_code& error_out, int32_t timeout, int32_t sleep_tick);

  /// @brief Destructor, closes communication
  ~MPU9150Network();

  /// @brief Send
  /// @param instruction Command to send to MPU9150
  /// @param data Parameter of the command
  /// @param size Size of data
  /// @return boost::system::error_code Error code
  boost::system::error_code Send(uint8_t instruction, const uint8_t* data, uint16_t size);

  /// @brief Receive
  /// @return boost::system::error_code Error code
  boost::system::error_code Receive();

  /// @brief Setting timeout duration
  /// @param [in] timeout Timeout duration [nanoseconds]
  void set_timeout(int32_t timeout) { timeout_ = timeout; }
  /// @brief Get timeout duration
  /// @return int32_t Timeout duration [nanoseconds]
  int32_t timeout() const { return timeout_; }

  /// @brief Setting sleep tick for waiting for send/receive
  /// @param [in] sleep_tick Sleep tick [nanoseconds]
  void set_sleep_tick(int32_t sleep_tick) { sleep_tick_ = sleep_tick; }
  /// @brief Get sleep tick for waiting for send/receive
  /// @return int32_t Sleep tick [nanoseconds]
  int32_t sleep_tick() const { return sleep_tick_; }

  /// @brief Returns the packet received by the last Receive function
  /// @return const std::vector<uint8_t>& Packet
  const std::vector<uint8_t>& all_packets() const { return parser_.packets(); }

  bool is_reply_packet() const { return parser_.is_reply_packet(); }

 private:
  boost::system::error_code Init(const std::string& device_name);
  /// File descriptor
  int fd_;
  /// Timeout duration [nanoseconds]
  int32_t timeout_;
  /// Sleep tick [nanoseconds]
  int32_t sleep_tick_;
  /// parser
  MPU9150PacketParser parser_;
  /// Send/receive buffer
  boost::array<uint8_t, 4095> buffer_;
};


/// Converter class
class MPU9150PacketConverter {
 public:
  /// Constructor, initialize variables
  MPU9150PacketConverter();

  /// Destructor, does nothing
  ~MPU9150PacketConverter() {}

  /// @brief Convert packet from sensor to SensorState and output
  /// @param [in] packet Packet from sensor
  /// @param [out] orientation Orientation in quaternion order xyzw
  /// @param [out] angular_velocity Angular velocity [rad/sec]
  /// @param [out] linear_acceleration Acceleration [m/sec^2]
  void ToSensorState(const std::vector<uint8_t>& packet, boost::array<double, 4>& orientation,
                     boost::array<double, 3>& angular_velocity, boost::array<double, 3>& linear_acceleration);

  /// @brief Convert packet from sensor to SensorState and output
  /// @param [in] packet Packet from sensor
  /// @param [out] sensor_state Converted values
  void ToSensorState(const std::vector<uint8_t>& packet, MPU9150State& sensor_state);

 private:
  uint32_t last_timestamp_;
  uint32_t last_packet_num_;
};
}  // end of namespace hsrb_imu_sensor_protocol
#endif  // HSRB_IMU_SENSOR_PROTOCOL_MPU9150_NETWORK_HPP_

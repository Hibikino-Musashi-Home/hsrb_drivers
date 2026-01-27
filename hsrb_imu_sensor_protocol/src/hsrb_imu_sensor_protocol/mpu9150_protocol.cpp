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
/// @brief User-friendly API for Invensense's gyro sensor MPU9150

#include <hsrb_imu_sensor_protocol/mpu9150_protocol.hpp>
#include <vector>

namespace hsrb_imu_sensor_protocol {

// Wait time after reset [sec]
const double kMPU9150ResetDuration = 3.0;
// Wait time after setting the acceleration zero point [sec]
const double kMPU9150AccelZeroSetDuration = 0.5;
// Timeout for waiting for command completion to MPU9150 [sec]
const double kMPU9150InstructionTimeout = 5.0;

/// @brief Determine if the packet is a checksum-only packet with parameter 0
/// @param [in] packet Target for determination
/// @return bool Returns true if the packet has parameter 0
/// @note MPU9150 returns a packet with parameter 0
///       upon completion of commands like reset and range setting.
///       In such cases, the packet length is 1 (checksum only).
bool IsParameterZeroPacket(const std::vector<uint8_t>& packet) {
  // The third element of the packet is the packet length
  if (packet.size() < 2) {
    return false;
  }
  if (packet[2] == 0x01) {
    return true;
  } else {
    return false;
  }
}

/// @brief Constructor
MPU9150Protocol::MPU9150Protocol(MPU9150Network& network) : network_(network), status_(kStatusWaiting) {}

// Read current sensor values
MPU9150Protocol::ErrorCode MPU9150Protocol::ReadState(ImuState& state) {
  // Do nothing unless kStatusWaiting
  MPU9150Protocol::ErrorCode error;
  if (status_ == kStatusWaiting) {
    // Receive packet from MPU9150
    error = network_.Receive();
    if (error) {
      return error;
    } else {
      MPU9150State mpu9150_state;
      packet_converter_.ToSensorState(network_.all_packets(), mpu9150_state);
      state.orientation = mpu9150_state.orientation;
      state.angular_velocity = mpu9150_state.angular_velocity;
      state.linear_acceleration = mpu9150_state.linear_acceleration;
    }
  }
  return MPU9150Protocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
}

// Perform sensor reset
MPU9150Protocol::ErrorCode MPU9150Protocol::TryReset(ResetResult& result) {
  MPU9150Protocol::ErrorCode error;
  switch (status_) {
    case kStatusWaiting:
      // Send reset command
      error = network_.Send(kMPU9150InstructionReset, NULL, 0);
      if (error) {
        result = kError;
        return error;
      } else {
        // If sent successfully, set each time and change to kStatusWaitForReset
        instruction_end_time_ =
            rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(kMPU9150ResetDuration);
        instruction_timeout_ =
            rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(kMPU9150InstructionTimeout);
        status_ = kStatusWaitForReset;
      }
      break;
    case kStatusWaitForReset:
      // Transition to waiting for reception state if the expected time for response from MPU9150 has passed
      if (rclcpp::Clock(RCL_ROS_TIME).now() > instruction_end_time_) {
        status_ = kStatusReceiveResetReturn;
      }
      break;
    case kStatusReceiveResetReturn:
      // Receive response for reset completion
      error = network_.Receive();
      if (error || !network_.is_reply_packet()) {
        // Return to kStatusWaiting if timeout is exceeded
        if (rclcpp::Clock(RCL_ROS_TIME).now() > instruction_timeout_) {
          status_ = kStatusWaiting;
          result = kError;
          return MPU9150Protocol::ErrorCode(boost::system::errc::timed_out, boost::system::system_category());
        }
      } else {
        status_ = kStatusWaiting;
        std::cerr << "Reset success" << std::endl;
        result = kDone;
        return MPU9150Protocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
      }
      break;
    default:
      break;
  }
  result = kContinue;
  return MPU9150Protocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
}


MPU9150Protocol::ErrorCode MPU9150Protocol::Reset(double timeout) {
  rclcpp::Time time_limit = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout);
  while (rclcpp::Clock(RCL_ROS_TIME).now() < time_limit) {
    MPU9150Protocol::ErrorCode error;
    ResetResult result;
    error = TryReset(result);
    if (error) {
      return error;
    } else if (result == kDone) {
      return MPU9150Protocol::ErrorCode(boost::system::errc::success, boost::system::system_category());
    }
  }
  return MPU9150Protocol::ErrorCode(boost::system::errc::timed_out, boost::system::system_category());
}
}  // namespace hsrb_imu_sensor_protocol

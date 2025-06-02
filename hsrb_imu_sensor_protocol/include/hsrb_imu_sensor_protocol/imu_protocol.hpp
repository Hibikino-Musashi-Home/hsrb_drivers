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
/// @file imu_protocol.hpp
#ifndef HSRB_IMU_SENSOR_PROTOCOL_IMU_PROTOCOL_HPP_
#define HSRB_IMU_SENSOR_PROTOCOL_IMU_PROTOCOL_HPP_

#include <vector>

#include <boost/array.hpp>
#include <boost/system/error_code.hpp>

namespace hsrb_imu_sensor_protocol {
/// Gyro sensor values
struct ImuState {
  /// Attitude, Quaternion in the order of xyzw
  boost::array<double, 4> orientation;
  /// Angular velocity [rad/sec]
  boost::array<double, 3> angular_velocity;
  /// Acceleration [m/sec^2]
  boost::array<double, 3> linear_acceleration;
};

/// IMU communication protocol
class IImuProtocol {
 public:
  typedef boost::system::error_code ErrorCode;
  virtual ~IImuProtocol() {}
  /// Result of sensor reset
  enum ResetResult {
    /// Normal termination
    kDone,
    /// Fail
    kError,
    /// Resetting
    kContinue
  };

  /// @brief Read the current sensor values
  /// @param [out] state Current sensor values
  /// @return boost::system::error_code Error code
  /// @note Types of error codes
  ///        Success: errc::success
  ///        Communication protocol error: errc::protocol_error
  ///        Timeout: errc::timed_out
  virtual ErrorCode ReadState(ImuState& state) = 0;
  /// @brief Perform sensor reset
  /// @param [out] result Result of reset processing
  /// @return boost::system::error_code Error code
  /// @note This function does not wait for reset completion.
  ///       Continue calling until result becomes kDone, kError
  ///       Types of error codes
  ///         Success: errc::success
  ///         Timeout: errc::timed_out
  ///                         Internally set communication timeout
  ///         Communication protocol error: errc::protocol_error
  virtual ErrorCode TryReset(ResetResult& result) = 0;
  /// @brief Perform sensor reset
  /// @param [in] timeout Timeout [s]
  /// @return boost::system::error_code Error code
  /// @note This function waits until timeout
  ///       Types of error codes
  ///         Success: errc::success
  ///         Timeout: errc::timed_out
  ///                        Set timeout period or
  ///                        Internally set communication timeout
  ///                        Timeout occurs whichever is sooner
  ///         Communication protocol error: errc::protocol_error
  virtual ErrorCode Reset(double timeout) = 0;
};

}  // namespace hsrb_imu_sensor_protocol

#endif /*HSRB_IMU_SENSOR_PROTOCOL_IMU_PROTOCOL_HPP_*/

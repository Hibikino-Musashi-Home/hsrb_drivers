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
#ifndef HSRB_POWER_ECU_NETWORK_MOCK_HPP_
#define HSRB_POWER_ECU_NETWORK_MOCK_HPP_

#include <string>
#include <vector>

#include <boost/system/error_code.hpp>
#include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>

namespace hsrb_power_ecu {
/**
 * @brief Mock class for the serial port
 */
class NetworkMock : public INetwork {
 public:
  NetworkMock();
  virtual ~NetworkMock();

  /**
   * @brief Open
   * @return boost::system::errc::success on success
   */
  virtual boost::system::error_code Open();
  /**
   * @brief Close
   * @return boost::system::errc::success on success
   */
  virtual boost::system::error_code Close();
  /**
   * @brief Change network settings
   * @param[in] param Setting name
   * @param[in] value Change value
   * @return boost::system::errc::success on successful transmission
   */
  virtual boost::system::error_code Configure(const std::string &param, int32_t value);
  /**
   * @brief Change network settings
   * @param[in] param Setting name
   * @param[in] value Change value
   * @return boost::system::errc::success on successful transmission
   */
  virtual boost::system::error_code Configure(const std::string &param, double value);
  /**
   * @brief Change network settings
   * @param[in] param Setting name
   * @param[in] value Change value
   * @return boost::system::errc::success on successful transmission
   */
  virtual boost::system::error_code Configure(const std::string &param, const std::string &value);
  /**
   * @brief Transmit
   * Transmit all contents of the transmission data buffer received as an argument within the timeout period
   * @param[in] data Transmission data buffer
   * @return boost::system::errc::success on successful transmission
   */
  virtual boost::system::error_code Send(const PacketBuffer &data);
  /**
   * @brief Receive
   * Store the transmission data at the end of the buffer.
   * If there is no received data, wait to receive within the timeout period.
   * @param[out] data Received buffer
   * @return boost::system::errc::success on successful transmission
   */
  virtual boost::system::error_code Receive(PacketBuffer &data);

  std::string GetSendBuffer() const;  //!< Retrieve the value of the send buffer
  void ResetSendBuffer();  //!< Clear the send buffer
  void UpdateBuffer(const std::string& buffer_data);  //!< Change the contents of the receive buffer

 private:
  std::string send_buffer_;  //!< Send buffer
  std::string receive_buffer_;     //!< Receive buffer
  bool is_update_receive_buffer_;  //!< Receive buffer update flag
  boost::mutex receive_buffer_mutex_;
  uint32_t timeout_ns_;      //!< Timeout period
  std::string port_name_;    //!< Port name

  bool is_need_ack_;                           //!< Whether an ACK reply is necessary
  bool is_need_ver_;                           //!< Whether a Ver reply is necessary
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_NETWORK_MOCK_HPP_

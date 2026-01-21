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
#ifndef HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_
#define HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_

#include <inttypes.h>

#include <string>

#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/unordered_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include "any_type_pointer_map.hpp"
#include "error_counter.hpp"
#include "power_ecu_com_frame_decoder.hpp"
#include "power_ecu_com_frame_encoder.hpp"
#include "ros2_msg_utils.hpp"

namespace hsrb_power_ecu {

/**
 * @brief Control command information management class
 *
 * The control command transmission from CPU to power ECU requires confirmation of ACK from power ECU for each transmission,
 * Also, considering communication failure, it is necessary to perform retry processing for each control command,
 * Only one transmission is performed per cycle.
 * On the other hand, since commands from topics are always accepted,
 * It is necessary to queue commands to process each command sequentially.
 * The CommandState class is a class that holds information about queued control commands.
 */
class CommandState {
 private:
  CommandState(CommandState const&);             // = delete;
  CommandState& operator=(CommandState const&);  // = delete;

 public:
  typedef boost::shared_ptr<CommandState> Ptr;

  /**
   * @brief Constructor
   * @param[in] command_name Command name
   */
  explicit CommandState(const std::string& command_name)
      : command_name_(command_name), return_value_(0), is_processing_(0), retry_count_(0) {}
  /**
   * @brief Destructor
   */
  ~CommandState() {}
  /**
   * @brief Get command name
   * @return Command name
   */
  inline const std::string& GetCommandName() const { return command_name_; }
  /**
   * @brief Get command return value
   * The assignment process for the command return value is implemented but not yet used.
   * @return Command return value
   */
  inline uint8_t GetReturnValue() const { return return_value_; }
  /**
   * @brief Assign command return value
   * @param[in] value Command return value
   */
  inline void SetReturnValue(const uint8_t value) { return_value_ = value; }
  /**
   * @brief Assign command processing flag
   * Returns True from command transmission to ack reception.
   * The assignment process for the flag value is implemented but not yet used.
   * @param[in] value Flag value
   */
  inline void SetProcessingStatus(const bool value) { is_processing_ = value; }
  /**
   * @brief Get command processing flag
   * @return Command processing True
   */
  inline bool IsProcessing() const { return is_processing_; }
  /**
   * @brief Increment retry count
   */
  inline void IncrementRetryCount() { ++retry_count_; }
  /**
   * @brief Clear retry count
   */
  inline void ClearRetryCount() { retry_count_ = 0; }
  /**
   * @brief Get retry count
   * @return Retry count
   */
  inline uint32_t GetRetryCount() const { return retry_count_; }

 private:
  const std::string command_name_;  //!< Command name
  uint8_t return_value_;            //!< Return value
  bool is_processing_;              //!< Processing flag (True during processing)
  uint32_t retry_count_;            //!< Retry count
};

// @n When included in the Protocol class, the class name can be Versions only
struct PowerEcuVersions {
  std::string power_ecu_version;
  std::string power_ecu_com_version;
};

/**
 * @brief Class that performs processing independent of the communication command protocol version <br>
 * <br>
 * Processing independent of the communication protocol version is as follows <br>
 *   - Version confirmation <br>
 *   - Control command transmission, ack reception <br>
 *   - Heartbeat transmission <br>
 * <br>
 * Also, for the expansion of control commands,
 * It provides registration functions for command queues and data decoders/encoders.<br>
 * Processing sent from topics is asynchronous, but,
 * Communication with the power ECU needs to be processed sequentially, so it is designed to perform queuing.<br>
 *
 * Also, since this class is used in applications called by the rosrun command,
 * This class is designed to be independent of roscore.
 * If you want to emit errors of this class as topics like diagnostics,
 * The upper class managing this class should handle that function.
 */
class PowerEcuProtocol : boost::noncopyable {
 public:
  typedef boost::shared_ptr<PowerEcuProtocol> Ptr;

  explicit PowerEcuProtocol(boost::shared_ptr<hsrb_power_ecu::INetwork> network,
                            const rclcpp::Node::SharedPtr& node);
  ~PowerEcuProtocol() { Close(); }

  /**
   * @brief Open
   */
  bool Open();

  /**
   * @brief Close
   */
  void Close();

  /**
   * @brief Obtain version information from ECU and prepare protocol communication
   */
  bool Init();

  /**
   * @brief Start communication
   *
   * @return
   */
  boost::system::error_code Start();

  /**
   * @brief End communication
   *
   * @return
   */
  boost::system::error_code Stop();

  /**
  * @brief Return status management information related to command by name
  *
  * Returns false if the command is not acceptable
  *
  * @param[in] command Command name to obtain
  * @param[out] command_state
  */
  inline bool GetCommandState(const std::string& command, hsrb_power_ecu::CommandState::Ptr& command_state) const {
    CommandMapType::const_iterator it = command_map_.find(command);
    if (it == command_map_.end()) {
      return false;
    }
    command_state = it->second;
    return true;
  }

  /**
   * @brief Check if command is valid
   *
   * @param[in] name Command name
   *
   * @return True when valid
   */
  inline bool HasCommand(const std::string& name) const { return (command_map_.find(name) != command_map_.end()); }

  /**
   * @brief Get data pointer
   *
   * @tparam T Data type
   * @param[in] name Data name
   *
   * @return Success: Data pointer<br>
   *         Failure: NULL<br>
   *         Failure occurs if the data name is unregistered or the data type does not match
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    T* ret = frame_decoder_.GetParamPtr<T>(name);
    if (ret != NULL) {
      return ret;
    }
    ret = frame_encoder_.GetParamPtr<T>(name);
    return ret;
  }

  /**
   * @brief Get Receive error rate
   * @return Error rate
   */
  inline double GetReceiveErrorRate() const { return read_error_counter_.GetErrorRate(); }

  /**
   * @brief Get Send error rate
   * @return Error rate
   */
  inline double GetSendErrorRate() const { return write_error_counter_.GetErrorRate(); }

  /**
   * @brief Obtain version information
   *
   * @param[out] result Version information
   *
   * @return True on success
   */
  bool GetPowerEcuVersions(PowerEcuVersions& result);

  /**
   * @brief Add to command queue
   *
   *
   * @param[in] command_name Command to add
   *
   * @return Success: true<br>
   * Returns false when the command is not acceptable
   */
  bool AddCommandQueue(const std::string& command_name) {
    CommandMapType::const_iterator it = command_map_.find(command_name);
    if (it == command_map_.end()) {
      return false;
    }
    AddCommandQueue(it->second);
    return true;
  }

  /**
   * @brief Process all command queues
   *
   * This method is intended to be called during non-real-time processes
   *
   * @param[in] check_ros ros::ok() check necessity true: check required false: check not required
   * @param[in] cycle_hz Polling cycle [hz]
   * @param[in] error_rate Acceptable error rate
   *
   * @return Execution result
   * @retval Success boost::system::errc::success
   * @retval Invalid argument boost::system::errc::invalid_argument
   * @retval Communication error boost::system::errc::operation_canceled
   * @retval Communication timeout boost::system::errc::timed_out
   * @retval Roscore not started boost::system::errc::operation_not_permitted
   */
  boost::system::error_code ProcessCommandQueue(bool check_ros, double cycle_hz, double error_rate);

  /**
   * @brief Receive processing
   *
   * Irrecoverable errors in the communication protocol (exceeding control command retry allowance, ACK timeout)
   * Exit within this function
   *
   * @return Execution result
   * @retval Normal boost::system::errc::success
   * @retval Network Receive failure boost::system::errc::network_down
   * @retval Received packet is corrupted boost::system::errc::protocol_error
   * @retval Control command retransmission boost::system::errc::resource_unavailable_try_again
   */
  boost::system::error_code ReceiveAll();

  /**
   * @brief Send processing
   *
   * @return Execution result
   * @retval Normal boost::system::errc::success
   * @retval Network Send failure boost::system::errc::network_down
   */
  boost::system::error_code SendAll();

 private:
  typedef boost::unordered_map<std::string, hsrb_power_ecu::CommandState::Ptr>
      CommandMapType;  //!< Type of control command map
  typedef boost::circular_buffer<hsrb_power_ecu::CommandState::Ptr>
      CommandBuffer;  //!< Type of control command queue

  /**
   * @brief Add to command queue (internal use)
   *
   * @param[in] command command_state
   */
  void AddCommandQueue(hsrb_power_ecu::CommandState::Ptr command);

  /**
   * @brief Register data decoder
   *
   * @tparam T Type of data decoder
   */
  template <typename T>
  void RegisterDataDecoder() {
    boost::shared_ptr<T> const decoder = boost::make_shared<T>();
    boost::system::error_code ret = frame_decoder_.RegisterDataDecoder(decoder);
    hsrb_power_ecu::Assert(ret == boost::system::errc::success, "frame decoder regiser failed.");
  }

  /**
   * @brief Register data encoder
   *
   * @tparam T Type of data encoder
   * @param[out] name Command name
   */
  template <typename T>
  std::string RegisterDataEncoder() {
    boost::shared_ptr<T> const encoder = boost::make_shared<T>();
    std::string command_name = encoder->GetPacketName();
    boost::system::error_code ret = frame_encoder_.RegisterDataEncoder(encoder);
    hsrb_power_ecu::Assert(ret == boost::system::errc::success, "frame decoder regiser failed.");
    hsrb_power_ecu::Assert((command_map_.find(command_name) == command_map_.end()), "command state register failed.");
    command_map_[command_name] = boost::make_shared<hsrb_power_ecu::CommandState>(command_name);
    return command_name;
  }

  /**
   * @brief Send command
   * Send command to serial device.
   *
   * Since commands are managed in the command queue,
   * Basically, use the AddCommandQueue method to send commands.
   *
   * Control commands managed in the command queue need to return an Rxack command as a return value by design.
   * Control commands that do not return an Rxack command (such as getv_ command) should have a dedicated method created,
   * Use SendCommand to send commands directly.
   *
   * @param[in] command Command information to send
   * @return Execution result
   * @retval Normal boost::system::errc::success
   * @retval Network Send failure boost::system::errc::network_down
   */
  boost::system::error_code SendCommand(const hsrb_power_ecu::CommandState::Ptr command);

  // Variables
  // Network interface management
  boost::shared_ptr<hsrb_power_ecu::INetwork> network_;  //!< Network interface
  PacketBuffer receive_buffer_;                          //!< Receive buffer
  PacketBuffer send_buffer_;                             //!< Send buffer

  // Transport management
  //// Decoder
  hsrb_power_ecu::PowerEcuComFrameDecoder frame_decoder_;  //!< Frame decoder
  //// Encoder
  hsrb_power_ecu::PowerEcuComFrameEncoder frame_encoder_;  //!< Frame encoder

  CommandBuffer command_queue_;       //!< Command queue

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_send_command_time_;  //!< Command send time
  rclcpp::Time last_heartbeat_time_;  //!< Last heartbeat send time

  bool is_waiting_ack_;            //!< Flag indicating whether waiting for ACK

  ErrorCounter read_error_counter_;   //!< Error rate of Read method
  ErrorCounter write_error_counter_;  //!< Error rate of Write method

  // Command map
  CommandMapType command_map_;               //!< Map of control commands

  // Received command data
  //// rxack
  bool* is_receive_ack_;  //!< Whether ACK has been returned
  uint8_t* ack_value_;    //!< Return value of reply command
  //// ver
  std::string* ver_power_ecu_version_;      //!< Power ECU firmware Ver [git hash 20 bytes] 40 hexadecimal digits
  std::string* ver_power_ecu_com_version_;  //!< Power ECU communication structure HASH [hash 20 bytes] 40 hexadecimal digits
  bool* is_receive_version_;                //!< Whether Ver command has been received
  // Sent command data
  //// heart
  uint32_t* counts;  //!< Heartbeat count value (incremented by +1 for each transmission) 8 hexadecimal digits uint32

  // Command names
  std::string heart_command_name_;  //!< heart command
  std::string getv_command_name_;   //!< getv_ command
  std::string time_command_name_;   //!< time command
  std::string start_command_name_;  //!< start command
  std::string stop_command_name_;   //!< stop command
  std::string mute_command_name_;   //!< mute command
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_

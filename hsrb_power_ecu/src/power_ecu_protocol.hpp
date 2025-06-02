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
 * @brief Control Command Information Management Class
 *
 * CPU->control command transmission to Power ECU requires confirmation of ACK from Power ECU after each transmission,
 * Furthermore, due to the possibility of communication failure, it is necessary to perform retry processing for each control command,
 * Only at most one transmission is performed per cycle.
 * On the other hand, commands from the topic are constantly accepted, so
 * It is necessary to queue commands for sequential processing of each command.
 * The CommandState class is a class that holds the information of control commands that are queued.
 */
class CommandState {
 private:
  CommandState(CommandState const&);             // = delete;
  CommandState& operator=(CommandState const&);  // = delete;

 public:
  typedef boost::shared_ptr<CommandState> Ptr;

  /**
   * @brief Constructor
   * @param[in] command_name Command Name
   */
  explicit CommandState(const std::string& command_name)
      : command_name_(command_name), return_value_(0), is_processing_(0), retry_count_(0) {}
  /**
   * @brief Destructor
   */
  ~CommandState() {}
  /**
   * @brief Get Command Name
   * @return Command Name
   */
  inline const std::string& GetCommandName() const { return command_name_; }
  /**
   * @brief Get Command Return Value
   * The assignment process of the command return value is implemented but not yet used.
   * @return Command Return Value
   */
  inline uint8_t GetReturnValue() const { return return_value_; }
  /**
   * @brief Assign Command Return Value
   * @param[in] value Command Return Value
   */
  inline void SetReturnValue(const uint8_t value) { return_value_ = value; }
  /**
   * @brief Assign Command Processing Flag
   * Returns True from command transmission to ACK reception.
   * The assignment process of the flag value is implemented but not yet used.
   * @param[in] value Flag Value
   */
  inline void SetProcessingStatus(const bool value) { is_processing_ = value; }
  /**
   * @brief Get Command Processing Flag
   * @return Command Processing True
   */
  inline bool IsProcessing() const { return is_processing_; }
  /**
   * @brief Increment Retry Count
   */
  inline void IncrementRetryCount() { ++retry_count_; }
  /**
   * @brief Clear Retry Count
   */
  inline void ClearRetryCount() { retry_count_ = 0; }
  /**
   * @brief Get Retry Count
   * @return Retry Count
   */
  inline uint32_t GetRetryCount() const { return retry_count_; }

 private:
  const std::string command_name_;  //!< Command name
  uint8_t return_value_;            //!< Return value
  bool is_processing_;              //!< Processing flag (True while processing)
  uint32_t retry_count_;            //!< Retry count
};

// @n When put into the Protocol class, the class name can be Versions only
struct PowerEcuVersions {
  std::string power_ecu_version;
  std::string power_ecu_com_version;
};

/**
 * @brief Class that performs processing independent of the communication command protocol version <br>
 * <br>
 * The processing independent of the communication protocol version is as follows <br>
 *   - Version confirmation <br>
 *   - Control command transmission, ACK reception <br>
 *   - Heartbeat transmission <br>
 * <br>
 * Also, for the extension of control commands,
 * It publicly offers registration functions to the command queue and data decoder/encoder. <br>
 * Processing sent from the topic is asynchronous, but
 * The communication with the Power ECU needs to be sequential, so a queuing design is adopted. <br>
 *
 * Additionally, since this class is used in applications called by the rosrun command
 * This class has a design that is independent of roscore.
 * If you wish to emit the errors of this class as topics like diagnostics,
 * The upper-level class managing this class should take on that function.
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
   * @brief Obtain version information from ECU and prepare for protocol communication
   */
  bool Init();

  /**
   * @brief Start Communication
   *
   * @return
   */
  boost::system::error_code Start();

  /**
   * @brief End Communication
   *
   * @return
   */
  boost::system::error_code Stop();

  /**
  * @brief Return the status management information regarding the command by name
  *
  * Returns false if the command is unacceptable
  *
  * @param[in] command Command name to retrieve
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
   * @brief Check if the command is valid
   *
   * @param[in] name Command name
   *
   * @return True when valid
   */
  inline bool HasCommand(const std::string& name) const { return (command_map_.find(name) != command_map_.end()); }

  /**
   * @brief Obtain pointer to data
   *
   * @tparam T Data type
   * @param[in] name Data name
   *
   * @return On success: pointer to data<br>
   *         On failure: NULL<br>
   *         Failure occurs if the data name is unregistered or if the data type is mismatched.
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
   * @brief Obtain Receive error rate
   * @return Error rate
   */
  inline double GetReceiveErrorRate() const { return read_error_counter_.GetErrorRate(); }

  /**
   * @brief Obtain Send error rate
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
   * @brief Add to Command Queue
   *
   *
   * @param[in] command_name Command to add
   *
   * @return On success: true<br>
   * Returns false when the command is unacceptable
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
   * @brief Process all Command Queue
   *
   * This method is intended to be called during non-real-time processes.
   *
   * @param[in] check_ros Whether to check ros::ok() true: Check needed false: Check not needed
   * @param[in] cycle_hz Polling cycle [hz]
   * @param[in] error_rate Allowable error rate
   *
   * @return Execution Result
   * @retval On success boost::system::errc::success
   * @retval Argument error boost::system::errc::invalid_argument
   * @retval Communication error boost::system::errc::operation_canceled
   * @retval Communication timeout boost::system::errc::timed_out
   * @retval roscore not running boost::system::errc::operation_not_permitted
   */
  boost::system::error_code ProcessCommandQueue(bool check_ros, double cycle_hz, double error_rate);

  /**
   * @brief Receive Processing
   *
   * Irrecoverable errors on the communication protocol (control command retry limit exceedance, ACK timeout)
   * Cause exit within this function
   *
   * @return Execution Result
   * @retval Normal boost::system::errc::success
   * @retval network Receive failure boost::system::errc::network_down
   * @retval Corrupted received packet boost::system::errc::protocol_error
   * @retval Control command retransmission boost::system::errc::resource_unavailable_try_again
   */
  boost::system::error_code ReceiveAll();

  /**
   * @brief Send Processing
   *
   * @return Execution Result
   * @retval Normal boost::system::errc::success
   * @retval network Send failure boost::system::errc::network_down
   */
  boost::system::error_code SendAll();

 private:
  typedef boost::unordered_map<std::string, hsrb_power_ecu::CommandState::Ptr>
      CommandMapType;  //!< Map type for control commands
  typedef boost::circular_buffer<hsrb_power_ecu::CommandState::Ptr>
      CommandBuffer;  //!< Command queue type

  /**
   * @brief Add to Command Queue (internal)
   *
   * @param[in] command command_state
   */
  void AddCommandQueue(hsrb_power_ecu::CommandState::Ptr command);

  /**
   * @brief Register Data Decoder
   *
   * @tparam T Data decoder type
   */
  template <typename T>
  void RegisterDataDecoder() {
    boost::shared_ptr<T> const decoder = boost::make_shared<T>();
    boost::system::error_code ret = frame_decoder_.RegisterDataDecoder(decoder);
    hsrb_power_ecu::Assert(ret == boost::system::errc::success, "frame decoder regiser failed.");
  }

  /**
   * @brief Register Data Encoder
   *
   * @tparam T Data encoder type
   * @param[out] name Command Name
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
   * @brief Send Command
   * Send a command to the serial device.
   *
   * Because commands are managed in the command queue,
   * It is basic to use the AddCommandQueue method for command transmission.
   *
   * Control commands managed by the command queue need to return the Rxack command as a return value.
   * For control commands that do not return the Rxack command (like getv_command),
   * A dedicated method is created, and the command is sent directly using SendCommand.
   *
   * @param[in] command Information of the command to send
   * @return Execution Result
   * @retval Normal boost::system::errc::success
   * @retval network Send failure boost::system::errc::network_down
   */
  boost::system::error_code SendCommand(const hsrb_power_ecu::CommandState::Ptr command);

  // Variables
  // Network Interface Management
  boost::shared_ptr<hsrb_power_ecu::INetwork> network_;  //!< Network interface
  PacketBuffer receive_buffer_;                          //!< Receive buffer
  PacketBuffer send_buffer_;                             //!< Send buffer

  // Transport Management
  //// Decoder
  hsrb_power_ecu::PowerEcuComFrameDecoder frame_decoder_;  //!< Frame decoder
  //// Encoder
  hsrb_power_ecu::PowerEcuComFrameEncoder frame_encoder_;  //!< Frame encoder

  CommandBuffer command_queue_;       //!< Command queue

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_send_command_time_;  //!< Command send time
  rclcpp::Time last_heartbeat_time_;  //!< Last heartbeat send time

  bool is_waiting_ack_;            //!< Flag indicating whether waiting for ACK

  ErrorCounter read_error_counter_;   //!< Read method error rate
  ErrorCounter write_error_counter_;  //!< Write method error rate

  // Command map
  CommandMapType command_map_;               //!< Map of control commands

  // Received command data
  //// rxack
  bool* is_receive_ack_;  //!< Whether ACK was received
  uint8_t* ack_value_;    //!< Return value of the reply command
  //// ver
  std::string* ver_power_ecu_version_;      //!< Power ECU firmware version [git hash 20 bytes] 40-digit hexadecimal
  std::string* ver_power_ecu_com_version_;  //!< Power ECU communication structure HASH [hash 20 bytes] 40-digit hexadecimal
  bool* is_receive_version_;                //!< Whether the Ver command was received
  // Send command data
  //// heart
  uint32_t* counts;  //!< Heartbeat count value (incremented by +1 each transmission) 8-digit hexadecimal uint32

  // Command name
  std::string heart_command_name_;  //!< Heart command
  std::string getv_command_name_;   //!< getv_command
  std::string time_command_name_;   //!< Time command
  std::string start_command_name_;  //!< Start command
  std::string stop_command_name_;   //!< Stop command
  std::string mute_command_name_;   //!< Mute command
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_

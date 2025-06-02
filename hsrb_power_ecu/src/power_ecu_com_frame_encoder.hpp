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
#ifndef POWER_ECU_COM_FRAME_ENCODER_HPP_
#define POWER_ECU_COM_FRAME_ENCODER_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/unordered/unordered_map.hpp>

#include "any_type_pointer_map.hpp"
#include "power_ecu_com_common.hpp"

namespace hsrb_power_ecu {
/**
 * @brief Interface for packet element encoder
 */
class IElementEncoder {
 public:
  /**
   * @brief Smart pointer of IElementEncoder
   */
  typedef boost::shared_ptr<IElementEncoder> Ptr;
  /**
   * @brief Destructor
   */
  virtual ~IElementEncoder() {}
  /**
   * @brief Encode
   * @param[out] buffer Output destination buffer
   * @return true when encoding succeeds
   */
  virtual bool Encode(PacketBuffer &buffer) = 0;
};

/**
 * @brief Interface for data encoder
 */
class IPowerEcuComDataEncoder {
 private:
  IPowerEcuComDataEncoder(IPowerEcuComDataEncoder const &);             // = delete;
  IPowerEcuComDataEncoder &operator=(IPowerEcuComDataEncoder const &);  // = delete;

 protected:
  /**
   * @brief Constructor
   * @param packet_size Size of the packet header part
   * @param packet_name Packet type of the packet header part
   */
  IPowerEcuComDataEncoder(const std::string &packet_size, const std::string &packet_name)
      : packet_size_(packet_size), packet_name_(packet_name) {}

 public:
  /**
   * @brief Destructor
   */
  virtual ~IPowerEcuComDataEncoder() {}
  /**
   * @brief Encode
   * @param[out] buffer Output destination buffer
   * @return
   * Normal termination boost::system::errc::success
   * Encoding failure boost::system::errc::protocol_error
   */
  virtual inline boost::system::error_code Encode(PacketBuffer &buffer) {
    BOOST_FOREACH (IElementEncoder::Ptr const p, element_encoder_list_) {
      if (!p->Encode(buffer)) {
        return boost::system::errc::make_error_code(
            boost::system::errc::protocol_error);  // TODO(kitsunai): テスト未実施
      }
      buffer.push_back(',');
    }
    return boost::system::errc::make_error_code(boost::system::errc::success);
  }
  /**
   * @brief Get the size of the packet header part
   * @return Packet size
   */
  virtual inline std::string GetPacketSizeStr() const { return packet_size_; }
  /**
   * @brief Get the packet type of the packet header part
   * @return Packet type
   */
  inline std::string GetPacketName() const { return packet_name_; }

  /**
   * @brief Get the pointer to data
   *
   * @tparam T Data type
   * @param[in] name Data name
   *
   * @return On success: Pointer to data<br>
   * On failure: NULL<br>
   * Fails if unregistered data name or data type mismatch occurs
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    return parameter_map_.GetPtr<T>(name);
  }

 protected:
  std::vector<IElementEncoder::Ptr> element_encoder_list_;  //!< List of encoding instructions per element
  const std::string packet_size_;                           //!< Packet size of the header part
  const std::string packet_name_;                           //!< Packet type of the header part
  any_type_pointer_map::Map parameter_map_;                 //!< Map of control command parameters
};

/**
 * @brief Frame encoder for the transmission packet
 * The communication format is categorized into elements like frame, data, and element.
 *
 * Example) For the command "H,ledc_,23,000,100,255,h12345678,\0",
 * - frame :
 * Refers to the entire communication packet e.g. "H,ledc_,23,000,100,255,h12345678,\0"
 * The frame encoder handles the calculation of the command header and footer,
 * The frame encoder knows the specification of the header and footer of the communication packet,
 * Manages the data encoder by using the command name as a key.
 *
 * - data :
 * Indicates the parts with the header and footer removed, e.g. "000,100,255,"
 * The data encoder divides data into elements and calls the element encoder.
 * After encoding all elements, performs post-processing like conversion to physical quantities.
 * The data encoder is defined for each command, with the command name, command size,
 * Besides the composition of elements (order, type, and digits of each element),
 * Knows references to PacketData containing information necessary for encoding.
 * Additionally, holds the information as PacketData type, which serves as the source for encoding for each command.
 *
 * - element :
 * Indicates the parts with the header and footer removed, e.g. "000"
 * The element encoder transforms between packet strings and each element type.
 * The element encoder is defined by notation (signed decimal, hexadecimal, etc.),
 * Knows the format for each element (signed decimal is [+-][0-9]+).
 *
 * Each encoder has a parent-child relationship of frame->data->element.
 * Also, the design of encoder and decoder is symmetric.
 *
 * RobotHW updates the values of PacketData
 * Calls the Encode method of the frame encoder to perform encoding.
 *
 */
class PowerEcuComFrameEncoder {
 public:
  typedef boost::shared_ptr<hsrb_power_ecu::IPowerEcuComDataEncoder> DataEncoderType;

 private:
  PowerEcuComFrameEncoder(PowerEcuComFrameEncoder const &);             // = delete;
  PowerEcuComFrameEncoder &operator=(PowerEcuComFrameEncoder const &);  // = delete;

  typedef boost::unordered_map<std::string, DataEncoderType> DataEncoderMap;  //!< Dictionary of data encoders

 public:
  /**
   * @brief Constructor
   */
  PowerEcuComFrameEncoder();
  /**
   * @brief Destructor
   */
  ~PowerEcuComFrameEncoder();
  /**
   * @brief Encode
   * @param[out] buffer Output destination buffer
   * @param[in] packet_name Name of the packet to encode for data search
   * @return On success boost::system::error::success
   */
  boost::system::error_code Encode(PacketBuffer &buffer, const std::string &packet_name);
  /**
   * @brief Register data encoder
   * @param[in] encoder Encoder to register
   * @return On success boost::system::errc::success
   */
  boost::system::error_code RegisterDataEncoder(DataEncoderType encoder);

  /**
   * @brief Get the pointer to data
   *
   * @tparam T Data type
   * @param[in] name Data name
   *
   * @return On success: Pointer to data<br>
   * On failure: NULL<br>
   * Fails if unregistered data name or data type mismatch occurs
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    BOOST_FOREACH (DataEncoderMap::value_type const encoder, data_encoder_map_) {
      T* ret = encoder.second->GetParamPtr<T>(name);
      if (ret != NULL) {
        return ret;
      }
    }
    return NULL;
  }

 private:
  DataEncoderMap data_encoder_map_;         //!< Dictionary of data encoders
  uint32_t check_sum_;                      //!< Temporary storage data for checksum calculation
  IElementEncoder::Ptr check_sum_encoder_;  //!< Encoder to convert checksum value to string
};
}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_FRAME_ENCODER_HPP_

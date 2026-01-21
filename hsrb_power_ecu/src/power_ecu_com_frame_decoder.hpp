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
#ifndef HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_
#define HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_

#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/unordered/unordered_map.hpp>

#include "any_type_pointer_map.hpp"
#include "power_ecu_com_common.hpp"

namespace hsrb_power_ecu {
/**
 * @brief Interface for packet element decoder
 */
class IElementDecoder {
 public:
  /**
   * @brief Smart pointer for IElementDecoder
   */
  typedef boost::shared_ptr<IElementDecoder> Ptr;
  /**
   * @brief Destructor
   */
  virtual ~IElementDecoder() {}
  /**
   * @brief Decode
   * @param[in] start_iterator Start of the packet
   * @param[in] end_iterator End of the packet
   * @return True on successful decode
   */
  virtual bool Decode(const PacketBuffer::const_iterator start_iterator,
                      const PacketBuffer::const_iterator end_iterator) = 0;
};

/**
 * @brief Interface for data decoder
 */
class IPowerEcuComDataDecoder {
 private:
  IPowerEcuComDataDecoder(IPowerEcuComDataDecoder const&);             // = delete;
  IPowerEcuComDataDecoder& operator=(IPowerEcuComDataDecoder const&);  // = delete;

 protected:
  /**
   * @brief Constructor
   * @param packet_size Size of the packet header
   * @param packet_name Type of the packet header
   */
  IPowerEcuComDataDecoder(const size_t packet_size, const std::string& packet_name)
      : packet_size_(packet_size), packet_name_(packet_name) {}

 public:
  /**
   * @brief Destructor
   */
  virtual ~IPowerEcuComDataDecoder() {}

  /**
   * @brief Decode
   * @param[in] start_iterator Start iterator of the packet data section
   * @param[in] end_iterator End iterator of the packet data section
   * @return
   * Normal termination boost::system::errc::success
   * Decode failure boost::system::errc::protocol_error
   */
  virtual boost::system::error_code Decode(const PacketBuffer::const_iterator start_iterator,
                                           const PacketBuffer::const_iterator end_iterator);
  /**
   * @brief Get the size of the packet header
   * @return Packet size
   */
  inline size_t GetPacketSize() const { return packet_size_; }

  /**
   * @brief Get the type of the packet header
   * @return Packet type
   */
  inline std::string GetPacketName() const { return packet_name_; }

  /**
   * @brief Get the pointer to the data
   *
   * @tparam T Type of the data
   * @param[in] name Name of the data
   *
   * @return On success: Pointer to the data<br>
   *         On failure: NULL<br>
   *         Failure occurs if the data name is unregistered or the data type does not match
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    return parameter_map_.GetPtr<T>(name);
  }

 protected:
  /**
   * @brief Post-processing
   * Function called after the Decode function
   * @return True on success
   */
  inline virtual bool Update() { return true; }

  std::vector<IElementDecoder::Ptr> element_decoder_list_;  //!< List of decode instructions for each element
  const size_t packet_size_;                                //!< Size of the packet
  const std::string packet_name_;                           //!< Type of the packet
  any_type_pointer_map::Map parameter_map_;                 //!< Parameter map for control commands
};

/**
 * @brief Frame decoder
 * Classifies the communication format into elements of frame, data, and element.
 *
 * Example) For the command "H,ledc_,23,000,100,255,h12345678,\0",
 * - frame :
 *   Refers to the entire communication packet Example) "H,ledc_,23,000,100,255,h12345678,\0"
 *   The frame decoder is responsible for calculating the header and footer of the command,
 *   The frame decoder knows the specifications of the header and footer of the communication packet,
 *   Manages the data decoder with the command name as the key.
 *
 * - data :
 *   Refers to the part with the header and footer of the communication packet removed, Example) "000,100,255,"
 *   The data decoder divides into elements of data and calls the element decoder.
 *   Also, after decoding all elements, performs post-processing such as conversion to physical quantities.
 *   The data decoder is defined for each command, and knows the command name, command size,
 *   the structure of elements (order, type, and digit count of each element),
 *   and the reference to PacketData where the decoded information is stored.
 *   Also, for each command, holds the information to be stored after decoding as PacketData type
 *
 * - element :
 *   Refers to the part with the header and footer of the communication packet removed Example) "000"
 *   The element decoder converts between packet strings and types of each element.
 *   The element decoder is defined for each notation (signed decimal, hexadecimal, etc.),
 *   and knows the format of each element (signed decimal is [+-][0-9]+).
 *
 * Each decoder has a parent-child relationship of frame->data->element.
 * Also, the design of the decoder and encoder is symmetric.
 *
 * RobotHW decodes by calling the Decode method of the frame encoder on the receive buffer.
 * The decode result is automatically stored in PacketData linked to the data decoder.
 *
 */
class PowerEcuComFrameDecoder {
 public:
  typedef boost::shared_ptr<hsrb_power_ecu::IPowerEcuComDataDecoder> DataDecoderType;

 private:
  PowerEcuComFrameDecoder(PowerEcuComFrameDecoder const&);                    // = delete;
  PowerEcuComFrameDecoder& operator=(PowerEcuComFrameDecoder const&);         // = delete;
  typedef boost::unordered_map<std::string, DataDecoderType> DataDecoderMap;  //!< Dictionary type for data decoders

 public:
  /**
   * @brief Constructor
   */
  PowerEcuComFrameDecoder();
  /**
   * @brief Decode
   * Reads from the start of the packet receive buffer and decodes the first packet.
   * Does not modify the packet receive buffer and returns the decoded index with encoded_iterator.
   * @param[in] start_iterator Start iterator of the decode target
   * @param[in] end_iterator End iterator of the decode target
   * @param[out] encoded_iterator Iterator indicating the decoded location
   * @return
   * Normal termination boost::system::errc::success
   * Decode complete boost::system::errc::result_out_of_range
   * There was a packet that could not be decoded boost::system::errc::protocol_error
   */
  boost::system::error_code Decode(const hsrb_power_ecu::PacketBuffer::const_iterator& start_iterator,
                                   const hsrb_power_ecu::PacketBuffer::const_iterator& end_iterator,
                                   hsrb_power_ecu::PacketBuffer::const_iterator& encoded_iterator);
  /**
   * @brief Register data decoder
   * @param [in] decoder Decoder to register
   * @return
   * On success boost::system::errc::success
   * Already registered or nullptr boost::system::errc::invalid_argument
   */
  boost::system::error_code RegisterDataDecoder(DataDecoderType decoder);

  /**
   * @brief Get the pointer to the data
   *
   * @tparam T Type of the data
   * @param[in] name Name of the data
   *
   * @return On success: Pointer to the data<br>
   *         On failure: NULL<br>
   *         Failure occurs if the data name is unregistered or the data type does not match
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    BOOST_FOREACH (DataDecoderMap::value_type const decoder, data_decoder_map_) {
      T* ret = decoder.second->GetParamPtr<T>(name);
      if (ret != NULL) {
        return ret;
      }
    }
    return NULL;
  }

 private:
  /**
   * @brief Get the string
   * @param[in] start_it Start iterator
   * @param[in] end_it End iterator
   * @param[in] size Read size
   * @param[out] output_string Storage for the obtained string
   * @return True on success
   */
  bool GetString(const size_t size, const hsrb_power_ecu::PacketBuffer::const_iterator& end_it,
                 hsrb_power_ecu::PacketBuffer::const_iterator& start_it, std::string& output_string);
  /**
   * @brief Skip the string
   * @param[in] skip_string String to skip
   * @param[out] it Start iterator
   * @return True if the skipped string is equal to the string received as an argument
   */
  bool SkipString(const std::string& skip_string, hsrb_power_ecu::PacketBuffer::const_iterator& it);

  DataDecoderMap data_decoder_map_;      //!< Dictionary of data decoders
  std::string packet_name_buffer_;       //!< Buffer for storing header name during packet decode
  std::string packet_size_buffer_;       //!< Buffer for storing packet size during packet decode
  std::string packet_check_sum_buffer_;  //!< Buffer for storing checksum during packet decode
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_POWER_ECU_COM_FRAME_DECODER_HPP_

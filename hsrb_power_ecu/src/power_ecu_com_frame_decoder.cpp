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
#include "power_ecu_com_frame_decoder.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>

#include <boost/algorithm/minmax_element.hpp>
#include <boost/crc.hpp>
#include <boost/foreach.hpp>

#include <rclcpp/rclcpp.hpp>

#include "power_ecu_com_common.hpp"
#include "ros2_msg_utils.hpp"


namespace hsrb_power_ecu {
/**
 * @brief Decode
 * @param[in] start_iterator Iterator at the beginning of the packet data part
 * @param[in] end_iterator Iterator at the end of the packet data part
 * @return
 * Successful completion boost::system::errc::success
 * Decode failure boost::system::errc::protocol_error
 */
boost::system::error_code IPowerEcuComDataDecoder::Decode(const PacketBuffer::const_iterator start_iterator,
                                                          const PacketBuffer::const_iterator end_iterator) {
  // Start decoding
  PacketBuffer::const_iterator current_iterator = start_iterator;
  BOOST_FOREACH (IElementDecoder::Ptr const decoder, element_decoder_list_) {
    // Get iterator of the next comma
    PacketBuffer::const_iterator const element_end_iterator = std::find(current_iterator, end_iterator, ',');
    // Return error if comma is not found
    if (element_end_iterator == end_iterator) {
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // Decode element
    bool const ret = decoder->Decode(current_iterator, element_end_iterator);

    if (!ret) {
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // Set iterator following the comma as the starting iterator
    current_iterator = element_end_iterator + 1;
  }

  if (!Update()) {
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);  // TODO(kitsunai): テスト未実施
  }
  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief Constructor
 */
PowerEcuComFrameDecoder::PowerEcuComFrameDecoder()
    : data_decoder_map_(), packet_name_buffer_(), packet_size_buffer_(), packet_check_sum_buffer_() {
  // Allocate buffer
  packet_name_buffer_.reserve(hsrb_power_ecu::com_common::kPacketNameBufferSize);
  packet_size_buffer_.reserve(hsrb_power_ecu::com_common::kPacketSizeBufferSize);
  packet_check_sum_buffer_.reserve(hsrb_power_ecu::com_common::kPacketCheckSumBufferSize);
}

/**
 * @brief Decode
 * Read from the start of the packet receive buffer and decode the first packet
 * Does not modify the packet receive buffer; returns the decoded index with encoded_iterator
 * @param[in] start_iterator Iterator at the beginning of the target to decode
 * @param[in] end_iterator Iterator at the end of the target to decode
 * @param[out] encoded_iterator Iterator indicating the location of the decoded data
 * @return
 * Successful completion boost::system::errc::success
 * Decode complete boost::system::errc::result_out_of_range
 * There was a packet that could not be decoded boost::system::errc::protocol_error
 */
boost::system::error_code PowerEcuComFrameDecoder::Decode(
    const hsrb_power_ecu::PacketBuffer::const_iterator& start_iterator,
    const hsrb_power_ecu::PacketBuffer::const_iterator& end_iterator,
    hsrb_power_ecu::PacketBuffer::const_iterator& encoded_iterator) {
  // Search for the packet's start character
  hsrb_power_ecu::PacketBuffer::const_iterator current_iterator = std::find(start_iterator, end_iterator, 'E');
  if (current_iterator == end_iterator) {
    // 'E' is not present in the buffer = No message being read
    encoded_iterator = end_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  }
  hsrb_power_ecu::PacketBuffer::const_iterator const last_iterator = current_iterator;

  // Header analysis
  size_t packet_size = 0;
  //// Confirm if reception of the header size has been completed
  if (static_cast<uint32_t>(std::distance(current_iterator, end_iterator)) <
      hsrb_power_ecu::com_common::kPacketHeaderLength) {
    encoded_iterator = last_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  } else {
    // Packet header
    if (!SkipString("E,", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // Packet type
    //// Since the buffer size has been checked in advance, false return is not expected
    bool ret;
    ret = GetString(5, end_iterator, current_iterator, packet_name_buffer_);
    hsrb_power_ecu::Assert(ret, "GetString return false.");
    if (!SkipString(",", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    // Packet size
    ret = GetString(3, end_iterator, current_iterator, packet_size_buffer_);
    hsrb_power_ecu::Assert(ret, "GetString return false.");
    packet_size = std::atoi(packet_size_buffer_.c_str());
    if (packet_size < 11) {  // Packet less than footer length(11) is not possible by specification
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
    if (!SkipString(",", current_iterator)) {
      encoded_iterator = current_iterator;
      return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
    }
  }

  // Return result_out_of_range if packet reception is incomplete
  if (static_cast<uint32_t>(std::distance(current_iterator, end_iterator)) < packet_size) {
    encoded_iterator = last_iterator;
    return boost::system::errc::make_error_code(boost::system::errc::result_out_of_range);
  }

  // Footer analysis
  size_t const footer_length = hsrb_power_ecu::com_common::kPacketFooterLength;
  size_t const frame_size = hsrb_power_ecu::com_common::kPacketHeaderLength + packet_size;

  // Packet checksum calculation
  uint32_t const check_sum_calc = hsrb_power_ecu::com_common::CalculateCrc32(
      last_iterator,                                    // Beginning character string
      current_iterator + packet_size - footer_length);  // Iterator following the read character

  // Get checksum string
  hsrb_power_ecu::PacketBuffer::const_iterator checksum_iterator =
      current_iterator + packet_size - footer_length + 1;  // Do not read "h" at the head of checksum string
  bool ret = GetString(footer_length - 3,              // Do not read "h" + ",\n"
                       end_iterator, checksum_iterator, packet_check_sum_buffer_);
  hsrb_power_ecu::Assert(ret, "GetString resturn false.");
  uint32_t const check_sum_res = static_cast<uint32_t>(std::strtol(packet_check_sum_buffer_.c_str(), NULL, 16));

  // Compare checksum
  if (check_sum_calc != check_sum_res) {
    encoded_iterator = last_iterator + frame_size;
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
  }

  // Search for matching decoder
  DataDecoderMap::iterator const map_it = data_decoder_map_.find(packet_name_buffer_);

  // Return protocol_error if no matching decoder is found
  if ((map_it == data_decoder_map_.end()) || (map_it->second->GetPacketSize() != packet_size)) {
    encoded_iterator = last_iterator + frame_size;
    return boost::system::errc::make_error_code(boost::system::errc::protocol_error);
  }

  // Decode
  boost::system::error_code const result =
      map_it->second->Decode(current_iterator,
                             (current_iterator + packet_size - footer_length));  // Iterator following the read character

  encoded_iterator = last_iterator + frame_size;
  return result;
}

/**
 * @brief Data decoder registration
 * @param [in] decoder Decoder to be registered
 * @return
 * Success boost::system::errc::success
 * Already registered or nullptr boost::system::errc::invalid_argument
 */
boost::system::error_code PowerEcuComFrameDecoder::RegisterDataDecoder(DataDecoderType decoder) {
  if (decoder == NULL || data_decoder_map_.find(decoder->GetPacketName()) != data_decoder_map_.end()) {
    return boost::system::errc::make_error_code(boost::system::errc::invalid_argument);
  }

  data_decoder_map_[decoder->GetPacketName()] = decoder;

  return boost::system::errc::make_error_code(boost::system::errc::success);
}

/**
 * @brief Get string
 * @param[in] start_it Starting iterator
 * @param[in] end_it Endpoint iterator
 * @param[in] size Reading size
 * @param[out] output_string Storage location of the obtained string
 * @return True if successful
 */
bool PowerEcuComFrameDecoder::GetString(const size_t size, const hsrb_power_ecu::PacketBuffer::const_iterator& end_it,
                                        hsrb_power_ecu::PacketBuffer::const_iterator& start_it,
                                        std::string& output_string) {
  size_t const distance = std::distance(start_it, end_it);
  if (size > distance) {
    return false;
  }

  // Display warning if internal buffer of string type is insufficient
  if (output_string.capacity() < distance) {
    output_string.reserve(distance + 1);
  }

  output_string.clear();

  std::copy(start_it, start_it + size, std::back_inserter(output_string));
  start_it += size;
  return true;
}

/**
 * @brief Skip string
 * @param[in] skip_string String to be skipped
 * @param[out] it Starting iterator
 * @return true if the skipped string is equal to the string received in the argument
 */
bool PowerEcuComFrameDecoder::SkipString(const std::string& skip_string,
                                         hsrb_power_ecu::PacketBuffer::const_iterator& it) {
  bool is_equal = true;
  for (size_t i = 0; i < skip_string.length(); ++i) {
    is_equal &= (static_cast<char>(*it) == skip_string[i]);
    ++it;
  }
  return is_equal;
}

}  // namespace hsrb_power_ecu

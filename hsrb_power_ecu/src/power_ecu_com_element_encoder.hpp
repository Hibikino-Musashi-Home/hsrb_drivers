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
#ifndef HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_
#define HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_

#include <algorithm>
#include <string>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include "power_ecu_com_frame_encoder.hpp"

namespace hsrb_power_ecu {

/**
 * @brief Encoder for hexadecimal packets
 * Internally, conversion using uint32_t is performed, therefore,
 * The upper limit of the number of digits is 8
 */
template <class Input>
class ElementHexUintEncoder : public IElementEncoder {
 private:
  ElementHexUintEncoder(ElementHexUintEncoder const&);             // = delete;
  ElementHexUintEncoder& operator=(ElementHexUintEncoder const&);  // = delete;

 public:
  /**
   * @brief Constructor
   * @param value The original value for conversion
   * @param length The number of digits in the converted string
   */
  ElementHexUintEncoder(const Input& value, const size_t length) : value_(value), length_(length) {
    convert_buffer_.reserve(length + 2);  // Number of digits + \0 + 'h'
  }
  /**
   * @brief Destructor
   */
  virtual ~ElementHexUintEncoder() {}

  /**
   * @brief Encode
   * @param[out] buffer Buffer for output
   * @return true upon successful encoding
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    uint32_t current_value = static_cast<uint32_t>(value_);
    convert_buffer_.clear();

    // Create hexadecimal characters from the lower digit
    for (size_t i = 0; i < length_; ++i) {
      uint32_t v = current_value % 0x10;
      char h;
      if (v > 15) {
        h = '0';  // TODO(kitsunai): 未テスト
      } else if (v < 10) {
        h = v + '0';
      } else {
        h = v + 'A' - 10;
      }
      convert_buffer_.push_back(h);
      current_value /= 0x10;
    }
    convert_buffer_.push_back('h');
    if (current_value != 0) {
      return false;
    }

    // Assign in reverse
    std::copy(convert_buffer_.rbegin(), convert_buffer_.rend(), std::back_inserter(buffer));
    return true;
  }

 private:
  const Input& value_;          //!< Value to encode
  const size_t length_;         //!< Number of digits
  std::string convert_buffer_;  //!< Buffer for conversion
};

/**
 * @brief A class to create hexadecimal messages with specified bits
 * Internally, conversion using uint32_t is performed, therefore,
 * The upper limit of the number of digits is 8
 */
class ElementHexUintBitsEncoder : public ElementHexUintEncoder<uint32_t> {
 private:
  ElementHexUintBitsEncoder(ElementHexUintBitsEncoder const&);             // = delete;
  ElementHexUintBitsEncoder& operator=(ElementHexUintBitsEncoder const&);  // = delete;

  typedef boost::unordered_map<uint32_t, const bool*> BitmapType;

 public:
  /**
   * @brief Constructor
   * @param length The number of digits in the converted string
   */
  explicit ElementHexUintBitsEncoder(const size_t length)
      : ElementHexUintEncoder<uint32_t>(value_, length), length_(length) {}
  /**
   * @brief Destructor
   */
  virtual ~ElementHexUintBitsEncoder() {}

  /**
   * @brief Encode
   * @param[out] buffer Buffer for output
   * @return true upon successful encoding
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    value_ = 0;
    BOOST_FOREACH (BitmapType::value_type const pair, bit_map_) {
      if (*pair.second) {
        value_ += (1 << pair.first);
      }
    }
    return ElementHexUintEncoder<uint32_t>::Encode(buffer);
  }

  /**
   * @brief Register the bit address and boolean value to be evaluated
   *
   * @param bit Address when the least significant bit during transmission (considered the most significant bit in the communication specification) is 0
   * @param flag Boolean value to be evaluated
   *
   * @return
   */
  inline bool RegisterBit(const uint32_t bit, const bool* const flag) {
    if (bit > ((length_ * 4) - 1)) {
      return false;  // TODO(kitsunai): 未テスト
    }
    if (bit_map_.count(bit) != 0) {
      return false;  // TODO(kitsunai): 未テスト
    }
    bit_map_[bit] = flag;
    return true;
  }

 private:
  size_t length_;
  BitmapType bit_map_;  //!< Map of bit addresses and flag values
  uint32_t value_;      //!< Buffer
};

/**
 * @brief Encoder for decimal packets
 * Internally, conversion using uint32_t is performed, therefore,
 * The upper limit of the number of digits is 9
 */
template <class Input>
class ElementUintEncoder : public IElementEncoder {
 private:
  ElementUintEncoder(ElementUintEncoder const&);             // = delete;
  ElementUintEncoder& operator=(ElementUintEncoder const&);  // = delete;

 public:
  /**
   * @brief Constructor
   * @param value The original value for conversion
   * @param length The number of digits in the converted string
   */
  ElementUintEncoder(const Input& value, const size_t length) : value_(value), length_(length) {
    convert_buffer_.reserve(length + 1);  // Number of digits + \0
  }
  /**
   * @brief Destructor
   */
  virtual ~ElementUintEncoder() {}

  /**
   * @brief Encode
   * @param[out] buffer Buffer for output
   * @return true upon successful encoding
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    uint32_t current_value = static_cast<uint32_t>(value_);
    convert_buffer_.clear();

    // Create from the lower digit
    for (size_t i = 0; i < length_; ++i) {
      uint32_t v = current_value % 10;
      char d;
      if (v > 10) {
        d = '0';  // TODO(kitsunai): 未テスト
      } else {
        d = v + '0';
      }
      convert_buffer_.push_back(d);
      current_value /= 10;
    }
    if (current_value != 0) {
      return false;
    }

    // Assign in reverse
    std::copy(convert_buffer_.rbegin(), convert_buffer_.rend(), std::back_inserter(buffer));
    return true;
  }

 private:
  const Input& value_;          //!< Value to encode
  const size_t length_;         //!< Number of digits
  std::string convert_buffer_;  //!< Buffer for conversion
};

class ElementStringEncoder : public IElementEncoder {
 private:
  ElementStringEncoder(ElementStringEncoder const&);             // = delete;
  ElementStringEncoder& operator=(ElementStringEncoder const&);  // = delete;

 public:
  /**
   * @brief Constructor
   * @param value The original value for conversion
   * @param length The number of digits in the converted string
   */
  ElementStringEncoder(const std::string& value, const size_t length) : value_(value), length_(length) {}
  /**
   * @brief Destructor
   */
  virtual ~ElementStringEncoder() {}

  /**
   * @brief Encode
   * @param[out] buffer Buffer for output
   * @return true upon successful encoding
   */
  inline virtual bool Encode(PacketBuffer& buffer) {
    if (value_.size() != length_) return false;
    std::copy(value_.begin(), value_.end(), std::back_inserter(buffer));
    return true;
  }

 private:
  const std::string& value_;  //!< Value to encode
  const size_t length_;       //!< Number of digits
};
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_POWER_ECU_COM_ELEMENT_ENCODER_HPP_

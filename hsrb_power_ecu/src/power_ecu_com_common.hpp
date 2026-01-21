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
#ifndef POWER_ECU_COM_COMMON_HPP_
#define POWER_ECU_COM_COMMON_HPP_

#include <stdint.h>
#include <string>

#include <hsrb_power_ecu/i_network.hpp>

namespace hsrb_power_ecu {
/**
 * @brief Common information for control command encoder and decoder
 */
namespace com_common {

/**
 * @brief crc32 checksum calculation
 * @param[in] start_it Start iterator of the checksum string
 * @param[in] end_it End iterator of the checksum string
 * @return Checksum result
 */
uint32_t CalculateCrc32(std::string::const_iterator start_it, std::string::const_iterator end_it);

/**
 * @brief crc32 checksum calculation
 * @param start_it Start iterator of the checksum string
 * @param end_it End iterator of the checksum string
 * @return Checksum result
 */
uint32_t CalculateCrc32(hsrb_power_ecu::PacketBuffer::const_iterator start_it,
                        hsrb_power_ecu::PacketBuffer::const_iterator end_it);

const size_t kPacketNameBufferSize = 5 + 1;      //!< Number of characters in the packet header type (5) + '\0'
const size_t kPacketSizeBufferSize = 3 + 1;      //!< Number of characters in the packet header size (3) + '\0'
const size_t kPacketCheckSumBufferSize = 8 + 1;  //!< Number of characters in the packet data checksum (8) + '\0'
const size_t kPacketHeaderLength = 12;           //!< Number of characters in the packet header
const size_t kPacketFooterLength = 11;           //!< Number of characters in the packet footer

}  // namespace com_common
}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_COMMON_HPP_

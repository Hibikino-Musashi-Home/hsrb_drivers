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
#ifndef POWER_ECU_COM_DATA_ENCODER_HPP_
#define POWER_ECU_COM_DATA_ENCODER_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/system/error_code.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_encoder.hpp"
#include "power_ecu_com_frame_encoder.hpp"

namespace hsrb_power_ecu {
/**
 * @brief Container for color specification
 *
 * @tparam T Type of color information
 */
template <typename T>
struct Color {
  T r;
  T g;
  T b;
};

// TODO(kitsunai): 送信コマンドで未実装の物がある
// Power shutdown command       pdown, performs shutdown of the power ECU
// Basic information read command           info1, reads basic information of the power ECU once
// Optional information read command     info2, reads optional information of the power ECU once
// Reprogram start command         rpros, starts reprogramming
// Reprogram data command       rprod, sends data for reprogramming
// Reprogram end command         rproe, ends reprogramming

/**
 * @brief Clock synchronization command
 */
class PowerEcuComTimeDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComTimeDataEncoder(PowerEcuComTimeDataEncoder const&);             // = delete;
  PowerEcuComTimeDataEncoder& operator=(PowerEcuComTimeDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    std::string start_time;  //!< Current time
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComTimeDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComTimeDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Basic information periodic transmission start command start, starts continuous transmission of the power ECU
class PowerEcuComStartDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComStartDataEncoder(PowerEcuComStartDataEncoder const&);             // = delete;
  PowerEcuComStartDataEncoder& operator=(PowerEcuComStartDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_enable_ecu1;  //!< Whether to periodically transmit ECU1
    bool is_enable_ecu2;  //!< Whether to periodically transmit ECU2
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComStartDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComStartDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Basic information periodic transmission stop command stop_, stops continuous transmission of the power ECU
class PowerEcuComStopDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComStopDataEncoder(PowerEcuComStopDataEncoder const&);             // = delete;
  PowerEcuComStopDataEncoder& operator=(PowerEcuComStopDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComStopDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComStopDataEncoder() {}
};

// Heartbeat                     heart, main CPU checks the survival of the power ECU
class PowerEcuComHeartDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComHeartDataEncoder(PowerEcuComHeartDataEncoder const&);             // = delete;
  PowerEcuComHeartDataEncoder& operator=(PowerEcuComHeartDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    uint16_t error_state;  //!< Error state                4-digit hexadecimal uint16
    uint32_t counts;       //!< Count value (+1 for each transmission) 8-digit hexadecimal uint32
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComHeartDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComHeartDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Pump switch                   pump_, controls the pump switch
class PowerEcuComPumpDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPumpDataEncoder(PowerEcuComPumpDataEncoder const&);             // = delete;
  PowerEcuComPumpDataEncoder& operator=(PowerEcuComPumpDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    uint8_t is_pump_enable;  //!< Pump switch (0:OFF 1:ON) 1-digit decimal uint8
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComPumpDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComPumpDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Drive system switch                   pbmsw, controls the power of the drive system
class PowerEcuComPbmswDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPbmswDataEncoder(PowerEcuComPbmswDataEncoder const&);             // = delete;
  PowerEcuComPbmswDataEncoder& operator=(PowerEcuComPbmswDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    uint8_t is_motor_enable;  //!< Drive system switch (0 : OFF 1 : ON) 1-digit decimal uint8
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComPbmswDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComPbmswDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Multi-purpose LED color specification                ledc_, specifies lighting/blinking and color of the multi-purpose LED
class PowerEcuComLedcDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComLedcDataEncoder(PowerEcuComLedcDataEncoder const&);             // = delete;
  PowerEcuComLedcDataEncoder& operator=(PowerEcuComLedcDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    Color<uint8_t> led_color;  //!< Color intensity (0-255) 3-digit decimal uint8
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComLedcDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComLedcDataEncoder() {}

 private:
  PacketData packet_data_;  //!< Packet data
};

// Attitude angle calculation reset (g_res)
class PowerEcuComGResDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComGResDataEncoder(PowerEcuComGResDataEncoder const&);             // = delete;
  PowerEcuComGResDataEncoder& operator=(PowerEcuComGResDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_quaternion_reset;
    bool is_gyro_reset;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComGResDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComGResDataEncoder() {}

 private:
  PacketData packet_data_;
};

// Solenoid switch (solsw)
class PowerEcuComSolswDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComSolswDataEncoder(PowerEcuComSolswDataEncoder const&);             // = delete;
  PowerEcuComSolswDataEncoder& operator=(PowerEcuComSolswDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    uint8_t is_solenoid_enable;  //!< Solenoid switch (0:OFF 1:ON) 1-digit decimal uint8
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComSolswDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComSolswDataEncoder() {}

 private:
  PacketData packet_data_;
};

// Power shutdown (individual) command (pdcmd)
class PowerEcuComPdcmdDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComPdcmdDataEncoder(PowerEcuComPdcmdDataEncoder const&);             // = delete;
  PowerEcuComPdcmdDataEncoder& operator=(PowerEcuComPdcmdDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_cpu_shutdown;
    bool is_gpu_shutdown;
    bool is_ex1_shutdown;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComPdcmdDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComPdcmdDataEncoder() {}

 private:
  PacketData packet_data_;
};

// Audio amplifier MUTE (mute_)
class PowerEcuComMuteDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComMuteDataEncoder(PowerEcuComMuteDataEncoder const&);             // = delete;
  PowerEcuComMuteDataEncoder& operator=(PowerEcuComMuteDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_amp_mute;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComMuteDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComMuteDataEncoder() {}

 private:
  PacketData packet_data_;
};

// Version information acquisition command (getv_)
class PowerEcuComGetvDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComGetvDataEncoder(PowerEcuComGetvDataEncoder const&);             // = delete;
  PowerEcuComGetvDataEncoder& operator=(PowerEcuComGetvDataEncoder const&);  // = delete;

  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    uint8_t reserved;  //!< Version type (always 0) 1-digit hexadecimal uint8
  };

 public:
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComGetvDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComGetvDataEncoder() {}

 private:
  PacketData packet_data_;
};

// Undock command (undck)
class PowerEcuComUndckDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuComUndckDataEncoder(PowerEcuComUndckDataEncoder const&);             // = delete;
  PowerEcuComUndckDataEncoder& operator=(PowerEcuComUndckDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_undck;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComUndckDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComUndckDataEncoder() {}

 private:
  PacketData packet_data_;
};

// 12Vu_ enable command (12vu_)
class PowerEcuCom12VuDataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom12VuDataEncoder(PowerEcuCom12VuDataEncoder const&);             // = delete;
  PowerEcuCom12VuDataEncoder& operator=(PowerEcuCom12VuDataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_12vu_enable;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuCom12VuDataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuCom12VuDataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd3_ enable command (5vd3_)
class PowerEcuCom5Vd3DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd3DataEncoder(PowerEcuCom5Vd3DataEncoder const&);             // = delete;
  PowerEcuCom5Vd3DataEncoder& operator=(PowerEcuCom5Vd3DataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_5vd3_enable;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuCom5Vd3DataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuCom5Vd3DataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd4_ enable command (5vd4_)
class PowerEcuCom5Vd4DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd4DataEncoder(PowerEcuCom5Vd4DataEncoder const&);             // = delete;
  PowerEcuCom5Vd4DataEncoder& operator=(PowerEcuCom5Vd4DataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_5vd4_enable;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuCom5Vd4DataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuCom5Vd4DataEncoder() {}

 private:
  PacketData packet_data_;
};

// 5Vd5_ enable command (5vd5_)
class PowerEcuCom5Vd5DataEncoder : public IPowerEcuComDataEncoder {
 private:
  PowerEcuCom5Vd5DataEncoder(PowerEcuCom5Vd5DataEncoder const&);             // = delete;
  PowerEcuCom5Vd5DataEncoder& operator=(PowerEcuCom5Vd5DataEncoder const&);  // = delete;

 public:
  /**
   * @brief Internal structure of the packet
   */
  struct PacketData {
    bool is_5vd5_enable;
  };
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuCom5Vd5DataEncoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuCom5Vd5DataEncoder() {}

 private:
  PacketData packet_data_;
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_DATA_ENCODER_HPP_
